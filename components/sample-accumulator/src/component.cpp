#include <roboframe/log.hpp>
#include "sample_accumulator/component.hpp"
#include <limits>
#include <stdexcept>
#include <utility>

namespace sample_accumulator {
static_assert(roboframe::Compatible(roboframe::sdk_version, {1, 0, 0}),
              "SampleAccumulator requires RoboframeSDK 1.x");

roboframe::ComponentInfo GetComponentInfo() {
    return {"sample-accumulator", SAMPLE_ACCUMULATOR_VERSION,
            roboframe::sdk_version, contract_version};
}

void Component::SetCallbacks(FeedbackCallback feedback, ResultCallback result) {
    feedback_ = std::move(feedback);
    result_ = std::move(result);
}

void Component::Activate() {
    lifecycle_ = roboframe::Lifecycle::active;
    ROBO_LOG_INFO("component") << "active";
}

void Component::Deactivate() {
    if (task_.busy()) throw std::logic_error("cancel task before deactivation");
    lifecycle_ = roboframe::Lifecycle::inactive;
    ROBO_LOG_INFO("component") << "inactive";
}

void Component::Submit(const Goal& goal) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("component inactive");
    if (goal.sample_count == 0) throw std::invalid_argument("sample count must be positive");
    task_.Begin(goal.id);
    target_ = goal.sample_count;
    count_ = 0;
    sum_ = 0;
    ROBO_LOG_INFO("component") << "accepted goal=" << goal.id << " samples=" << target_;
}

void Component::PushSample(Sample sample) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("component inactive");
    if (task_.status().state != roboframe::TaskState::running) throw std::logic_error("no running goal");
    const auto low = std::numeric_limits<std::int64_t>::min();
    const auto high = std::numeric_limits<std::int64_t>::max();
    if ((sample.value > 0 && sum_ > high - sample.value) ||
        (sample.value < 0 && sum_ < low - sample.value)) {
        Complete(roboframe::TaskState::failed, Error::overflow);
        return;
    }
    ++count_;
    sum_ += sample.value;
    ROBO_LOG_INFO("component") << "PushSample value=" << sample.value
              << " -> count=" << count_ << " sum=" << sum_;
    const Feedback feedback{task_.status().id, count_, sum_};
    if (feedback_) feedback_(feedback);
    if (count_ == target_) Complete(roboframe::TaskState::succeeded);
}

void Component::Cancel(roboframe::GoalId id) {
    task_.RequestCancel(id);
    // Synchronous computation has no queued work: cancellation is immediately complete.
    Complete(roboframe::TaskState::canceled);
}

void Component::Complete(roboframe::TaskState state, Error error) {
    task_.Finish(state);
    const Result result{task_.status().id, state, count_, sum_, error};
    ROBO_LOG_INFO("component") << "terminal goal=" << result.goal_id;
    if (result_) result_(result);
}
} // namespace sample_accumulator
