#include <roboframe/log.hpp>
#include "integration/demo_integration.hpp"
#include <utility>

namespace roboframe {
DemoIntegration::DemoIntegration() {
    components_.accumulator.SetCallbacks(
        [this](const sample_accumulator::Feedback& value) {
            ROBO_LOG_INFO("integration") << "feedback -> output";
            if (feedback_) feedback_(value);
        },
        [this](const sample_accumulator::Result& value) {
            ROBO_LOG_INFO("integration") << "result -> output";
            if (result_) result_(value);
        });
    components_.Activate();
}

void DemoIntegration::SetCallbacks(sample_accumulator::Component::FeedbackCallback feedback,
                                   sample_accumulator::Component::ResultCallback result) {
    feedback_ = std::move(feedback);
    result_ = std::move(result);
}

void DemoIntegration::Submit(const sample_accumulator::Goal& goal) {
    ROBO_LOG_INFO("integration") << "submit goal=" << goal.id;
    components_.accumulator.Submit(goal);
}

void DemoIntegration::Cancel(GoalId id) {
    ROBO_LOG_INFO("integration") << "cancel goal=" << id;
    components_.accumulator.Cancel(id);
}

void DemoIntegration::Activate() { components_.Activate(); }
void DemoIntegration::Deactivate() { components_.accumulator.Deactivate(); }

void DemoIntegration::Shutdown() {
    components_.Shutdown();
    feedback_ = {};
    result_ = {};
    ROBO_LOG_INFO("integration") << "shutdown; all tasks terminal";
}
} // namespace roboframe
