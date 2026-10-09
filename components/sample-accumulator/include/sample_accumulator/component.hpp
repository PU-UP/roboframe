#pragma once
#include <functional>
#include <roboframe/component.hpp>
#include "sample_accumulator/contract.hpp"

namespace sample_accumulator {
// Implemented in the library: metadata describes the actual compiled component.
roboframe::ComponentInfo GetComponentInfo();

// Single caller thread. Callbacks must not throw, reenter, or destroy this component.
class Component final : public roboframe::Component {
public:
    using FeedbackCallback = std::function<void(const Feedback&)>;
    using ResultCallback = std::function<void(const Result&)>;
    void SetCallbacks(FeedbackCallback feedback, ResultCallback result);
    void Activate() override;
    void Deactivate() override;
    void Submit(const Goal& goal);
    void PushSample(Sample sample);
    void Cancel(roboframe::GoalId id);
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    roboframe::TaskStatus status() const { return task_.status(); }

private:
    void Complete(roboframe::TaskState state, Error error = Error::none);
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    roboframe::Task task_;
    std::uint64_t target_ = 0, count_ = 0;
    std::int64_t sum_ = 0;
    FeedbackCallback feedback_;
    ResultCallback result_;
};
} // namespace sample_accumulator
