#pragma once
#include "integration/components.hpp"

namespace roboframe {
// Task coordination and output wiring. Component remains the sole owner of task status.
class DemoIntegration {
public:
    DemoIntegration();
    DemoIntegration(const DemoIntegration&) = delete;
    DemoIntegration& operator=(const DemoIntegration&) = delete;
    void SetCallbacks(sample_accumulator::Component::FeedbackCallback feedback,
                      sample_accumulator::Component::ResultCallback result);
    void Submit(const sample_accumulator::Goal& goal);
    void Cancel(GoalId id);
    void PushSample(sample_accumulator::Sample sample);
    TaskStatus status() const { return components_.accumulator.status(); }
    Lifecycle lifecycle() const { return components_.accumulator.lifecycle(); }
    void Activate();
    void Deactivate();
    void Shutdown();
private:
    Components components_;
    sample_accumulator::Component::FeedbackCallback feedback_;
    sample_accumulator::Component::ResultCallback result_;
};
} // namespace roboframe
