#include <distance_goal/distance_goal_component.hpp>
#include <cmath>
#include <stdexcept>
#include <utility>
namespace distance_goal {
DistanceGoalComponent::DistanceGoalComponent(Output output)
    : output_(std::move(output)), execution_(std::chrono::milliseconds(10), {
          [this](const Goal& goal) { Start(goal); },
          [this](const Goal& goal, const Execution::Context& context) { return Step(goal, context); },
          [this] { if (output_) output_({}); }}) {}
DistanceGoalComponent::~DistanceGoalComponent() { execution_.Shutdown(); }
void DistanceGoalComponent::Activate() {
    std::lock_guard<std::mutex> lock(mutex_);
    execution_.Activate();
    meters_ = 0;
    active_ = true;
}
void DistanceGoalComponent::RequestDeactivate() {
    { std::lock_guard<std::mutex> lock(mutex_); active_ = false; }
    execution_.RequestDeactivate();
}
void DistanceGoalComponent::Deactivate() { RequestDeactivate(); execution_.WaitIdle(); }
roboframe::Lifecycle DistanceGoalComponent::lifecycle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_ ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
}
void DistanceGoalComponent::UpdateDistance(double meters) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) throw std::logic_error("distance goal inactive");
    if (!std::isfinite(meters) || meters < meters_)
        throw std::invalid_argument("distance must be finite and nondecreasing");
    meters_ = meters;
}
void DistanceGoalComponent::Start(const Goal& goal) {
    if (!std::isfinite(goal.meters) || goal.meters <= 0 || goal.timeout.count() <= 0)
        throw std::invalid_argument("invalid distance goal");
    deadline_ = Execution::Clock::now() + goal.timeout;
}
DistanceGoalComponent::Execution::Step DistanceGoalComponent::Step(
        const Goal& goal, const Execution::Context& context) {
    double meters;
    { std::lock_guard<std::mutex> lock(mutex_); meters = meters_; }
    if (context.canceled()) return {};
    if (meters < goal.meters && Execution::Clock::now() >= deadline_)
        throw std::runtime_error("distance goal timed out");
    Progress progress{true, meters, goal.meters};
    if (output_) output_(progress);
    return {meters >= goal.meters ? roboframe::TaskState::succeeded : roboframe::TaskState::running,
            progress, meters};
}
}
