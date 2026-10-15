#include <path_distance/path_distance_component.hpp>
#include <cmath>
#include <stdexcept>
namespace path_distance {
PathDistanceComponent::PathDistanceComponent(std::size_t capacity)
    : capacity_(capacity), execution_(capacity, [this](const roboframe::TimedPose2D& pose,
          const roboframe::Cancellation& token) { return Compute(pose, token); }) {}
PathDistanceComponent::~PathDistanceComponent() { execution_.Stop(); }
void PathDistanceComponent::Activate() {
    if (active_ || retained()) throw std::logic_error("distance component not quiescent");
    sample_ = {};
    execution_.Reset(++epoch_);
    active_ = true;
}
void PathDistanceComponent::RequestDeactivate() { active_ = false; execution_.Reset(0); }
void PathDistanceComponent::Deactivate() {
    RequestDeactivate();
    execution_.WaitIdle();
    Execution::Result discard;
    while (execution_.Take(discard)) {}
}
roboframe::Lifecycle PathDistanceComponent::lifecycle() const {
    return active_ ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
}
roboframe::Admission PathDistanceComponent::Push(roboframe::TimedPose2D pose) {
    return active_ ? execution_.Submit(epoch_, pose) : roboframe::Admission::closed;
}
bool PathDistanceComponent::Take(Execution::Result& result) { return execution_.Take(result); }
std::size_t PathDistanceComponent::retained() const { return execution_.status().retained; }
bool PathDistanceComponent::CanAccept() const { return active_ && retained() < capacity_; }
Sample PathDistanceComponent::Compute(const roboframe::TimedPose2D& pose,
                                      const roboframe::Cancellation& token) {
    roboframe::ValidatePose(pose); // Worker errors are delivered by Take; previous state is preserved.
    if (token.canceled()) return {};
    if (sample_.count) sample_.meters += std::hypot(pose.pose.x - sample_.latest.pose.x,
                                                   pose.pose.y - sample_.latest.pose.y);
    sample_.latest = pose;
    ++sample_.count;
    return sample_;
}
}
