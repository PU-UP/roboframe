#include <odom/odom_component.hpp>
#include <stdexcept>
#include <utility>

namespace odom {
OdomComponent::OdomComponent(Output output) : output_(std::move(output)) {
    if (!output_) throw std::invalid_argument("odometry output required");
}
void OdomComponent::Activate() {
    if (lifecycle_ == roboframe::Lifecycle::active) throw std::logic_error("odometry already active");
    has_latest_ = false;
    latest_ = {};
    accepted_ = 0;
    lifecycle_ = roboframe::Lifecycle::active;
}
void OdomComponent::Deactivate() {
    has_latest_ = false;
    latest_ = {};
    lifecycle_ = roboframe::Lifecycle::inactive;
}
roboframe::Lifecycle OdomComponent::lifecycle() const { return lifecycle_; }
void OdomComponent::Push(const roboframe::TimedPose2D& pose) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("odometry inactive");
    roboframe::ValidatePose(pose);
    latest_ = pose;
    has_latest_ = true;
    ++accepted_;
    output_(latest_);
}
bool OdomComponent::Latest(roboframe::TimedPose2D& pose) const {
    if (!has_latest_) return false;
    pose = latest_;
    return true;
}
} // namespace odom
