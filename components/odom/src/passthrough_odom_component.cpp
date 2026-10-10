#include <odom/passthrough_odom_component.hpp>
#include <stdexcept>
#include <utility>

namespace odom {
static_assert(roboframe::Compatible(roboframe::sdk_version, {1, 1, 0}), "Odom requires SDK >=1.1, same major");
roboframe::ComponentInfo GetComponentInfo() {
    return {"odom", ROBOFRAME_COMPONENT_VERSION, roboframe::sdk_version, contract_version};
}
PassthroughOdomComponent::PassthroughOdomComponent(Output output) : output_(std::move(output)) {
    if (!output_) throw std::invalid_argument("odometry output required");
}
void PassthroughOdomComponent::PushOdometry(const roboframe::TimedPose2D& pose) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("odometry inactive");
    roboframe::ValidatePose(pose);
    if (pose.stamp_us <= last_stamp_us_) throw std::invalid_argument("odometry timestamps must strictly increase");
    last_stamp_us_ = pose.stamp_us;
    output_(pose);
}
} // namespace odom
