#include "integration/robot_integration.hpp"
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
RobotIntegration::RobotIntegration(GridSpec spec, std::string directory, roboframe::PoseBufferConfig config)
    : pose_buffer_(config), odometry_([this](const TimedPose2D& pose) {
          RouteOdometry(pose);
      }), grid_(spec), store_(std::move(directory)) {
    CheckCompatibility(odom::GetComponentInfo(), Version{1, 0, 0});
    CheckCompatibility(map_updater::GetComponentInfo(), Version{1, 0, 0});
    CheckCompatibility(map_server::GetComponentInfo(), Version{1, 0, 0});
    // The only potentially failing activation comes first; later activations allocate no resources.
    store_.Activate();
    odometry_.Activate();
    grid_.Activate();
    active_ = true;
    ROBO_LOG_INFO("integration") << "modules active: odom -> map-updater -> map-server; SDK "
                             << ToString(sdk_version);
}
RobotIntegration::~RobotIntegration() { Shutdown(); }
void RobotIntegration::RequireActive() const {
    if (!active_) throw std::logic_error("integration stopped");
}
void RobotIntegration::Shutdown() {
    if (!active_) return;
    // EOF/exit abandons pending work; it never silently finishes or saves a map.
    if (status_.phase == MappingPhase::mapping) {
        status_.dropped += pending_.size();
        status_.drop_cancel += pending_.size();
        status_.phase = MappingPhase::idle;
    }
    pending_.clear();
    grid_.Deactivate();
    odometry_.Deactivate();
    pose_buffer_.Clear();
    store_.Deactivate();
    active_ = false;
}
} // namespace roboframe
