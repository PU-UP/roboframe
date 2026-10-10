#include "integration/robot_integration_impl.hpp"
#include <utility>
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
namespace {
void CheckComponent(const ComponentInfo& info, Version required) {
    CheckCompatibility(info, required);
    ROBO_LOG_INFO("component") << info.name << " implementation=" << info.implementation_version
        << " contract=" << ToString(info.contract) << " built_with_sdk=" << ToString(info.built_with_sdk);
}
}
RobotIntegration::RobotIntegration(std::string directory) : map_directory_(std::move(directory)) {}
RobotIntegration::~RobotIntegration() = default;
void RobotIntegration::Configure(MappingOptions options) {
    if (impl_) throw std::logic_error("already configured");
    impl_ = std::make_unique<Impl>(options, map_directory_);
}
RobotIntegration::Impl& RobotIntegration::Configured() const {
    if (!impl_) throw std::logic_error("CONFIG required");
    return *impl_;
}
void RobotIntegration::StartMapping(std::uint64_t id) { Configured().StartMapping(id); }
void RobotIntegration::FinishMapping() { Configured().FinishMapping(); }
void RobotIntegration::CancelMapping() { Configured().CancelMapping(); }
void RobotIntegration::SaveMap(const std::string& name) { Configured().SaveMap(name); }
void RobotIntegration::SnapshotMap(const std::string& name) { Configured().SnapshotMap(name); }
void RobotIntegration::LoadMap(const std::string& name) { Configured().LoadMap(name); }
void RobotIntegration::PushOdometry(const TimedPose2D& pose) { Configured().PushOdometry(pose); }
void RobotIntegration::PushScan(Scan2DPtr scan) { Configured().PushScan(std::move(scan)); }
void RobotIntegration::Shutdown() { if (impl_) impl_->Shutdown(); }
const MappingStatus& RobotIntegration::mapping_status() const { return Configured().status_; }
std::size_t RobotIntegration::pending() const { return Configured().pending_.size(); }
GridSnapshot RobotIntegration::map() const { return Configured().frozen_; }

RobotIntegration::Impl::Impl(MappingOptions options, std::string directory)
    : pose_buffer_({512, options.max_pose_gap_us}), odometry_([this](const TimedPose2D& pose) {
          RouteOdometry(pose);
      }), grid_(options.grid), store_(std::move(directory)) {
    CheckComponent(odom::GetComponentInfo(), Version{1, 0, 0});
    CheckComponent(map_updater::GetComponentInfo(), Version{2, 0, 0});
    CheckComponent(map_server::GetComponentInfo(), Version{1, 0, 0});
    // The only potentially failing activation comes first; later activations allocate no resources.
    store_.Activate();
    odometry_.Activate();
    grid_.Activate();
    active_ = true;
    ROBO_LOG_INFO("integration") << "modules active: odom -> map-updater -> map-server; SDK "
                             << ToString(sdk_version);
}
RobotIntegration::Impl::~Impl() { Shutdown(); }
void RobotIntegration::Impl::RequireActive() const {
    if (!active_) throw std::logic_error("integration stopped");
}
void RobotIntegration::Impl::Shutdown() {
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
