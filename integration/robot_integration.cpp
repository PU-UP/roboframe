#include "integration/robot_integration_impl.hpp"
#include <utility>
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
namespace {
void CheckComponent(const ComponentInfo& info) {
    CheckCompatibility(info);
    ROBO_LOG_INFO("component") << info.name << " source=" << info.source_id << " built_with_sdk=" << ToString(info.built_with_sdk);
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
void RobotIntegration::PushMapReference(const MapPose2D& pose) { Configured().PushMapReference(pose); }
TransformStatus2D RobotIntegration::transforms() const { return Configured().transforms_; }
void RobotIntegration::PushScan(Scan2DPtr scan) { Configured().PushScan(std::move(scan)); }
void RobotIntegration::Shutdown() { if (impl_) impl_->Shutdown(); }
const MappingStatus& RobotIntegration::mapping_status() const { return Configured().status_; }
std::size_t RobotIntegration::pending() const { return Configured().pending_.size(); }
GridSnapshot RobotIntegration::map() const { return Configured().frozen_; }
GridSnapshot RobotIntegration::live_map() const { return Configured().live_; }
DataFlowStatus RobotIntegration::data_flow_status() const { return Configured().Resources(); }
void RobotIntegration::WaitPlanner() { Configured().WaitPlanner(); }

RobotIntegration::Impl::Impl(MappingOptions options, std::string directory)
    : pose_buffer_({512, options.max_pose_gap_us}), odometry_([this](const TimedPose2D& pose) {
          RouteOdometry(pose);
      }), lidar_map_([this](const MapCorrection2D& value) {
          correction_ = value; has_correction_ = true;
      }), grid_(options.grid), store_(std::move(directory)), frame_(options.grid.frame),
      max_correction_age_us_(options.max_correction_age_us) {
    if (max_correction_age_us_ <= 0 || max_correction_age_us_ > 60000000)
        throw std::invalid_argument("invalid correction maximum age");
    CheckComponent(odom::GetComponentInfo());
    CheckComponent(lidar_map::GetComponentInfo());
    CheckComponent(map_updater::GetComponentInfo());
    CheckComponent(map_server::GetComponentInfo());
    CheckComponent(planner::GetComponentInfo());
    // If later activation fails, the planner member destructor joins its worker.
    planner_.Activate();
    store_.Activate();
    odometry_.Activate();
    lidar_map_.Activate();
    grid_.Activate();
    active_ = true;
    ROBO_LOG_INFO("integration") << "modules active: odom + lidar-map -> map-updater -> planner/map-server; SDK "
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
    } else if (updating_) {
        resources_.live_dropped += pending_.size();
    }
    planner_.Deactivate();
    pending_.clear();
    latest_scan_.reset();
    updating_ = false;
    grid_.Deactivate();
    odometry_.Deactivate();
    lidar_map_.Deactivate();
    transforms_ = {}; has_correction_ = false;
    pose_buffer_.Clear();
    store_.Deactivate();
    active_ = false;
}
} // namespace roboframe
