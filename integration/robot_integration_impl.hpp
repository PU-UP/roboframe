#pragma once
#include "integration/robot_integration.hpp"
#include "integration/pose_buffer.hpp"
#include "integration/map_update_worker.hpp"
#include <map_server/file_map_component.hpp>
#include <map_updater/occupancy_map_component.hpp>
#include <odom/passthrough_odom_component.hpp>
#include <lidar_map/recorded_lidar_map_component.hpp>
#include <planner/read_only_planner_component.hpp>
#include <deque>

namespace roboframe {
// Private assembly shared only by the integration implementation files.
struct RobotIntegration::Impl {
    Impl(MappingOptions options, std::string directory, ProductOutputs& outputs, std::chrono::milliseconds delay);
    ~Impl();
    void StartMapping(std::uint64_t id);
    void FinishMapping();
    void CancelMapping();
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name); // Inspection checkpoint; does not finish mapping.
    void LoadMap(const std::string& name);
    void PushOdometry(const TimedPose2D& pose);
    void PushMapReference(const MapPose2D& pose);
    void PushScan(roboframe::Scan2DPtr scan);
    void Shutdown();
    void RouteOdometry(const TimedPose2D& pose);
    void Drain(bool finishing);
    void RequireActive() const;
    void Publish(const Scan2DPtr& scan, bool force = false);
    void WaitPlanner();
    void Poll();
    void CompleteFinish();
    DataFlowStatus Resources() const;
    void EmitMapping(MappingEventKind kind, const std::string& name = {});
    ProductOutputs& outputs_;
    std::uint64_t generation_ = 0;
    PoseBuffer pose_buffer_;
    odom::PassthroughOdomComponent odometry_;
    lidar_map::RecordedLidarMapComponent lidar_map_;
    MapUpdateWorker mapper_;
    std::size_t in_flight_ = 0;
    std::uint64_t copied_bytes_ = 0;
    GridSnapshot computed_;
    map_server::FileMapComponent store_;
    planner::ReadOnlyPlannerComponent planner_{std::chrono::milliseconds(5)};
    const GridFrame frame_;
    const std::int64_t max_correction_age_us_;
    MapCorrection2D correction_;
    bool has_correction_ = false;
    TransformStatus2D transforms_;
    MappingStatus status_;
    GridSnapshot frozen_, live_;
    Scan2DPtr latest_scan_;
    DataFlowStatus resources_;
    bool updating_ = false;
    std::int64_t last_publication_us_ = -1;
    std::deque<roboframe::Scan2DPtr> pending_;
    std::int64_t last_scan_us_ = -1;
    bool active_ = false;
    static constexpr std::size_t pending_limit = 64;
};
} // namespace roboframe
