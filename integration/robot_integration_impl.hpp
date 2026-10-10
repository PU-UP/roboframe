#pragma once
#include "integration/robot_integration.hpp"
#include "integration/pose_buffer.hpp"
#include <map_server/file_map_component.hpp>
#include <map_updater/occupancy_map_component.hpp>
#include <odom/passthrough_odom_component.hpp>
#include <deque>

namespace roboframe {
// Private assembly shared only by the integration implementation files.
struct RobotIntegration::Impl {
    Impl(MappingOptions options, std::string directory);
    ~Impl();
    void StartMapping(std::uint64_t id);
    void FinishMapping();
    void CancelMapping();
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name); // Inspection checkpoint; does not finish mapping.
    void LoadMap(const std::string& name);
    void PushOdometry(const TimedPose2D& pose);
    void PushScan(roboframe::Scan2DPtr scan);
    void Shutdown();
    void RouteOdometry(const TimedPose2D& pose);
    void Drain(bool finishing);
    void RequireActive() const;
    PoseBuffer pose_buffer_;
    odom::PassthroughOdomComponent odometry_;
    map_updater::OccupancyMapComponent grid_;
    map_server::FileMapComponent store_;
    MappingStatus status_;
    GridSnapshot frozen_;
    std::deque<roboframe::Scan2DPtr> pending_;
    std::int64_t last_scan_us_ = -1;
    bool active_ = false;
    static constexpr std::size_t pending_limit = 64;
};
} // namespace roboframe
