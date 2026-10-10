#pragma once
#include <map_server/file_map_component.hpp>
#include <map_updater/occupancy_map_component.hpp>
#include <odom/passthrough_odom_component.hpp>
#include "integration/pose_buffer.hpp"
#include <deque>
#include <string>

namespace roboframe {
class RobotIntegration {
public:
    enum class MappingPhase { idle, mapping, ready };
    struct MappingStatus {
        MappingPhase phase = MappingPhase::idle;
        std::uint64_t session = 0, received = 0, integrated = 0, ignored = 0;
        std::uint64_t dropped = 0, outside_rays = 0, used_rays = 0;
        std::uint64_t drop_sync = 0, drop_queue = 0, drop_bounds = 0, drop_cancel = 0;
    };
    RobotIntegration(GridSpec spec, std::string directory, PoseBufferConfig synchronization = {});
    ~RobotIntegration();
    RobotIntegration(const RobotIntegration&) = delete;
    RobotIntegration& operator=(const RobotIntegration&) = delete;
    void StartMapping(std::uint64_t id);
    void FinishMapping();
    void CancelMapping();
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name); // Inspection checkpoint; does not finish mapping.
    void LoadMap(const std::string& name);
    void PushOdometry(const TimedPose2D& pose);
    void PushScan(map_updater::ScanPtr scan);
    void Shutdown();
    const MappingStatus& mapping_status() const { return status_; }
    std::size_t pending() const { return pending_.size(); }
    GridSnapshot map() const { return frozen_; }
private:
    void RouteOdometry(const TimedPose2D& pose);
    void Drain(bool finishing);
    void RequireActive() const;
    PoseBuffer pose_buffer_;
    odom::PassthroughOdomComponent odometry_;
    map_updater::OccupancyMapComponent grid_;
    map_server::FileMapComponent store_;
    MappingStatus status_;
    GridSnapshot frozen_;
    std::deque<map_updater::ScanPtr> pending_;
    std::int64_t last_scan_us_ = -1;
    bool active_ = false;
    static constexpr std::size_t pending_limit = 64;
};
} // namespace roboframe
