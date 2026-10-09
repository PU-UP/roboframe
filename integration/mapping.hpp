#pragma once
#include <map_store/component.hpp>
#include <occupancy_grid/component.hpp>
#include <odom_buffer/component.hpp>
#include <deque>
#include <string>

namespace roboframe {
class MappingIntegration {
public:
    enum class Phase { idle, mapping, ready };
    struct Status {
        Phase phase = Phase::idle;
        std::uint64_t session = 0, received = 0, integrated = 0, ignored = 0;
        std::uint64_t dropped = 0, outside_rays = 0, used_rays = 0;
        std::uint64_t drop_sync = 0, drop_queue = 0, drop_bounds = 0, drop_cancel = 0;
    };
    MappingIntegration(GridSpec spec, std::string directory, odom_buffer::Config odometry = {});
    ~MappingIntegration();
    MappingIntegration(const MappingIntegration&) = delete;
    MappingIntegration& operator=(const MappingIntegration&) = delete;
    void Begin(std::uint64_t id);
    void Finish();
    void Cancel();
    void Save(const std::string& name);
    void Snapshot(const std::string& name); // Inspection checkpoint; does not finish mapping.
    void Load(const std::string& name);
    void PushOdometry(const TimedPose2D& pose);
    void PushScan(occupancy_grid::ScanPtr scan);
    void Shutdown();
    const Status& status() const { return status_; }
    std::size_t pending() const { return pending_.size(); }
    GridSnapshot map() const { return frozen_; }
private:
    void Drain(bool finishing);
    void RequireActive() const;
    odom_buffer::Component odometry_;
    occupancy_grid::Component grid_;
    map_store::Component store_;
    Status status_;
    GridSnapshot frozen_;
    std::deque<occupancy_grid::ScanPtr> pending_;
    std::int64_t last_scan_us_ = -1;
    bool active_ = false;
    static constexpr std::size_t pending_limit = 64;
};
} // namespace roboframe
