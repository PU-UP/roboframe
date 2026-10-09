#include "integration/mapping.hpp"
#include <stdexcept>

namespace roboframe {
void MappingIntegration::PushOdometry(const TimedPose2D& pose) {
    RequireActive();
    odometry_.PushOdometry(pose);
    if (status_.phase == Phase::mapping) Drain(false);
}
void MappingIntegration::PushScan(occupancy_grid::ScanPtr scan) {
    RequireActive();
    if (!scan) throw std::invalid_argument("null scan");
    occupancy_grid::ValidateScan(*scan);
    if (scan->stamp_us <= last_scan_us_) throw std::invalid_argument("scan timestamps must strictly increase");
    last_scan_us_ = scan->stamp_us;
    if (status_.phase != Phase::mapping) { ++status_.ignored; return; }
    ++status_.received;
    if (pending_.size() == pending_limit) {
        pending_.pop_front();
        ++status_.dropped;
        ++status_.drop_queue;
    }
    pending_.push_back(std::move(scan));
    Drain(false);
}
void MappingIntegration::Drain(bool finishing) {
    while (!pending_.empty()) {
        TimedPose2D pose;
        const auto match = odometry_.At(pending_.front()->stamp_us, pose);
        if (match == odom_buffer::Match::waiting && !finishing) break;
        if (match == odom_buffer::Match::ready) {
            try {
                const auto update = grid_.PushScan(pending_.front(), pose);
                ++status_.integrated;
                status_.used_rays += update.used_rays;
                status_.outside_rays += update.outside_rays;
            } catch (const std::out_of_range&) {
                ++status_.drop_bounds;
                ++status_.dropped; // Pose outside the configured map; never silently resize.
            }
        } else {
            ++status_.drop_sync;
            ++status_.dropped; // Missing bracket, expired history, or excessive timestamp gap.
        }
        pending_.pop_front();
    }
}
} // namespace roboframe
