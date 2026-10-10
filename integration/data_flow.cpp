#include "integration/robot_integration.hpp"
#include <stdexcept>

namespace roboframe {
void RobotIntegration::PushOdometry(const TimedPose2D& pose) {
    RequireActive();
    odometry_.PushOdometry(pose);
}
void RobotIntegration::RouteOdometry(const TimedPose2D& pose) {
    pose_buffer_.PushOdometry(pose);
    if (status_.phase == MappingPhase::mapping) Drain(false);
}
void RobotIntegration::PushScan(map_updater::ScanPtr scan) {
    RequireActive();
    if (!scan) throw std::invalid_argument("null scan");
    map_updater::ValidateScan(*scan);
    if (scan->stamp_us <= last_scan_us_) throw std::invalid_argument("scan timestamps must strictly increase");
    last_scan_us_ = scan->stamp_us;
    if (status_.phase != MappingPhase::mapping) { ++status_.ignored; return; }
    ++status_.received;
    if (pending_.size() == pending_limit) {
        pending_.pop_front();
        ++status_.dropped;
        ++status_.drop_queue;
    }
    pending_.push_back(std::move(scan));
    Drain(false);
}
void RobotIntegration::Drain(bool finishing) {
    while (!pending_.empty()) {
        TimedPose2D pose;
        const auto match = pose_buffer_.At(pending_.front()->stamp_us, pose);
        if (match == roboframe::PoseMatch::waiting && !finishing) break;
        if (match == roboframe::PoseMatch::ready) {
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
