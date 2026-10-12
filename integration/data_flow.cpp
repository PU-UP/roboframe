#include "integration/robot_integration_impl.hpp"
#include <stdexcept>
#include <roboframe/log.hpp>

namespace roboframe {
void RobotIntegration::Impl::PushOdometry(const TimedPose2D& pose) {
    RequireActive();
    odometry_.PushOdometry(pose);
}
void RobotIntegration::Impl::RouteOdometry(const TimedPose2D& pose) {
    pose_buffer_.PushOdometry(pose);
    if (outputs_.odometry) outputs_.odometry(pose);
    if (updating_) Drain(false);
}
void RobotIntegration::Impl::PushMapReference(const MapPose2D& pose) {
    RequireActive();
    if (frame_ != GridFrame::map || !updating_)
        throw std::logic_error("reference requires active map-frame mapping");
    if (status_.phase == MappingPhase::finishing) return;
    lidar_map_.PushReferencePose(pose);
}
void RobotIntegration::Impl::PushScan(roboframe::Scan2DPtr scan) {
    RequireActive();
    if (!scan) throw std::invalid_argument("null scan");
    map_updater::ValidateScan(*scan);
    if (scan->stamp_us <= last_scan_us_) throw std::invalid_argument("scan timestamps must strictly increase");
    last_scan_us_ = scan->stamp_us;
    if (!updating_ || status_.phase == MappingPhase::finishing) { ++status_.ignored; return; }
    if (status_.phase == MappingPhase::mapping) ++status_.received;
    if (pending_.size() == pending_limit) {
        pending_.pop_front();
        if (status_.phase == MappingPhase::mapping) {
            ++status_.dropped;
            ++status_.drop_queue;
        } else { ++resources_.live_dropped; }
    }
    pending_.push_back(std::move(scan));
    Drain(false);
}
void RobotIntegration::Impl::Drain(bool finishing) {
    while (!pending_.empty()) {
        TimedPose2D pose;
        const auto match = pose_buffer_.At(pending_.front()->stamp_us, pose);
        if (match == roboframe::PoseMatch::waiting && !finishing) break;
        if (match == roboframe::PoseMatch::ready) {
            try {
                lidar_map_.PushScan(pending_.front(), pose);
                transforms_ = {};
                if (frame_ == GridFrame::map && (!has_correction_ || correction_.stamp_us > pose.stamp_us ||
                        pose.stamp_us - correction_.stamp_us > max_correction_age_us_)) {
                    if (outputs_.transforms) outputs_.transforms(transforms_);
                    ++resources_.tf_dropped;
                    if (status_.phase == MappingPhase::mapping) { ++status_.dropped; ++status_.drop_sync; }
                    else { ++resources_.live_dropped; }
                    pending_.pop_front();
                    continue;
                }
                transforms_ = {true, {pose, frame_ == GridFrame::map ? correction_ : MapCorrection2D{pose.stamp_us, {}}}};
                if (outputs_.transforms) outputs_.transforms(transforms_);
                if (mapper_.Submit(generation_, pending_.front(), transforms_.transforms)) {
                    ++in_flight_;
                } else {
                    ++resources_.compute_rejected;
                    if (status_.phase == MappingPhase::mapping) { ++status_.dropped; ++status_.drop_queue; }
                    else ++resources_.live_dropped;
                }
            } catch (const std::out_of_range&) {
                if (status_.phase == MappingPhase::mapping) {
                    ++status_.drop_bounds;
                    ++status_.dropped;
                } else { ++resources_.live_dropped; } // Never silently resize.
            }
        } else {
            if (status_.phase == MappingPhase::mapping) {
                ++status_.drop_sync;
                ++status_.dropped;
            } else { ++resources_.live_dropped; } // Missing bracket or excessive timestamp gap.
        }
        pending_.pop_front();
    }
}
void RobotIntegration::Impl::Poll() {
    MapUpdateWorker::Result result;
    // No worker calls product callbacks. All state changes and output happen here on the owner.
    for (std::size_t i = 0; i < MapUpdateWorker::capacity && mapper_.Take(result); ++i) {
        if (result.generation != generation_ || !updating_) { ++resources_.stale_results; continue; }
        --in_flight_;
        const bool mapping = status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing;
        if (result.outcome == MapUpdateWorker::Result::Outcome::success) {
            computed_ = std::move(result.map); copied_bytes_ = result.copied_bytes;
            latest_scan_ = std::move(result.scan);
            if (mapping) {
                ++status_.integrated; status_.used_rays += result.update.used_rays;
                status_.outside_rays += result.update.outside_rays;
            } else ++resources_.live_updates;
            Publish(latest_scan_);
        } else if (result.outcome == MapUpdateWorker::Result::Outcome::outside) {
            if (mapping) { ++status_.dropped; ++status_.drop_bounds; }
            else ++resources_.live_dropped;
        } else {
            ROBO_LOG_ERROR("mapping") << "compute failed: " << result.error;
            if (mapping) {
                status_.dropped += 1 + in_flight_ + pending_.size();
                status_.drop_cancel += 1 + in_flight_ + pending_.size();
            } else resources_.live_dropped += 1 + in_flight_ + pending_.size();
            mapper_.Reset(0); in_flight_ = 0; pending_.clear(); updating_ = false;
            status_.phase = MappingPhase::idle; EmitMapping(MappingEventKind::failed);
        }
    }
    if (status_.phase == MappingPhase::finishing && !in_flight_) CompleteFinish();
}
void RobotIntegration::Impl::Publish(const Scan2DPtr& scan, bool force) {
    if (!scan || !computed_ || (live_ && live_->revision == computed_->revision)) return;
    if (!force && last_publication_us_ >= 0 && scan->stamp_us - last_publication_us_ < 100000) return;
    live_ = computed_;
    planner_.Submit(live_, scan);
    last_publication_us_ = scan->stamp_us;
    ++resources_.shared_scans; ++resources_.publications;
    if (outputs_.map) outputs_.map({generation_, live_});
    if (status_.phase == MappingPhase::mapping) EmitMapping(MappingEventKind::progress);
}
void RobotIntegration::Impl::WaitPlanner() {
    RequireActive();
    if (in_flight_) throw std::logic_error("map computation pending");
    if (updating_) Publish(latest_scan_, true);
    const auto state = planner_.status();
    if (state.active_read || state.pending) throw std::logic_error("planner read pending");
}
DataFlowStatus RobotIntegration::Impl::Resources() const {
    auto result = resources_;
    result.lidar_scans = lidar_map_.received();
    result.correction_outputs = lidar_map_.published();
    result.missed_references = lidar_map_.missed_references();
    const auto work = mapper_.status();
    result.compute_pending = work.queued; result.compute_active = work.active;
    result.compute_results = work.completed; result.compute_retained = work.retained;
    result.compute_peak_retained = work.peak;
    result.copied_bytes = copied_bytes_;
    if (live_) {
        result.map_bytes = live_->cells.size();
        result.table_bytes = live_->cells.table_bytes();
    }
    const auto reader = planner_.status();
    result.planner_received = reader.received;
    result.planner_completed = reader.completed;
    result.planner_replaced = reader.replaced;
    result.planner_pending = reader.pending;
    result.planner_active = reader.active_read;
    result.planner_peak_retained = reader.peak_retained;
    result.planner_inconsistent = reader.inconsistent;
    result.planner_revision = reader.revision;
    return result;
}
} // namespace roboframe
