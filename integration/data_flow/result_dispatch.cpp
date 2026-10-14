#include "integration/robot_integration_impl.hpp"
namespace roboframe {
std::size_t RobotIntegration::Impl::PendingCompute() const { return inputs_.status().submitted - completed_; }
void RobotIntegration::Impl::Poll() {
    const auto events = inputs_.TakeEvents();
    if (events.has_odom && outputs_.odometry) outputs_.odometry(events.odom);
    if (events.has_correction && outputs_.correction) outputs_.correction(events.correction);
    if (events.has_tf && outputs_.transforms) outputs_.transforms(events.tf);
    MapExecution::Result result;
    for (std::size_t i = 0; i < compute_capacity && computation_.Take(result); ++i)
        OnMapResult(std::move(result));
    if (updating_ && !PendingCompute()) Publish(latest_scan_, true);
    AdvanceWorkflows();
}
void RobotIntegration::Impl::Publish(const Scan2DPtr& scan, bool force) {
    if (!scan || !computed_ || (live_ && live_->revision == computed_->revision)) return;
    if (!force && last_publication_us_ >= 0 && scan->stamp_us - last_publication_us_ < 100000) return;
    live_ = computed_;
    last_publication_us_ = scan->stamp_us;
    ++resources_.shared_scans; ++resources_.publications;
    if (outputs_.map) outputs_.map({generation_, live_});
    OnMapPublished();
}
DataFlowStatus RobotIntegration::Impl::Resources() const {
    auto result = resources_;
    const auto state = inputs_.status();
    auto task = status_;
    CountInputs(state.counts, task, result);
    const auto& input = state.resources;
    result.pose_nodes = input.pose_nodes; result.history_rejected = input.history_rejected;
    result.slam_scans = input.slam_scans; result.correction_outputs = input.correction_outputs;
    result.missed_references = input.missed_references; result.tf_dropped = input.tf_dropped;
    result.compute_rejected = input.compute_rejected;
    const auto work = computation_.status();
    result.compute_pending = work.queued; result.compute_active = work.active;
    result.compute_results = work.completed; result.compute_retained = work.retained;
    result.compute_peak_retained = work.peak; result.copied_bytes = copied_bytes_;
    if (live_) { result.map_bytes = live_->cells.size(); result.table_bytes = live_->cells.table_bytes(); }
    return result;
}
} // namespace roboframe
