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
    AdvanceWorkflows();
}
void RobotIntegration::Impl::Publish(const Scan2DPtr& scan, bool force) {
    if (!scan || !computed_ || (live_ && live_->revision == computed_->revision)) return;
    if (!force && last_publication_us_ >= 0 && scan->stamp_us - last_publication_us_ < 100000) return;
    live_ = computed_;
    planner_.Submit(live_, scan);
    last_publication_us_ = scan->stamp_us;
    ++resources_.shared_scans; ++resources_.publications;
    if (outputs_.map) outputs_.map({generation_, live_});
    OnMapPublished();
}
void RobotIntegration::Impl::WaitPlanner() {
    RequireActive();
    if (PendingCompute()) throw std::logic_error("map computation pending");
    if (updating_) Publish(latest_scan_, true);
    const auto state = planner_.status();
    if (state.active_read || state.pending) throw std::logic_error("planner read pending");
}
DataFlowStatus RobotIntegration::Impl::Resources() const {
    auto result = resources_;
    const auto state = inputs_.status();
    auto task = status_;
    CountInputs(state.counts, task, result);
    const auto& input = state.resources;
    result.lidar_scans = input.lidar_scans; result.correction_outputs = input.correction_outputs;
    result.missed_references = input.missed_references; result.tf_dropped = input.tf_dropped;
    result.compute_rejected = input.compute_rejected;
    const auto work = computation_.status();
    result.compute_pending = work.queued; result.compute_active = work.active;
    result.compute_results = work.completed; result.compute_retained = work.retained;
    result.compute_peak_retained = work.peak; result.copied_bytes = copied_bytes_;
    if (live_) { result.map_bytes = live_->cells.size(); result.table_bytes = live_->cells.table_bytes(); }
    const auto reader = planner_.status();
    result.planner_received = reader.received; result.planner_completed = reader.completed;
    result.planner_replaced = reader.replaced; result.planner_pending = reader.pending;
    result.planner_active = reader.active_read; result.planner_peak_retained = reader.peak_retained;
    result.planner_inconsistent = reader.inconsistent; result.planner_revision = reader.revision;
    return result;
}
} // namespace roboframe
