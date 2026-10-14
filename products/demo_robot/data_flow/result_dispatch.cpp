#include "products/demo_robot/demo_robot_integration_impl.hpp"
namespace demo_robot {
using namespace roboframe;
std::size_t DemoRobotIntegration::Impl::PendingCompute() const { return inputs_.status().submitted-completed_; }
void DemoRobotIntegration::Impl::PublishInputs() {
    const auto events=inputs_.TakeEvents();
    if (events.has_odom && outputs_.odometry) outputs_.odometry(events.odom);
    if (events.has_correction && outputs_.correction) outputs_.correction(events.correction);
    if (events.has_tf && outputs_.transforms) outputs_.transforms(events.tf);
}
void DemoRobotIntegration::Impl::OnMapResult(MapExecution::Result result) {
    ++completed_;
    if (result.input.task && !task_.accepts(result.input.task)) ++resources_.detached_task_results;
    RecordTaskResult(result.input,result.output);
    ++resources_.local_updates;
    copied_bytes_=result.output.copied_bytes;
    resources_.submaps=result.output.submaps; resources_.retained_points=result.output.points;
    if (result.output.integrated) {
        computed_=result.output.map; latest_scan_=result.input.scan;
        if (!task_.accepts(result.input.task)) ++resources_.live_updates;
        Publish(latest_scan_);
    } else if (!task_.accepts(result.input.task)) ++resources_.live_dropped;
    AdvanceTasks(); // Complete at the fence before processing unrelated later results.
}
void DemoRobotIntegration::Impl::Publish(const Scan2DPtr& scan,bool force) {
    if (!scan || !computed_ || live_==computed_) return;
    if (!force && last_publication_us_>=0 && scan->stamp_us-last_publication_us_<100000) return;
    live_=computed_; last_publication_us_=scan->stamp_us;
    ++resources_.shared_scans; ++resources_.publications;
    if (outputs_.map) outputs_.map({generation_,live_});
    if (task_.status.phase==MappingPhase::mapping) EmitMapping(MappingEventKind::progress);
}
DataFlowStatus DemoRobotIntegration::Impl::Resources() const {
    auto result=resources_; const auto state=inputs_.status(); const auto& input=state.resources;
    result.live_dropped+=input.live_dropped;
    result.pose_nodes=input.pose_nodes; result.history_rejected=input.history_rejected;
    result.slam_scans=input.slam_scans; result.correction_outputs=input.correction_outputs;
    result.missed_references=input.missed_references; result.tf_dropped=input.tf_dropped;
    result.compute_rejected=input.compute_rejected;
    const auto work=computation_.status();
    result.compute_pending=work.queued; result.compute_active=work.active;
    result.compute_results=work.completed; result.compute_retained=work.retained;
    result.compute_peak_retained=work.peak; result.copied_bytes=copied_bytes_;
    if (live_) { result.map_bytes=live_->cells.size(); result.table_bytes=live_->cells.table_bytes(); }
    return result;
}
} // namespace demo_robot
