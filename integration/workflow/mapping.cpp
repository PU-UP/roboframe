#include "integration/robot_integration_impl.hpp"
#include <roboframe/log.hpp>
namespace roboframe {
void RobotIntegration::Impl::StartMapping(std::uint64_t id) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing) throw std::logic_error("mapping already running");
    if (!id || id <= status_.session) throw std::invalid_argument("session id must increase");
    const auto old = inputs_.Close(); // Admission closes before changing execution generation.
    CaptureInputs(old);
    resources_.live_dropped += old.pending + old.submitted - completed_;
    computation_.Reset(++generation_);
    accounted_inputs_ = inputs_.Open(generation_).counts; completed_ = 0;
    computed_.reset(); copied_bytes_ = 0; frozen_.reset(); live_.reset(); latest_scan_.reset();
    last_publication_us_ = -1; updating_ = true;
    status_ = {}; status_.session = id; status_.phase = MappingPhase::mapping;
    if (outputs_.transforms) outputs_.transforms({});
    EmitMapping(MappingEventKind::started);
}
void RobotIntegration::Impl::FinishMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping) throw std::logic_error("no mapping session");
    const auto input = inputs_.Pause();
    if (input.submitted == completed_ && !computed_) {
        inputs_.Resume(); throw std::logic_error("cannot finish an empty map");
    }
    status_.phase = MappingPhase::finishing; EmitMapping(MappingEventKind::finishing);
}
void RobotIntegration::Impl::CompleteFinish() {
    CaptureInputs(inputs_.status());
    if (!computed_) {
        inputs_.Close(); status_.phase = MappingPhase::idle; updating_ = false;
        EmitMapping(MappingEventKind::failed); return;
    }
    frozen_ = computed_; Publish(latest_scan_, true);
    status_.phase = MappingPhase::ready; inputs_.Resume();
    EmitMapping(MappingEventKind::finished);
}
void RobotIntegration::Impl::CancelMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping && status_.phase != MappingPhase::finishing)
        throw std::logic_error("no mapping session");
    const auto input = inputs_.Close();
    CaptureInputs(input);
    const auto abandoned = input.pending + input.submitted - completed_;
    status_.dropped += abandoned; status_.drop_cancel += abandoned;
    computation_.Reset(0); completed_ = input.submitted; computed_.reset();
    status_.phase = MappingPhase::idle; updating_ = false;
    latest_scan_.reset(); frozen_.reset(); live_.reset();
    if (outputs_.transforms) outputs_.transforms({});
    EmitMapping(MappingEventKind::canceled);
}
void RobotIntegration::Impl::SaveMap(const std::string& name) {
    RequireActive();
    if (status_.phase != MappingPhase::ready) throw std::logic_error("finish or load before save");
    store_.Save(name, frozen_); EmitMapping(MappingEventKind::saved, name);
}
void RobotIntegration::Impl::SnapshotMap(const std::string& name) {
    RequireActive();
    if (!updating_ || !computed_) throw std::logic_error("snapshot requires a nonempty updating map");
    store_.Save(name, computed_);
}
void RobotIntegration::Impl::LoadMap(const std::string& name) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing)
        throw std::logic_error("cannot load during mapping");
    auto map = store_.Load(name); // No input lock; failure preserves current state.
    if (map->spec.frame != frame_) throw std::invalid_argument("loaded map frame differs from product frame");
    const auto input = inputs_.Close();
    CaptureInputs(input);
    resources_.live_dropped += input.pending + input.submitted - completed_;
    completed_ = input.submitted; computation_.Reset(++generation_);
    frozen_ = std::move(map); live_ = frozen_; computed_.reset(); latest_scan_.reset(); updating_ = false;
    planner_.Submit(live_); ++resources_.publications; status_.phase = MappingPhase::ready;
    if (outputs_.transforms) outputs_.transforms({});
    if (outputs_.map) outputs_.map({generation_, live_});
    EmitMapping(MappingEventKind::loaded, name);
}
// Interpret data counters here, where product business state is owned.
void RobotIntegration::Impl::CountInputs(const SensorFlow::Counts& input, MappingStatus& task, DataFlowStatus& resources) const {
    task.ignored += input.ignored - accounted_inputs_.ignored;
    const auto dropped = input.dropped - accounted_inputs_.dropped;
    if (task.phase == MappingPhase::mapping || task.phase == MappingPhase::finishing) {
        task.received += input.received - accounted_inputs_.received;
        task.dropped += dropped;
        task.drop_sync += input.drop_sync - accounted_inputs_.drop_sync;
        task.drop_queue += input.drop_queue - accounted_inputs_.drop_queue;
    } else resources.live_dropped += dropped;
}
void RobotIntegration::Impl::CaptureInputs(const SensorFlow::State& input) {
    CountInputs(input.counts, status_, resources_);
    accounted_inputs_ = input.counts;
}
MappingStatus RobotIntegration::Impl::Status() const {
    auto task = status_;
    auto resources = resources_;
    CountInputs(inputs_.status().counts, task, resources);
    return task;
}
void RobotIntegration::Impl::OnMapResult(MapExecution::Result result) {
    if (result.generation != generation_ || !updating_) { ++resources_.stale_results; return; }
    ++completed_;
    const bool mapping = status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing;
    try {
        if (result.error) std::rethrow_exception(result.error);
        if (result.canceled) throw std::runtime_error("unexpected current-session cancellation");
        computed_ = std::move(result.output.map); copied_bytes_ = result.output.copied_bytes;
        latest_scan_ = std::move(result.input.scan);
        if (mapping) {
            ++status_.integrated; status_.used_rays += result.output.update.used_rays;
            status_.outside_rays += result.output.update.outside_rays;
        } else ++resources_.live_updates;
        Publish(latest_scan_);
    } catch (const std::out_of_range&) {
        if (mapping) { ++status_.dropped; ++status_.drop_bounds; } else ++resources_.live_dropped;
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("integration") << "compute failed: " << error.what();
        const auto input = inputs_.Close();
        CaptureInputs(input);
        const auto abandoned = 1 + input.pending + input.submitted - completed_;
        if (mapping) { status_.dropped += abandoned; status_.drop_cancel += abandoned; }
        else resources_.live_dropped += abandoned;
        computation_.Reset(0); completed_ = input.submitted; updating_ = false;
        status_.phase = MappingPhase::idle; EmitMapping(MappingEventKind::failed);
    }
}
void RobotIntegration::Impl::OnMapPublished() {
    if (status_.phase == MappingPhase::mapping) EmitMapping(MappingEventKind::progress);
}
void RobotIntegration::Impl::AdvanceWorkflows() {
    if (status_.phase == MappingPhase::finishing && !PendingCompute()) CompleteFinish();
}
} // namespace roboframe
