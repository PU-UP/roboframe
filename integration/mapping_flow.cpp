#include "integration/robot_integration_impl.hpp"
#include <roboframe/log.hpp>
namespace roboframe {
void RobotIntegration::Impl::StartMapping(std::uint64_t id) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing) throw std::logic_error("mapping already running");
    if (!id || id <= status_.session) throw std::invalid_argument("session id must increase");
    const auto old = inputs_.Close(); // Admission closes before changing execution generation.
    resources_.live_dropped += old.pending + old.submitted - completed_;
    computation_.Reset(++generation_);
    inputs_.Begin(generation_); completed_ = 0;
    computed_.reset(); copied_bytes_ = 0; frozen_.reset(); live_.reset(); latest_scan_.reset();
    last_publication_us_ = -1; updating_ = true;
    status_ = {}; status_.session = id; status_.phase = MappingPhase::mapping;
    if (outputs_.transforms) outputs_.transforms({});
    EmitMapping(MappingEventKind::started);
}
void RobotIntegration::Impl::FinishMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping) throw std::logic_error("no mapping session");
    const auto input = inputs_.Finish();
    if (input.submitted == completed_ && !computed_) {
        inputs_.ResumeMapping(); throw std::logic_error("cannot finish an empty map");
    }
    status_.phase = MappingPhase::finishing; EmitMapping(MappingEventKind::finishing);
}
void RobotIntegration::Impl::CompleteFinish() {
    if (!computed_) {
        inputs_.Close(); status_.phase = MappingPhase::idle; updating_ = false;
        EmitMapping(MappingEventKind::failed); return;
    }
    frozen_ = computed_; Publish(latest_scan_, true);
    status_.phase = MappingPhase::ready; inputs_.Live();
    EmitMapping(MappingEventKind::finished);
}
void RobotIntegration::Impl::CancelMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping && status_.phase != MappingPhase::finishing)
        throw std::logic_error("no mapping session");
    const auto input = inputs_.Close();
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
    resources_.live_dropped += input.pending + input.submitted - completed_;
    completed_ = input.submitted; computation_.Reset(++generation_);
    frozen_ = std::move(map); live_ = frozen_; computed_.reset(); latest_scan_.reset(); updating_ = false;
    planner_.Submit(live_); ++resources_.publications; status_.phase = MappingPhase::ready;
    if (outputs_.transforms) outputs_.transforms({});
    if (outputs_.map) outputs_.map({generation_, live_});
    EmitMapping(MappingEventKind::loaded, name);
}
} // namespace roboframe
