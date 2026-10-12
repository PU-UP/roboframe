#include "integration/robot_integration_impl.hpp"
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
void RobotIntegration::Impl::StartMapping(std::uint64_t id) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing) throw std::logic_error("mapping already running");
    if (!id || id <= status_.session) throw std::invalid_argument("session id must increase");
    mapper_.Reset(++generation_);
    computed_.reset(); copied_bytes_ = 0;
    lidar_map_.Reset();
    has_correction_ = false; transforms_ = {};
    resources_.live_dropped += pending_.size() + in_flight_;
    in_flight_ = 0;
    pending_.clear();
    frozen_.reset();
    live_.reset();
    latest_scan_.reset();
    last_publication_us_ = -1;
    updating_ = true;
    status_ = {};
    status_.session = id;
    status_.phase = MappingPhase::mapping;
    if (outputs_.transforms) outputs_.transforms({});
    EmitMapping(MappingEventKind::started);
    ROBO_LOG_INFO("mapping") << "begin session=" << id;
}
void RobotIntegration::Impl::FinishMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping) throw std::logic_error("no mapping session");
    Drain(true);
    if (!in_flight_ && !computed_) throw std::logic_error("cannot finish an empty map");
    status_.phase = MappingPhase::finishing;
    EmitMapping(MappingEventKind::finishing);
}
void RobotIntegration::Impl::CompleteFinish() {
    if (!computed_) {
        status_.phase = MappingPhase::idle; updating_ = false;
        EmitMapping(MappingEventKind::failed); return;
    }
    frozen_ = computed_;
    Publish(latest_scan_, true);
    status_.phase = MappingPhase::ready;
    EmitMapping(MappingEventKind::finished);
    ROBO_LOG_INFO("mapping") << "finished integrated=" << status_.integrated << " dropped=" << status_.dropped;
}
void RobotIntegration::Impl::CancelMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping && status_.phase != MappingPhase::finishing)
        throw std::logic_error("no mapping session");
    status_.dropped += pending_.size() + in_flight_;
    status_.drop_cancel += pending_.size() + in_flight_;
    mapper_.Reset(0); in_flight_ = 0; computed_.reset();
    pending_.clear();
    status_.phase = MappingPhase::idle;
    updating_ = false;
    lidar_map_.Reset(); has_correction_ = false; transforms_ = {};
    latest_scan_.reset();
    frozen_.reset(); live_.reset();
    if (outputs_.transforms) outputs_.transforms({});
    EmitMapping(MappingEventKind::canceled);
    ROBO_LOG_INFO("mapping") << "canceled session=" << status_.session;
}
void RobotIntegration::Impl::SaveMap(const std::string& name) {
    RequireActive();
    if (status_.phase != MappingPhase::ready) throw std::logic_error("finish or load before save");
    store_.Save(name, frozen_);
    EmitMapping(MappingEventKind::saved, name);
    ROBO_LOG_INFO("mapping") << "saved " << name << " revision=" << frozen_->revision;
}
void RobotIntegration::Impl::SnapshotMap(const std::string& name) {
    RequireActive();
    if (!updating_ || !computed_)
        throw std::logic_error("snapshot requires a nonempty updating map");
    store_.Save(name, computed_); // Most recent completed snapshot, never queued input.
}
void RobotIntegration::Impl::LoadMap(const std::string& name) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing) throw std::logic_error("cannot load during mapping");
    auto map = store_.Load(name); // Failure preserves the existing map and phase.
    if (map->spec.frame != frame_) throw std::invalid_argument("loaded map frame differs from product frame");
    lidar_map_.Reset(); has_correction_ = false; transforms_ = {};
    frozen_ = std::move(map);
    live_ = frozen_;
    resources_.live_dropped += pending_.size() + in_flight_;
    in_flight_ = 0;
    pending_.clear();
    latest_scan_.reset();
    updating_ = false; // Loading occupancy alone cannot reconstruct the mapper's evidence state.
    planner_.Submit(live_);
    ++resources_.publications;
    status_.phase = MappingPhase::ready;
    mapper_.Reset(++generation_);
    computed_.reset();
    if (outputs_.transforms) outputs_.transforms({});
    if (outputs_.map) outputs_.map({generation_, live_});
    EmitMapping(MappingEventKind::loaded, name);
}
} // namespace roboframe
