#include "integration/robot_integration_impl.hpp"
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
void RobotIntegration::Impl::StartMapping(std::uint64_t id) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping) throw std::logic_error("mapping already running");
    if (!id || id <= status_.session) throw std::invalid_argument("session id must increase");
    grid_.Reset();
    lidar_map_.Reset();
    has_correction_ = false; transforms_ = {};
    resources_.live_dropped += pending_.size();
    pending_.clear();
    frozen_.reset();
    live_.reset();
    latest_scan_.reset();
    last_publication_us_ = -1;
    updating_ = true;
    status_ = {};
    status_.session = id;
    status_.phase = MappingPhase::mapping;
    ROBO_LOG_INFO("mapping") << "begin session=" << id;
}
void RobotIntegration::Impl::FinishMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping) throw std::logic_error("no mapping session");
    Drain(true);
    if (!grid_.revision()) throw std::logic_error("cannot finish an empty map");
    auto snapshot = grid_.Snapshot();
    frozen_ = std::move(snapshot);
    Publish(latest_scan_, true);
    status_.phase = MappingPhase::ready;
    ROBO_LOG_INFO("mapping") << "finished integrated=" << status_.integrated
        << " dropped=" << status_.dropped << "; baseline frozen, live updates continue";
}
void RobotIntegration::Impl::CancelMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping) throw std::logic_error("no mapping session");
    status_.dropped += pending_.size();
    status_.drop_cancel += pending_.size();
    pending_.clear();
    status_.phase = MappingPhase::idle;
    updating_ = false;
    lidar_map_.Reset(); has_correction_ = false; transforms_ = {};
    latest_scan_.reset();
    frozen_.reset();
    ROBO_LOG_INFO("mapping") << "canceled session=" << status_.session;
}
void RobotIntegration::Impl::SaveMap(const std::string& name) {
    RequireActive();
    if (status_.phase != MappingPhase::ready) throw std::logic_error("finish or load before save");
    store_.Save(name, frozen_);
    ROBO_LOG_INFO("mapping") << "saved " << name << " revision=" << frozen_->revision;
}
void RobotIntegration::Impl::SnapshotMap(const std::string& name) {
    RequireActive();
    if (!updating_ || !grid_.revision())
        throw std::logic_error("snapshot requires a nonempty updating map");
    store_.Save(name, grid_.Snapshot());
}
void RobotIntegration::Impl::LoadMap(const std::string& name) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping) throw std::logic_error("cannot load during mapping");
    auto map = store_.Load(name); // Failure preserves the existing map and phase.
    if (map->spec.frame != frame_) throw std::invalid_argument("loaded map frame differs from product frame");
    lidar_map_.Reset(); has_correction_ = false; transforms_ = {};
    frozen_ = std::move(map);
    live_ = frozen_;
    resources_.live_dropped += pending_.size();
    pending_.clear();
    latest_scan_.reset();
    updating_ = false; // Loading occupancy alone cannot reconstruct the mapper's evidence state.
    planner_.Submit(live_);
    ++resources_.publications;
    status_.phase = MappingPhase::ready;
}
} // namespace roboframe
