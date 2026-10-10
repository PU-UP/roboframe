#include "integration/robot_integration_impl.hpp"
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
void RobotIntegration::Impl::StartMapping(std::uint64_t id) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping) throw std::logic_error("mapping already running");
    if (!id || id <= status_.session) throw std::invalid_argument("session id must increase");
    grid_.Reset();
    pending_.clear();
    frozen_.reset();
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
    status_.phase = MappingPhase::ready;
    ROBO_LOG_INFO("mapping") << "finished integrated=" << status_.integrated
        << " dropped=" << status_.dropped << "; map frozen";
}
void RobotIntegration::Impl::CancelMapping() {
    RequireActive();
    if (status_.phase != MappingPhase::mapping) throw std::logic_error("no mapping session");
    status_.dropped += pending_.size();
    status_.drop_cancel += pending_.size();
    pending_.clear();
    status_.phase = MappingPhase::idle;
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
    if (status_.phase != MappingPhase::mapping || !grid_.revision())
        throw std::logic_error("snapshot requires a nonempty running map");
    store_.Save(name, grid_.Snapshot());
}
void RobotIntegration::Impl::LoadMap(const std::string& name) {
    RequireActive();
    if (status_.phase == MappingPhase::mapping) throw std::logic_error("cannot load during mapping");
    auto map = store_.Load(name); // Failure preserves the existing map and phase.
    frozen_ = std::move(map);
    status_.phase = MappingPhase::ready;
}
} // namespace roboframe
