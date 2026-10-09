#include "integration/mapping.hpp"
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
MappingIntegration::MappingIntegration(GridSpec spec, std::string directory, odom_buffer::Config config)
    : odometry_(config), grid_(spec), store_(std::move(directory)) {
    CheckCompatibility(odom_buffer::GetComponentInfo(), odom_buffer::contract_version);
    CheckCompatibility(occupancy_grid::GetComponentInfo(), occupancy_grid::contract_version);
    CheckCompatibility(map_store::GetComponentInfo(), map_store::contract_version);
    // The only potentially failing activation comes first; later activations allocate no resources.
    store_.Activate();
    odometry_.Activate();
    grid_.Activate();
    active_ = true;
    ROBO_LOG_INFO("mapping") << "modules active: odom-buffer -> occupancy-grid -> map-store; SDK "
                             << ToString(sdk_version);
}
MappingIntegration::~MappingIntegration() { Shutdown(); }
void MappingIntegration::RequireActive() const {
    if (!active_) throw std::logic_error("integration stopped");
}
void MappingIntegration::Begin(std::uint64_t id) {
    RequireActive();
    if (status_.phase == Phase::mapping) throw std::logic_error("mapping already running");
    if (!id || id <= status_.session) throw std::invalid_argument("session id must increase");
    grid_.Reset();
    pending_.clear();
    frozen_.reset();
    status_ = {};
    status_.session = id;
    status_.phase = Phase::mapping;
    ROBO_LOG_INFO("mapping") << "begin session=" << id;
}
void MappingIntegration::Finish() {
    RequireActive();
    if (status_.phase != Phase::mapping) throw std::logic_error("no mapping session");
    Drain(true);
    if (!grid_.revision()) throw std::logic_error("cannot finish an empty map");
    auto snapshot = grid_.Snapshot();
    frozen_ = std::move(snapshot);
    status_.phase = Phase::ready;
    ROBO_LOG_INFO("mapping") << "finished integrated=" << status_.integrated
        << " dropped=" << status_.dropped << "; map frozen";
}
void MappingIntegration::Cancel() {
    RequireActive();
    if (status_.phase != Phase::mapping) throw std::logic_error("no mapping session");
    status_.dropped += pending_.size();
    status_.drop_cancel += pending_.size();
    pending_.clear();
    status_.phase = Phase::idle;
    frozen_.reset();
    ROBO_LOG_INFO("mapping") << "canceled session=" << status_.session;
}
void MappingIntegration::Save(const std::string& name) {
    RequireActive();
    if (status_.phase != Phase::ready) throw std::logic_error("finish or load before save");
    store_.Save(name, frozen_);
    ROBO_LOG_INFO("mapping") << "saved " << name << " revision=" << frozen_->revision;
}
void MappingIntegration::Snapshot(const std::string& name) {
    RequireActive();
    if (status_.phase != Phase::mapping || !grid_.revision())
        throw std::logic_error("snapshot requires a nonempty running map");
    store_.Save(name, grid_.Snapshot());
}
void MappingIntegration::Load(const std::string& name) {
    RequireActive();
    if (status_.phase == Phase::mapping) throw std::logic_error("cannot load during mapping");
    auto map = store_.Load(name); // Failure preserves the existing map and phase.
    frozen_ = std::move(map);
    status_.phase = Phase::ready;
}
void MappingIntegration::Shutdown() {
    if (!active_) return;
    // EOF/exit abandons pending work; it never silently finishes or saves a map.
    if (status_.phase == Phase::mapping) {
        status_.dropped += pending_.size();
        status_.drop_cancel += pending_.size();
        status_.phase = Phase::idle;
    }
    pending_.clear();
    grid_.Deactivate();
    odometry_.Deactivate();
    store_.Deactivate();
    active_ = false;
}
} // namespace roboframe
