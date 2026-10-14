#include "integration/robot_integration_impl.hpp"
#include <utility>
#include <roboframe/log.hpp>
#include <stdexcept>

namespace roboframe {
namespace {
void CheckComponent(const ComponentInfo& info) {
    CheckCompatibility(info);
    ROBO_LOG_INFO("component") << info.name << " source=" << info.source_id << " built_with_sdk=" << ToString(info.built_with_sdk);
}
}
RobotIntegration::RobotIntegration(std::string directory, std::chrono::milliseconds delay)
    : map_directory_(std::move(directory)), mapper_test_delay_(delay) {}
RobotIntegration::~RobotIntegration() = default;
void RobotIntegration::Configure(MappingOptions options) {
    if (impl_) throw std::logic_error("already configured");
    impl_ = std::make_unique<Impl>(options, map_directory_, outputs_, mapper_test_delay_);
}
void RobotIntegration::SetOutputs(ProductOutputs outputs) { outputs_ = std::move(outputs); }
MapView RobotIntegration::map_view(bool baseline) const {
    const auto& impl = Configured();
    return {impl.generation_, baseline ? impl.frozen_ : impl.live_};
}
RobotIntegration::Impl& RobotIntegration::Configured() const {
    if (!impl_) throw std::logic_error("CONFIG required");
    return *impl_;
}
void RobotIntegration::StartMapping(std::uint64_t id) { Configured().StartMapping(id); }
void RobotIntegration::Poll() { if (impl_ && impl_->active_) impl_->Poll(); }
void RobotIntegration::FinishMapping() { Configured().FinishMapping(); }
void RobotIntegration::CancelMapping() { Configured().CancelMapping(); }
void RobotIntegration::SaveMap(const std::string& name) { Configured().SaveMap(name); }
void RobotIntegration::SnapshotMap(const std::string& name) { Configured().SnapshotMap(name); }
void RobotIntegration::LoadMap(const std::string& name) { Configured().LoadMap(name); }
void RobotIntegration::PushOdometry(const TimedPose2D& pose) { Configured().inputs_.PushOdometry(pose); }
void RobotIntegration::PushMapReference(const MapPose2D& pose) { Configured().inputs_.PushReference(pose); }
TransformStatus2D RobotIntegration::transforms() const { return Configured().inputs_.status().transforms; }
void RobotIntegration::PushScan(Scan2DPtr scan) { Configured().inputs_.PushScan(std::move(scan)); }
void RobotIntegration::Shutdown() {
    if (!impl_ || !impl_->active_) return;
    impl_->Shutdown();
    if (outputs_.transforms) outputs_.transforms({});
    impl_->EmitMapping(MappingEventKind::stopped);
}
MappingStatus RobotIntegration::mapping_status() const { return Configured().Status(); }
std::size_t RobotIntegration::pending() const {
    const auto& impl = Configured();
    const auto input = impl.inputs_.status();
    return input.pending + input.submitted - impl.completed_;
}
GridSnapshot RobotIntegration::map() const { return Configured().frozen_; }
GridSnapshot RobotIntegration::live_map() const { return Configured().live_; }
DataFlowStatus RobotIntegration::data_flow_status() const { return Configured().Resources(); }

RobotIntegration::Impl::Impl(MappingOptions options, std::string directory, ProductOutputs& outputs, std::chrono::milliseconds delay)
    : outputs_(outputs), grid_(options.grid),
      computation_(compute_capacity, [this, delay](const ScanWork& work, const Cancellation& cancel) {
          if (!cancel.WaitFor(delay)) return MapResult{};
          if (computed_generation_ != cancel.generation()) {
              grid_.Reset(); computed_generation_ = cancel.generation();
          }
          const auto update = grid_.PushScan(work.scan, work.transforms, [&] { return cancel.canceled(); });
          return MapResult{grid_.Snapshot(), update, grid_.copied_bytes()};
      }), inputs_({options.grid.frame, options.max_pose_gap_us, options.max_correction_age_us},
                  [this](std::uint64_t generation, Scan2DPtr scan, ScanTransforms2D transforms) {
          return computation_.Submit(generation, {std::move(scan), transforms});
      }), store_(std::move(directory)), frame_(options.grid.frame) {
    if (delay.count() < 0 || delay.count() > 5000) throw std::invalid_argument("invalid compute test delay");
    CheckComponent(odom::GetComponentInfo());
    CheckComponent(lidar_map::GetComponentInfo());
    CheckComponent(map_updater::GetComponentInfo());
    CheckComponent(map_server::GetComponentInfo());
    store_.Activate(); grid_.Activate();
    active_ = true;
    ROBO_LOG_INFO("integration") << "sensor routing and bounded compute active; SDK " << ToString(sdk_version);
}
RobotIntegration::Impl::~Impl() { Shutdown(); }
void RobotIntegration::Impl::EmitMapping(MappingEventKind kind, const std::string& name) {
    if (outputs_.mapping) outputs_.mapping({kind, Status(), generation_, name});
}
void RobotIntegration::Impl::RequireActive() const {
    if (!active_) throw std::logic_error("integration stopped");
}
void RobotIntegration::Impl::Shutdown() {
    if (!active_) return;
    const auto input = inputs_.Close();
    CaptureInputs(input);
    const auto abandoned = input.pending + input.submitted - completed_;
    inputs_.Shutdown(); // Closes concurrent ingress before joining any computation.
    if (status_.phase == MappingPhase::mapping || status_.phase == MappingPhase::finishing) {
        status_.dropped += abandoned; status_.drop_cancel += abandoned;
        status_.phase = MappingPhase::idle;
    } else if (updating_) resources_.live_dropped += abandoned;
    computation_.Stop(); completed_ = input.submitted;
    grid_.Deactivate(); store_.Deactivate();
    latest_scan_.reset(); updating_ = false; active_ = false;
}
} // namespace roboframe
