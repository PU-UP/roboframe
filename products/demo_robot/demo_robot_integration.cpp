#include "products/demo_robot/demo_robot_integration_impl.hpp"
#include <utility>
#include <roboframe/core/log.hpp>
namespace demo_robot {
using namespace roboframe;
DemoRobotIntegration::DemoRobotIntegration(std::string directory,std::chrono::milliseconds delay)
    : map_directory_(std::move(directory)),mapper_test_delay_(delay) {}
DemoRobotIntegration::~DemoRobotIntegration() { SetOutputs({}); Shutdown(); }
void DemoRobotIntegration::Configure(DemoRobotOptions options) {
    if (impl_) throw std::logic_error("already configured");
    if (options.control_period_ms<=0 || options.control_period_ms>1000 ||
        options.tf_timeout_ms<=0 || options.tf_timeout_ms>60000 ||
        options.localization_test_delay_ms<0 || options.localization_test_delay_ms>5000)
        throw std::invalid_argument("invalid execution timing");
    auto impl=std::make_unique<Impl>(options,map_directory_,outputs_,mapper_test_delay_);
    UseExecution(impl->inputs_.execution()); UseExecution(impl->computation_); UseServer(impl->planner_); UseServer(impl->controller_); impl_=std::move(impl);
}
DemoRobotIntegration::Impl& DemoRobotIntegration::Configured() const {
    if (!impl_) throw std::logic_error("CONFIG required");
    return *impl_;
}
void DemoRobotIntegration::SetOutputs(DemoRobotOutputs outputs) {
    if (impl_) {
        // Clearing waits for any in-flight velocity callback before releasing its receiver.
        std::lock_guard<std::mutex> lock(impl_->velocity_mutex_);
        impl_->velocity_output_=outputs.velocity;
    }
    outputs_=std::move(outputs);
}
void DemoRobotIntegration::ValidateStart() const { Configured(); }
void DemoRobotIntegration::OnStateChanged() {
    if (outputs_.runtime) outputs_.runtime(runtime_status());
    if (runtime_status().state!=RuntimeState::running && impl_) {
        if (outputs_.transforms) outputs_.transforms({});
        if (impl_->task_interrupted_) {
            impl_->task_interrupted_=false;
            impl_->EmitMapping(runtime_status().state==RuntimeState::error ? MappingEventKind::failed : MappingEventKind::stopped);
        }
    }
}
void DemoRobotIntegration::OnStart(std::uint64_t generation) { Configured().Start(generation); }
void DemoRobotIntegration::CloseInputs() noexcept {
    if (!impl_) return;
    // Controller cleanup emits its final zero before the execution wait completes.
    if (impl_->navigation_.busy()) impl_->EndNavigation(NavigationPhase::canceled,"runtime_stopped");
    impl_->inputs_.Close();
}
void DemoRobotIntegration::OnStop() noexcept { if (impl_) impl_->Stop(); }
void DemoRobotIntegration::OnReset() { if (impl_) impl_->ResetData(); }
void DemoRobotIntegration::OnPoll() {
    if (!impl_) return;
    if (runtime_status().state==RuntimeState::running) {
        impl_->PublishInputs();
        impl_->controller_.UpdateMap(impl_->NavigationMap());
    }
    Dispatch(impl_->inputs_.execution(),Impl::compute_capacity-impl_->computation_.status().retained,[this](SensorIngress::LocalizationExecution::Result result) {
        if (result.error || result.canceled) { Fail();return; }
        auto input=std::move(result.output);
        // FIFO handoff preserves atlas order; every admitted scan receives final accounting.
        if (impl_->computation_.Submit(result.generation,input)!=Admission::accepted) {
            ++impl_->completed_;++impl_->resources_.compute_rejected;
            impl_->RecordTaskResult(input,Impl::MapResult{});
            impl_->AdvanceTasks();
        }
    });
    Dispatch(impl_->computation_,Impl::compute_capacity,[this](Impl::MapExecution::Result result) {
        if (result.error || result.canceled) {
            try {
                if (result.error) std::rethrow_exception(result.error);
                throw std::runtime_error("unexpected cancellation in current runtime");
            } catch (const std::exception& error) {
                ROBO_LOG_ERROR("runtime") << error.what();
            } catch (...) {
                ROBO_LOG_ERROR("runtime") << "unknown computation failure";
            }
            Fail(); // This product cannot safely reuse a partly updated atlas.
            return;
        }
        impl_->OnMapResult(std::move(result));
    });
    if (runtime_status().state==RuntimeState::running) {
        impl_->AdvanceNavigation();
        if (!impl_->PendingCompute()) impl_->Publish(impl_->latest_scan_,true);
        impl_->AdvanceTasks();
    }
}
void DemoRobotIntegration::StartMapping(std::uint64_t id) { RequireRunning(); Configured().StartMapping(id); }
void DemoRobotIntegration::FinishMapping() { RequireRunning(); Configured().FinishMapping(); }
void DemoRobotIntegration::CancelMapping() { RequireRunning(); Configured().CancelMapping(); }
void DemoRobotIntegration::Navigate(NavigationGoal goal) { RequireRunning(); Configured().Navigate(goal); }
void DemoRobotIntegration::CancelNavigation() {
    RequireRunning(); auto& impl=Configured();
    if (!impl.navigation_.busy()) throw std::logic_error("no navigation task");
    impl.EndNavigation(NavigationPhase::canceled,"requested");
}
NavigationStatus DemoRobotIntegration::navigation_status() const { return Configured().navigation_.status; }
NavigationPath DemoRobotIntegration::navigation_path() const {
    const auto& task=Configured().navigation_; return {task.status.session,task.status.generation,task.path};
}
void DemoRobotIntegration::SaveMap(const std::string& name) { Configured().SaveMap(name); }
void DemoRobotIntegration::SnapshotMap(const std::string& name) { Configured().SnapshotMap(name); }
void DemoRobotIntegration::LoadMap(const std::string& name) {
    if (runtime_status().state!=RuntimeState::idle) throw std::logic_error("stop runtime before load");
    Configured().LoadMap(name);
}
void DemoRobotIntegration::PushOdometry(const TimedPose2D& pose) { Configured().inputs_.PushOdometry(pose); }
void DemoRobotIntegration::PushMapReference(const MapPose2D& pose) { Configured().inputs_.PushReference(pose); }
void DemoRobotIntegration::PushScan(Scan2DPtr scan) { Configured().inputs_.PushScan(std::move(scan)); }
TransformStatus2D DemoRobotIntegration::transforms() const { return Configured().inputs_.status().transforms; }
MappingStatus DemoRobotIntegration::mapping_status() const { return Configured().Status(); }
std::size_t DemoRobotIntegration::pending() const {
    const auto& impl=Configured(); return impl.inputs_.status().pending+impl.PendingCompute();
}
GridSnapshot DemoRobotIntegration::map() const { return Configured().frozen_; }
GridSnapshot DemoRobotIntegration::live_map() const { return Configured().live_; }
MapView DemoRobotIntegration::map_view(bool baseline) const {
    const auto& impl=Configured();
    if (impl.navigation_.busy()) throw std::logic_error("full map export disabled during navigation");
    return {baseline ? impl.baseline_generation_ : impl.generation_,baseline ? impl.frozen_ : impl.live_};
}
DataFlowStatus DemoRobotIntegration::data_flow_status() const {
    auto result=Configured().Resources(); result.stale_results=runtime_status().stale_results; return result;
}
void DemoRobotIntegration::Impl::ResetData() {
    mapping_tree_.Halt(); navigation_tree_.Halt();
    inputs_.Clear(); local_.Reset(); grid_.Reset(); slam_.Reset(); graph_.Reset();
    anchor_={}; correction_us_=-1; had_node_=false; completed_=0; copied_bytes_=0;
    computed_.reset(); live_.reset(); latest_scan_.reset(); last_publication_us_=-1;
    resources_={}; // Frozen assets and increasing task IDs survive reset.
}
void DemoRobotIntegration::Impl::Start(std::uint64_t generation) {
    ResetData(); generation_=generation;
    odom_.Activate(); slam_.Activate(); graph_.Activate(); local_.Activate(); grid_.Activate();
    controller_.UpdateTransforms({},{});controller_.UpdateMap(frozen_);
    inputs_.Open(generation);
}
void DemoRobotIntegration::Impl::Stop() noexcept {
    // Ingress is closed and computation has quiesced before this hook.
    task_interrupted_=task_.busy();
    mapping_tree_.Halt();
    navigation_tree_.Halt();
    AdvanceNavigation();
    completed_=inputs_.status().submitted;
    odom_.Deactivate(); slam_.Deactivate(); graph_.Deactivate(); local_.Deactivate(); grid_.Deactivate();
}
} // namespace demo_robot
