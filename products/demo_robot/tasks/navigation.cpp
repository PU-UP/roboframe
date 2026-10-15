#include "products/demo_robot/demo_robot_integration_impl.hpp"
namespace demo_robot {
using namespace roboframe;
bool DemoRobotIntegration::Impl::NavigationTFReady(const SensorIngress::State& state) const {
    return state.transforms.valid && std::chrono::steady_clock::now()-state.odometry_received<=tf_timeout_;
}
void DemoRobotIntegration::Impl::EmitNavigation() { if (outputs_.navigation) outputs_.navigation(navigation_.status); }
void DemoRobotIntegration::Impl::EmitVelocity(Twist2D velocity,std::int64_t stamp) {
    if (stamp<0) stamp=inputs_.status().transforms.transforms.odom_from_base.stamp_us;
    std::lock_guard<std::mutex> lock(velocity_mutex_);
    if (velocity_output_) velocity_output_({velocity_session_,velocity_generation_,++velocity_sequence_,stamp,velocity});
}
void DemoRobotIntegration::Impl::Navigate(NavigationGoal goal) {
    if (navigation_.busy()) throw std::logic_error("navigation already active; wait for cancellation before replacing goal");
    if (!goal.session || goal.session<=navigation_.status.session) throw std::invalid_argument("session id must increase");
    ValidatePose({0,goal.pose});
    const auto map=NavigationMap();const auto state=inputs_.status();
    if (!map || map->spec.frame!=GridFrame::map || frame_!=GridFrame::map)
        throw std::logic_error("navigation requires a map-frame map");
    if (!NavigationTFReady(state)) throw std::logic_error("navigation requires fresh valid TF");
    navigation_tree_.Halt();plan_action_.Reset();follow_action_.Reset();navigation_={};
    navigation_.status={NavigationPhase::planning,goal.session,generation_,{}};
    navigation_.input={map,state.transforms,goal.pose};
    {
        std::lock_guard<std::mutex> lock(velocity_mutex_);
        velocity_session_=goal.session;velocity_generation_=generation_;
    }
    EmitVelocity({});EmitNavigation();AdvanceNavigation();
}
void DemoRobotIntegration::Impl::ReleaseNavigation() noexcept {
    plan_action_.Halt();follow_action_.Halt();
}
void DemoRobotIntegration::Impl::EndNavigation(NavigationPhase phase,const char* reason) {
    const bool has_controller=bool(follow_action_.handle());
    navigation_tree_.Halt();ReleaseNavigation();
    navigation_.ending=phase;navigation_.status.reason=reason;
    navigation_.status.phase=plan_action_.Drained() && follow_action_.Drained() ? phase : NavigationPhase::canceling;
    // Once submitted, the controller owns output through its final zero.
    if (!has_controller) EmitVelocity({});
    EmitNavigation();
}
void DemoRobotIntegration::Impl::AdvanceNavigation() {
    if (!navigation_.busy()) return;
    if (navigation_.status.phase==NavigationPhase::canceling) {
        if (plan_action_.Drained() && follow_action_.Drained()) {
            navigation_.status.phase=navigation_.ending;EmitNavigation();
        }
        return;
    }
    if (navigation_.status.phase==NavigationPhase::planning && !NavigationTFReady(inputs_.status())) {
        EndNavigation(NavigationPhase::failed,"tf_unavailable");return;
    }
    try {
        const auto status=navigation_tree_.Tick();
        if (status!=bt::Status::running) {
            const auto reason=navigation_.status.reason;
            EndNavigation(status==bt::Status::success ? NavigationPhase::succeeded : NavigationPhase::failed,reason.c_str());
        }
    } catch (const std::exception&) {
        EndNavigation(NavigationPhase::failed,"server_unavailable");
    }
}
} // namespace demo_robot
