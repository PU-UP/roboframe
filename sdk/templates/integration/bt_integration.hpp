#pragma once
#include "../component/goal_component.hpp"
#include <roboframe/core/integration.hpp>
#include <roboframe/orchestration/server_action.hpp>

namespace integration_template {
// The handler is test/example work. A product substitutes real Server components and typed goals.
class BtIntegration final : public roboframe::Integration {
public:
    using Work=component_template::GoalComponent::Execution;
    explicit BtIntegration(Work::Hooks hooks)
        :work_(std::move(hooks)),
         action_(work_.server(),[this] { return goal_; },[this](const Work::Result& result) noexcept { result_=result; }),
         tree_({{"Execute",[this] { return action_.Tick(); },[this] { action_.Halt(); }}}) {}
    ~BtIntegration() override { Shutdown(); }
    void Begin(int goal) {
        RequireRunning();
        if (running_ || !action_.Drained()) throw std::logic_error("task busy");
        tree_.Halt();action_.Reset();goal_=goal;running_=true;result_={};
    }
    void Cancel() { tree_.Halt();action_.Halt();running_=false; }
    bool settled() { return !running_ && action_.Drained(); }
    // Includes canceled and failed (with error), even when collected by settled/Stop.
    const Work::Result& result() const { return result_; }
private:
    void OnStart(std::uint64_t) override { work_.Activate(); }
    void CloseInputs() noexcept override { Cancel(); work_.RequestDeactivate(); }
    void OnStop() noexcept override { work_.Deactivate(); action_.Drained(); }
    void OnReset() override { result_={}; }
    void OnPoll() override {
        if (running_ && tree_.Tick()!=roboframe::bt::Status::running) running_=false;
        if (!running_) action_.Drained();
    }
    component_template::GoalComponent work_;
    int goal_=0;
    bool running_=false;
    Work::Result result_;
    roboframe::bt::ServerAction<Work> action_;
    roboframe::bt::Sequence tree_;
};
}
