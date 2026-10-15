#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/core/server.hpp>

namespace component_template {
class GoalComponent final : public roboframe::Component {
public:
    using Execution=roboframe::Server<int,int,int>;
    explicit GoalComponent(Execution::Hooks hooks):execution_(std::chrono::milliseconds(10),std::move(hooks)) {}
    ~GoalComponent() override { execution_.Shutdown(); }
    void Activate() override { execution_.Activate(); }
    void RequestDeactivate() { execution_.RequestDeactivate(); }
    void Deactivate() override { execution_.Deactivate(); }
    roboframe::Lifecycle lifecycle() const override {
        return execution_.active() ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
    }
    Execution& server() { return execution_; }
private:
    Execution execution_;
};
} // namespace component_template
