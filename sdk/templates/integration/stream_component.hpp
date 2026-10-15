#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/core/execution.hpp>
#include <roboframe/core/input_gate.hpp>
#include <memory>
#include <vector>

namespace integration_template {
// Example ports and budgets only; replace with the real algorithm's public contract.
class StreamComponent final : public roboframe::Component {
public:
    using Input = std::shared_ptr<const std::vector<unsigned char>>;
    using Output = std::size_t;
    using Execution = roboframe::ExecutionChannel<Input, Output>;
    explicit StreamComponent(Execution::Handler compute) : execution_(8, std::move(compute)) {}
    ~StreamComponent() override { execution_.Stop(); }
    void Activate() override {
        if (lifecycle()==roboframe::Lifecycle::active) throw std::logic_error("component already active");
        execution_.Reset(++epoch_);
        ingress_.Open(epoch_);
    }
    void RequestDeactivate() {
        ingress_.Close();
        execution_.Reset(0);
    }
    void Deactivate() override {
        RequestDeactivate();
        execution_.WaitIdle();
        Execution::Result discarded;
        while (execution_.Take(discarded)) {} // Release old inputs/results before a restart.
    }
    roboframe::Lifecycle lifecycle() const override {
        auto access=ingress_.Lock();
        return access.open() ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
    }
    roboframe::Admission Push(Input input) {
        if (!input || input->size()>1024*1024) throw std::invalid_argument("invalid payload");
        auto access=ingress_.Lock();
        if (!access.open()) return roboframe::Admission::closed;
        return execution_.Submit(access.generation(),std::move(input));
    }
    bool Take(Execution::Result& result) { return execution_.Take(result); }
private:
    roboframe::InputGate ingress_;
    std::uint64_t epoch_=0;
    Execution execution_; // Declared last; hooks cannot outlive algorithm state.
};
} // namespace integration_template
