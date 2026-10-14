#pragma once
#include <roboframe/core/input_gate.hpp>
#include <roboframe/core/integration.hpp>
#include <memory>
#include <vector>

namespace integration_template {
// Replace these ports and the injected handler with your actual component interface.
// This is a reading/copying template, not a second framework base or mandatory wrapper.
class ProductIntegration final : public roboframe::Integration {
public:
    using Input = std::shared_ptr<const std::vector<unsigned char>>;
    using Output = std::size_t;
    using Execution = roboframe::ExecutionChannel<Input, Output>;
    explicit ProductIntegration(Execution::Handler compute) : execution_(8, std::move(compute)) {
        // Product-owned assembly: connect components here; components know only their own ports.
        UseExecution(execution_);
    }
    ~ProductIntegration() { output_ = {}; Shutdown(); }
    // Configure on the control owner; callback must not block, throw or reenter.
    void SetOutput(std::function<void(Output)> output) { output_ = std::move(output); }
    roboframe::Admission Push(Input input) {
        if (!input || input->size() > 1024 * 1024) throw std::invalid_argument("invalid payload");
        auto access = ingress_.Lock();
        if (!access.open()) return roboframe::Admission::closed;
        return execution_.Submit(access.generation(), std::move(input));
    }
private:
    void OnStart(std::uint64_t generation) override {
        // Activate selected components before opening ingress.
        ingress_.Open(generation);
    }
    void CloseInputs() noexcept override { ingress_.Close(); }
    void OnStop() noexcept override { /* Deactivate after execution has quiesced. */ }
    void OnReset() override { /* Clear product working state; choose asset policy explicitly. */ }
    void OnPoll() override {
        Dispatch(execution_, 8, [this](Execution::Result result) {
            if (result.error || result.canceled) { Fail(); return; }
            // Connect a downstream input here, or publish on this control owner.
            if (output_) output_(result.output);
        });
    }
    roboframe::InputGate ingress_;
    Execution execution_;
    std::function<void(Output)> output_;
};
} // namespace integration_template
