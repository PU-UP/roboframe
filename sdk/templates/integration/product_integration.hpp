#pragma once
#include "stream_component.hpp"
#include <roboframe/core/integration.hpp>

namespace integration_template {
// Wiring example, not a required base class or another executor owner.
class ProductIntegration final : public roboframe::Integration {
public:
    using Input = StreamComponent::Input;
    using Output = StreamComponent::Output;
    using Execution = StreamComponent::Execution;
    explicit ProductIntegration(Execution::Handler compute) : component_(std::move(compute)) {}
    ~ProductIntegration() override { output_={}; Shutdown(); }
    // Control owner only; output callbacks must not block, throw or reenter.
    void SetOutput(std::function<void(Output)> output) { output_=std::move(output); }
    roboframe::Admission Push(Input input) { return component_.Push(std::move(input)); }
private:
    void OnStart(std::uint64_t) override { component_.Activate(); }
    void CloseInputs() noexcept override { component_.RequestDeactivate(); }
    void OnStop() noexcept override { component_.Deactivate(); }
    void OnReset() override {}
    void OnPoll() override {
        if (runtime_status().state!=roboframe::RuntimeState::running) return;
        Execution::Result result;
        for (std::size_t i=0; i<8 && component_.Take(result); ++i) {
            if (result.error || result.canceled) { Fail(); return; }
            if (output_) output_(result.output);
        }
    }
    StreamComponent component_;
    std::function<void(Output)> output_;
};
} // namespace integration_template
