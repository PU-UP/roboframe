#pragma once
#include <sample_accumulator/component.hpp>
#include <roboframe/log.hpp>

namespace roboframe {
// Product assembly and resource lifetime; no protocol or task workflow.
struct Components {
    sample_accumulator::Component accumulator;

    Components() {
        const auto info = sample_accumulator::GetComponentInfo();
        CheckCompatibility(info, sample_accumulator::contract_version);
        ROBO_LOG_INFO("integration") << info.name << " implementation=" << info.implementation_version
            << " SDK=" << ToString(info.built_with_sdk) << " contract=" << ToString(info.contract);
    }
    void Activate() { accumulator.Activate(); }
    void Shutdown() {
        const auto task = accumulator.status();
        if (task.state == TaskState::running || task.state == TaskState::canceling)
            accumulator.Cancel(task.id);
        accumulator.Deactivate();
    }
};
} // namespace roboframe
