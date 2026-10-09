#include <sample_accumulator/component.hpp>
#include <roboframe/log.hpp>
#include <iostream>
#include <stdexcept>

int main() {
    try {
        const auto info = sample_accumulator::GetComponentInfo();
        roboframe::CheckCompatibility(info, sample_accumulator::contract_version);
        ROBO_LOG_INFO("direct-demo") << info.name << " implementation=" << info.implementation_version
            << " SDK=" << roboframe::ToString(info.built_with_sdk);
        sample_accumulator::Component component;
        unsigned results = 0;
        bool correct = false;
        component.SetCallbacks(
            [](const sample_accumulator::Feedback& feedback) {
                std::cout << "feedback: goal=" << feedback.goal_id << " count=" << feedback.count
                          << " sum=" << feedback.sum << std::endl;
            },
            [&](const sample_accumulator::Result& result) {
                ++results;
                correct = result.state == roboframe::TaskState::succeeded &&
                    result.count == 2 && result.sum == 8;
                std::cout << "result: goal=" << result.goal_id << " sum=" << result.sum << std::endl;
            });
        component.Activate();
        component.Submit({1, 2});
        component.PushSample({3});
        component.PushSample({5});
        if (results != 1 || !correct) throw std::runtime_error("unexpected terminal result");
        component.Deactivate();
        std::cout << "PASS: direct component main (no integration or IPC)" << std::endl;
        return 0;
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("direct-demo") << error.what();
        return 1;
    }
}
