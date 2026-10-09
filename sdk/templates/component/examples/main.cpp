#include <value_scale/component.hpp>
#include <roboframe/log.hpp>
#include <limits>
#include <stdexcept>

void Require(bool value) {
    if (!value) throw std::runtime_error("component self-check failed");
}
template<class Function> void Reject(Function function) {
    bool rejected = false;
    try { function(); } catch (const std::exception&) { rejected = true; }
    Require(rejected);
}
int main() {
    try {
        roboframe::CheckCompatibility(value_scale::GetComponentInfo(), value_scale::contract_version);
        Reject([] { value_scale::Component bad({std::numeric_limits<double>::infinity()}); });
        value_scale::Component component({2.0});
        double result = 0;
        int outputs = 0;
        component.SetOutputCallback([&](value_scale::Output output) {
            result = output.value;
            ++outputs;
        });
        Reject([&] { component.PushValue({3.0}); });
        component.Activate();
        component.PushValue({3.0});
        Require(outputs == 1 && result == 6.0);
        Reject([&] { component.PushValue({std::numeric_limits<double>::quiet_NaN()}); });
        Reject([&] { component.PushValue({std::numeric_limits<double>::max()}); });
        Require(outputs == 1);
        component.Deactivate();
        Reject([&] { component.PushValue({3.0}); });
        component.Activate();
        component.PushValue({4.0});
        Require(outputs == 2 && result == 8.0);
        component.Deactivate();
        ROBO_LOG_INFO("self-check") << "PASS: independent value-scale component";
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("self-check") << error.what();
        return 1;
    }
}
