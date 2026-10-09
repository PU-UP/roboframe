#include <value_scale/component.hpp>
#include <roboframe/log.hpp>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace value_scale {
static_assert(roboframe::Compatible(roboframe::sdk_version, {1, 0, 0}), "ValueScale requires SDK 1.x");

roboframe::ComponentInfo GetComponentInfo() {
    return {"value-scale", VALUE_SCALE_VERSION, roboframe::sdk_version, contract_version};
}
Component::Component(Config config) : config_(config) {
    if (!std::isfinite(config.gain)) throw std::invalid_argument("gain must be finite");
}
void Component::SetOutputCallback(std::function<void(Output)> callback) {
    output_ = std::move(callback);
}
void Component::Activate() {
    lifecycle_ = roboframe::Lifecycle::active;
    ROBO_LOG_INFO("value-scale") << "active; gain=" << config_.gain;
}
void Component::Deactivate() { lifecycle_ = roboframe::Lifecycle::inactive; }
void Component::PushValue(Input input) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("component inactive");
    if (!std::isfinite(input.value)) throw std::invalid_argument("input must be finite");
    const double result = input.value * config_.gain;
    if (!std::isfinite(result)) throw std::overflow_error("scaled value is not finite");
    if (output_) output_({result});
}
} // namespace value_scale
