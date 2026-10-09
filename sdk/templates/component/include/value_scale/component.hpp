#pragma once
#include <roboframe/component.hpp>
#include <roboframe/version.hpp>
#include <functional>

namespace value_scale {
// Keep this small contract here; split into contract.hpp only when it grows.
constexpr roboframe::Version contract_version{1, 0, 0};
struct Config { double gain = 1.0; };
struct Input { double value; };
struct Output { double value; };
roboframe::ComponentInfo GetComponentInfo();

// Single caller thread. Callbacks must not throw, reenter, or destroy this component.
class Component final : public roboframe::Component {
public:
    explicit Component(Config config);
    void SetOutputCallback(std::function<void(Output)> callback);
    void Activate() override;
    void Deactivate() override;
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void PushValue(Input input);
private:
    const Config config_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    std::function<void(Output)> output_;
};
} // namespace value_scale
