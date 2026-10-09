#pragma once
#include <value_scale/component.hpp>
#include <functional>
#include <utility>

// Product assembly owns configuration, wiring and resource lifetime.
// This small integration needs no workflow state or universal integration base class.
class Integration {
public:
    explicit Integration(value_scale::Config config) : component_(config) {
        roboframe::CheckCompatibility(value_scale::GetComponentInfo(), value_scale::contract_version);
    }
    Integration(const Integration&) = delete;
    Integration& operator=(const Integration&) = delete;

    void SetOutputCallback(std::function<void(value_scale::Output)> callback) {
        component_.SetOutputCallback(std::move(callback));
    }
    void Start() { component_.Activate(); }
    void PushValue(value_scale::Input value) { component_.PushValue(value); }
    void Shutdown() { component_.Deactivate(); }
private:
    value_scale::Component component_;
};
