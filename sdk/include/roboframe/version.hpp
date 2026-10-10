#pragma once
#include <roboframe/sdk_version.hpp>
#include <stdexcept>
#include <string>

namespace roboframe {
struct Version {
    unsigned major;
    unsigned minor;
    unsigned patch;
};
constexpr Version sdk_version{
    ROBOFRAME_SDK_VERSION_MAJOR, ROBOFRAME_SDK_VERSION_MINOR, ROBOFRAME_SDK_VERSION_PATCH};

// A minor/patch release must preserve compatibility with earlier releases of that major.
constexpr bool Compatible(Version available, Version required) {
    return available.major == required.major &&
        (available.minor > required.minor ||
         (available.minor == required.minor && available.patch >= required.patch));
}
inline std::string ToString(Version version) {
    return std::to_string(version.major) + "." + std::to_string(version.minor) +
        "." + std::to_string(version.patch);
}
struct ComponentInfo {
    const char* name;
    const char* source_id;
    Version built_with_sdk;
};
// Keep the caller SDK version local even when linking a component built with another SDK.
static inline void CheckCompatibility(const ComponentInfo& component) {
    if (!Compatible(sdk_version, component.built_with_sdk))
        throw std::runtime_error(std::string(component.name) + ": SDK mismatch: host=" +
            ToString(sdk_version) + ", component built with=" + ToString(component.built_with_sdk));

}
} // namespace roboframe
