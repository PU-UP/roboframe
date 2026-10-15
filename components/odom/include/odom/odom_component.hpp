#pragma once
#include <odom/version.hpp>
#include <roboframe/core/component.hpp>
#include <roboframe/datatype/geometry2d.hpp>
#include <cstdint>
#include <functional>

namespace odom {
inline roboframe::ComponentInfo Info() {
    return {component_name, source_id, roboframe::sdk_version};
}

// Synchronous odometry intake. One caller thread owns construction, Activate, Push, Latest, and Deactivate.
// Input and output are T_odom_base: metres, radians, acquisition time in microseconds.
// Push validates the sample, retains a copy as the latest pose, then calls the sink.
// Samples are not integrated, filtered, or reordered. A timestamp that does not increase is still accepted.
// The sink reference lasts only for that call. The sink must not reenter or destroy this component.
// If the sink throws, the sample stays retained and counts as accepted.
// Deactivate drops the retained pose. accepted() keeps the current activation's count until the next Activate.
class OdomComponent final : public roboframe::Component {
public:
    using Output = std::function<void(const roboframe::TimedPose2D&)>;
    explicit OdomComponent(Output output);
    void Activate() override;
    void Deactivate() override;
    roboframe::Lifecycle lifecycle() const override;
    void Push(const roboframe::TimedPose2D& pose);
    bool Latest(roboframe::TimedPose2D& pose) const;
    std::uint64_t accepted() const { return accepted_; }
private:
    Output output_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    bool has_latest_ = false;
    roboframe::TimedPose2D latest_{};
    std::uint64_t accepted_ = 0;
};
} // namespace odom
