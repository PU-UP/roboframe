#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/datatype/geometry2d.hpp>
#include <odom/version.hpp>
#include <functional>

namespace odom {
roboframe::ComponentInfo GetComponentInfo();
// Framework example: future DRComponent/VIOComponent may independently provide the same pose semantics.
// Input/output: acquisition timestamp in microseconds, T_odom_base in metres/radians.
// Output is synchronous; its reference lasts only for this call. Serialize all calls.
// The sink must not reenter/destroy this component. If it throws, the sample stays consumed.
class PassthroughOdomComponent final : public roboframe::Component {
public:
    using Output = std::function<void(const roboframe::TimedPose2D&)>;
    explicit PassthroughOdomComponent(Output output);
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { lifecycle_ = roboframe::Lifecycle::inactive; last_stamp_us_ = -1; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void PushOdometry(const roboframe::TimedPose2D& pose);
private:
    Output output_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    std::int64_t last_stamp_us_ = -1;
};
} // namespace odom
