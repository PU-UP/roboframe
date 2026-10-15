#pragma once
#include <odom/odom_component.hpp>
#include <roboframe/core/integration.hpp>
#include <roboframe/datatype/geometry2d.hpp>
#include <cstdint>

namespace odom_session {
// Frozen or live view of one session. Pose fields are meaningful only when has_pose is true.
struct SessionView {
    std::uint64_t valid_samples = 0;
    bool has_pose = false;
    roboframe::TimedPose2D latest{};
    double distance_m = 0;
};

// One control thread owns Start, Stop, Reset, Poll, Push, and view.
// Odom stays a synchronous validator/passthrough; this type owns session policy.
// Push copies each accepted pose from the component callback. The callback must not reenter.
// Distance is the sum of planar distances between adjacent accepted poses, in push order.
// Yaw is not included. Stop freezes the sum; the next Start clears it and does not bridge sessions.
// There is no behavior tree: odom has no goal port.
class SessionIntegration final : public roboframe::Integration {
public:
    SessionIntegration();
    ~SessionIntegration() override;
    void Push(const roboframe::TimedPose2D& pose);
    SessionView view() const;
private:
    void OnStart(std::uint64_t generation) override;
    void CloseInputs() noexcept override;
    void OnStop() noexcept override;
    void OnReset() override;
    void OnPoll() override;
    void Retain(const roboframe::TimedPose2D& pose);

    bool inputs_open_ = false;
    bool has_pose_ = false;
    bool has_segment_ = false;
    roboframe::TimedPose2D latest_{};
    roboframe::TimedPose2D segment_{};
    double distance_m_ = 0;
    SessionView frozen_{};
    odom::OdomComponent odom_;
};
} // namespace odom_session
