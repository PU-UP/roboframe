#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/core/server.hpp>
#include <roboframe/datatype/navigation2d.hpp>
#include <roboframe/datatype/transforms2d.hpp>
#include <controller/version.hpp>
namespace controller {
roboframe::ComponentInfo GetComponentInfo();
enum class ControlStatus { idle, tracking, reached, blocked, invalid_tf };
struct ControlResult { ControlStatus status=ControlStatus::idle; roboframe::Twist2D velocity; };
struct FollowPathGoal {
    roboframe::Path2DPtr path;
    roboframe::GridSnapshot planned_map;
};
struct ControlFeedback {
    ControlResult control;
    std::int64_t pose_stamp_us=0;
};
class ControllerComponent final : public roboframe::Component {
public:
    using Execution=roboframe::Server<FollowPathGoal,ControlFeedback,ControlStatus>;
    using Clock=Execution::Clock;
    // Worker callback: bounded enqueue only; no blocking IO or reentry.
    using Output=std::function<void(const ControlFeedback&)>;
    explicit ControllerComponent(std::chrono::milliseconds period=std::chrono::milliseconds(50),
                                 std::chrono::milliseconds tf_timeout=std::chrono::milliseconds(500),
                                 Output output={});
    ~ControllerComponent() override { execution_.Shutdown(); }
    void Activate() override { execution_.Activate(); }
    void Deactivate() override { execution_.Deactivate(); }
    roboframe::Lifecycle lifecycle() const override {
        return execution_.active() ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
    }
    Execution& server() { return execution_; }
    void UpdateTransforms(roboframe::TransformStatus2D tf,Clock::time_point received);
    void UpdateMap(roboframe::GridSnapshot map);
    void ClearInputs(); // Inactive and quiescent.
private:
    Execution::Step StepControl(const Execution::Context& context);
    void SetPath(roboframe::Path2DPtr path,roboframe::GridSnapshot map);
    void ClearPath() noexcept { path_.reset();planned_map_.reset();index_=0; }
    ControlResult Compute(const roboframe::TransformStatus2D& tf,roboframe::GridSnapshot current);
    bool Clear(const roboframe::Grid& map,const roboframe::Pose2D& pose) const;
    const std::chrono::milliseconds tf_timeout_;
    const Output output_;
    mutable std::mutex input_mutex_;
    roboframe::TransformStatus2D tf_;
    Clock::time_point odometry_received_{};
    roboframe::GridSnapshot map_;
    // Algorithm state belongs exclusively to the executor worker.
    roboframe::Path2DPtr path_;
    roboframe::GridSnapshot planned_map_;
    std::size_t index_=0;
    Execution execution_; // Join before destroying state and callback.
};
} // namespace controller
