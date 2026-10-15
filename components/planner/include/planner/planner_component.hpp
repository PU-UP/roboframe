#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/core/server.hpp>
#include <roboframe/datatype/navigation2d.hpp>
#include <roboframe/datatype/transforms2d.hpp>
#include <planner/version.hpp>
namespace planner {
roboframe::ComponentInfo GetComponentInfo();
enum class PlanStatus { success, unreachable, canceled };
struct PlanResult { PlanStatus status=PlanStatus::unreachable; roboframe::Path2DPtr path; };
struct PlanGoal {
    roboframe::GridSnapshot map;
    roboframe::TransformStatus2D tf;
    roboframe::Pose2D goal;
};
struct PlanFeedback {};
class PlannerComponent final : public roboframe::Component {
public:
    using Execution=roboframe::Server<PlanGoal,PlanFeedback,PlanResult>;
    explicit PlannerComponent(double robot_radius=0.2);
    ~PlannerComponent() override { execution_.Shutdown(); }
    void Activate() override { execution_.Activate(); }
    void Deactivate() override { execution_.Deactivate(); }
    roboframe::Lifecycle lifecycle() const override {
        return execution_.active() ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
    }
    Execution& server() { return execution_; }
private:
    PlanResult Plan(const PlanGoal& goal,const Execution::Context& context) const;
    const double radius_;
    Execution execution_; // Join before destroying algorithm state.
};
} // namespace planner
