#include <planner/planner_component.hpp>
#include <thread>
#include <type_traits>
using namespace roboframe;
void Check(bool v) { if (!v) throw std::runtime_error("planner execution check failed"); }
template<class F> void Until(F f) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (!f()) { Check(std::chrono::steady_clock::now()<deadline);std::this_thread::yield(); }
}
int main() {
    static_assert(std::is_base_of<Component,planner::PlannerComponent>::value,"component owns execution");
    static_assert(!std::is_base_of<Component,planner::PlannerComponent::Execution>::value,"executor is optional");
    auto map=std::make_shared<Grid>();map->spec={20,20,0.2,0,0,GridFrame::map};map->revision=1;map->cells=GridCells(400,254);
    planner::PlannerComponent component;component.Activate();
    planner::PlanGoal goal{map,{true,{{0,{1.1,1.1,0}},{0,{}}}},{2.1,2.1,0}};
    auto handle=component.server().Submit(goal);
    planner::PlannerComponent::Execution::Result result;
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::succeeded && result.value.path && result.value.path->map_revision==1);
    component.Deactivate();component.Activate();
    goal.goal={99,99,0};handle=component.server().Submit(goal);
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::failed);
    auto large=std::make_shared<Grid>();large->spec={1000,1000,0.2,0,0,GridFrame::map};
    large->revision=2;large->cells=GridCells(1000000,254);
    goal.map=large;goal.goal={190.1,190.1,0};
    handle=component.server().Submit(goal);
    Check(component.server().RequestCancel(handle));
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::canceled && !result.error && !result.value.path);
    component.Deactivate();component.Activate();
    goal.map=map;goal.goal={2.1,2.1,0};handle=component.server().Submit(goal);
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::succeeded);
    component.Deactivate();
}
