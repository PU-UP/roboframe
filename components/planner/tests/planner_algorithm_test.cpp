#include <planner/planner_component.hpp>
#include <stdexcept>
#include <thread>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("planner algorithm test failed"); }
int main() {
    auto map=std::make_shared<Grid>();map->spec={30,30,0.2,10,20,GridFrame::map};
    map->revision=7;map->stamp_us=10;map->cells=GridCells(900,254);
    for (int y=0;y<30;++y) if (y<18 || y>26) map->cells.Set(y*30+15,0);
    planner::PlannerComponent component;component.Activate();
    TransformStatus2D tf{true,{{10,{1,2,0}},{10,{10,20,0}}}};
    auto plan=[&](GridSnapshot grid,Pose2D goal) {
        const auto handle=component.server().Submit({grid,tf,goal});
        planner::PlannerComponent::Execution::Result result;
        const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
        while (!component.server().TakeResult(handle,result)) {
            Check(std::chrono::steady_clock::now()<deadline);std::this_thread::yield();
        }
        return result;
    };
    const auto execution=plan(map,{15,22,0});
    const auto& result=execution.value;
    Check(execution.state==TaskState::succeeded && !execution.error);
    Check(result.status==planner::PlanStatus::success && result.path->map_revision==7);
    Check(result.path->poses.front().x==11 && result.path->poses.front().y==22);
    bool detour=false;
    for (const auto& pose:result.path->poses) {
        const int x=std::floor((pose.x-10)/0.2),y=std::floor((pose.y-20)/0.2);
        Check(map->cells[y*30+x]==254);if (pose.y>23.6) detour=true;
    }
    Check(detour && map->revision==7 && map->cells[15]==0);
    auto blocked=std::make_shared<Grid>(*map);
    for (int y=0;y<30;++y) blocked->cells.Set(y*30+15,205);
    Check(plan(blocked,{15,22,0}).value.status==planner::PlanStatus::unreachable);
    Check(plan(map,{10,20,0}).value.status==planner::PlanStatus::unreachable);
    tf.valid=false;
    const auto failed=plan(map,{15,22,0});
    Check(failed.state==TaskState::failed && bool(failed.error));
    bool rejected=false;
    try { std::rethrow_exception(failed.error); } catch (const std::invalid_argument&) { rejected=true; }
    Check(rejected);
    component.Deactivate();
}
