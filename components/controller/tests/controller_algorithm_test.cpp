#include <controller/controller_component.hpp>
#include <stdexcept>
#include <thread>
using namespace roboframe;
void Require(bool ok,int line) { if (!ok) throw std::runtime_error("controller algorithm check line "+std::to_string(line)); }
#define Check(x) Require((x),__LINE__)
template<class F> void Until(F f) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (!f()) { Check(std::chrono::steady_clock::now()<end);std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
int main() {
    auto map=std::make_shared<Grid>();map->spec={30,30,0.2,10,20,GridFrame::map};
    map->revision=7;map->cells=GridCells(900,254);
    auto path=std::make_shared<Path2D>();path->map_revision=7;
    path->poses={{12.1,22.1,0},{12.3,22.1,0},{12.5,22.1,0}};
    controller::ControllerComponent component(std::chrono::milliseconds(2),std::chrono::milliseconds(1000));
    component.Activate();component.UpdateMap(map);
    TransformStatus2D tf{true,{{10,{2.1,2.1,0}},{10,{10,20,0}}}};
    auto locate=[&] { component.UpdateTransforms(tf,std::chrono::steady_clock::now()); };
    locate();auto handle=component.server().Submit({path,map});
    auto feedback=[&] { return component.server().GetStatus(handle).feedback; };
    auto result=[&] {
        controller::ControllerComponent::Execution::Result value;
        Until([&] { return component.server().TakeResult(handle,value); });
        Check(component.server().GetStatus(handle).feedback.control.velocity.linear==0);
        return value;
    };
    Until([&] { return feedback().control.velocity.linear>0; });
    Check(feedback().control.status==controller::ControlStatus::tracking && feedback().control.velocity.linear<=0.3);
    tf.transforms.odom_from_base.pose.yaw=1.57;locate();
    Until([&] { return feedback().control.velocity.angular<0; });
    Check(feedback().control.velocity.linear==0);
    auto obstacle=std::make_shared<Grid>(*map);obstacle->cells.Set(10*30+11,0);++obstacle->revision;
    component.UpdateMap(obstacle);
    Check(result().value==controller::ControlStatus::blocked);
    component.UpdateMap(map);component.UpdateTransforms({},std::chrono::steady_clock::now());
    handle=component.server().Submit({path,map});Check(result().value==controller::ControlStatus::invalid_tf);
    tf.transforms.odom_from_base.pose={2.1,2.1,0};locate();
    handle=component.server().Submit({path,map});
    Until([&] { return feedback().control.velocity.linear>0; });
    tf.transforms.odom_from_base={20,{2.3,2.1,0}};locate();
    Until([&] { return feedback().pose_stamp_us==20; });
    tf.transforms.odom_from_base={30,{2.5,2.1,0}};locate();
    const auto reached=result();
    Check(reached.state==TaskState::succeeded && reached.value==controller::ControlStatus::reached);
    auto malformed=std::make_shared<Path2D>(*path);malformed->map_revision=8;
    handle=component.server().Submit({malformed,map});
    const auto failed=result();
    Check(failed.state==TaskState::failed && bool(failed.error));
    bool rejected=false;
    try { std::rethrow_exception(failed.error); } catch (const std::invalid_argument&) { rejected=true; }
    Check(rejected);
    component.Deactivate();
}
