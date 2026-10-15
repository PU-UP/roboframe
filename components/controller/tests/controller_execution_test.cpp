#include <controller/controller_component.hpp>
#include <stdexcept>
#include <thread>
#include <atomic>
#include <type_traits>
using namespace roboframe;
void Check(bool value) { if (!value) throw std::runtime_error("controller execution check failed"); }
template<class F> void Until(F f) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (!f()) { Check(std::chrono::steady_clock::now()<deadline);std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
int main() {
    static_assert(std::is_base_of<Component,controller::ControllerComponent>::value,"component owns execution");
    static_assert(!std::is_base_of<Component,controller::ControllerComponent::Execution>::value,"executor is optional");
    std::atomic<int> moves{0},stops{0};
    std::atomic<bool> output_fault{false};
    const auto owner=std::this_thread::get_id();
    auto map=std::make_shared<Grid>();map->spec={30,30,0.2,10,20,GridFrame::map};map->revision=7;map->cells=GridCells(900,254);
    auto path=std::make_shared<Path2D>();path->map_revision=7;path->start_stamp_us=10;
    path->poses={{12.1,22.1,0},{12.3,22.1,0}};
    controller::ControllerComponent component(std::chrono::milliseconds(10),std::chrono::milliseconds(100),
        [&](const controller::ControlFeedback& feedback) {
            Check(std::this_thread::get_id()!=owner);
            if (feedback.control.velocity.linear!=0 || feedback.control.velocity.angular!=0) {
                ++moves;
                if (output_fault.exchange(false)) throw std::runtime_error("injected output failure");
            } else ++stops;
        });
    auto locate=[&](double x) {
        component.UpdateTransforms({true,{{10,{x,2.1,0}},{10,{10,20,0}}}},std::chrono::steady_clock::now());
    };
    component.Activate();component.UpdateMap(map);locate(2.1);
    auto handle=component.server().Submit({path,map});
    Until([&] { return component.server().timing().cycles>=3; }); // No integration, BT or Poll exists here.
    Check(component.server().GetStatus(handle).feedback.control.velocity.linear>0);
    controller::ControllerComponent::Execution::Result result;
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::failed && result.value==controller::ControlStatus::invalid_tf);
    Check(component.server().GetStatus(handle).feedback.control.velocity.linear==0);
    locate(2.1);handle=component.server().Submit({path,map});
    Until([&] { return component.server().GetStatus(handle).feedback.control.velocity.linear>0; });
    component.server().RequestCancel(handle);
    Check(component.server().GetStatus(handle).feedback.control.velocity.linear==0);
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::canceled);
    locate(2.1);handle=component.server().Submit({path,map});
    Until([&] { return component.server().GetStatus(handle).feedback.control.velocity.linear>0; });
    locate(2.3);
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::succeeded && result.value==controller::ControlStatus::reached);
    const auto stopped=stops.load();
    handle=component.server().Submit({{},map});
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.error && result.state==TaskState::failed && stops>stopped);
    const auto stopped_again=stops.load();
    locate(2.1);output_fault=true;handle=component.server().Submit({path,map});
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.error && result.state==TaskState::failed && stops>stopped_again);
    Check(moves>=3);
    component.Deactivate();component.ClearInputs();
}
