#include "goal_component.hpp"
#include <atomic>
#include <thread>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("goal component check failed"); }
template<class F> void Until(F ready) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!ready()) { Check(std::chrono::steady_clock::now()<deadline);std::this_thread::yield(); }
}
int main() {
    using C=component_template::GoalComponent;
    using Work=C::Execution;
    std::atomic<unsigned> cycles{0},finished{0};
    C component({{},
        [&](const int& goal,const Work::Context&) {
            ++cycles;
            if (goal<0) throw std::runtime_error("injected backend failure");
            return Work::Step{goal ? TaskState::succeeded : TaskState::running,1,goal};
        },[&] { ++finished; }});
    Check(component.lifecycle()==Lifecycle::inactive);
    component.Activate();
    auto handle=component.server().Submit(0);
    Until([&] { return cycles>=3; });
    component.RequestDeactivate();
    component.Deactivate();
    Work::Result result;
    Check(component.server().TakeResult(handle,result));
    Check(result.state==TaskState::canceled && finished==1);
    Check(component.lifecycle()==Lifecycle::inactive);

    component.Activate();
    handle=component.server().Submit(-1);
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::failed && result.error && finished==2);
    handle=component.server().Submit(7);
    Until([&] { return component.server().TakeResult(handle,result); });
    Check(result.state==TaskState::succeeded && result.value==7 && finished==3);
    component.Deactivate();
}
