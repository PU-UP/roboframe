#include <roboframe/orchestration/server_action.hpp>
#include <atomic>
#include <string>
#include <thread>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("server action check failed"); }
template<class F> void Until(F f) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!f()) { Check(std::chrono::steady_clock::now()<end);std::this_thread::yield(); }
}
int main() {
    using S=Server<int,int,int>;
    std::atomic<bool> fail_finish{false},hold_finish{false},finishing{false};
    S server(std::chrono::milliseconds(1),{{},
        [](const int& goal,const S::Context&) {
            return S::Step{goal ? TaskState::succeeded : TaskState::running,0,goal};
        },
        [&] {
            finishing=true;
            while (hold_finish) std::this_thread::yield();
            if (fail_finish) throw std::runtime_error("finish failed");
        }});
    server.Activate();
    const auto owner=std::this_thread::get_id();
    unsigned calls=0;
    bool on_owner=true;
    int goal=7;
    S::Result result;
    bt::ServerAction<S> action(server,[&] { return goal; },[&](const S::Result& r) {
        on_owner=on_owner && std::this_thread::get_id()==owner;
        result=r;++calls;
    });
    Check(action.Drained() && calls==0);
    Until([&] { return action.Tick()!=bt::Status::running; });
    Check(result.state==TaskState::succeeded && result.value==7 && !result.error && calls==1);
    Check(action.Drained() && action.Tick()==bt::Status::success && calls==1);

    action.Reset();goal=0;
    action.Tick();action.Halt();
    Until([&] { return action.Drained(); });
    Check(result.state==TaskState::canceled && !result.error && calls==2);
    Check(action.Drained() && action.Tick()==bt::Status::failure && calls==2);

    // A cancel request cannot deliver a result or permit reuse before finish returns.
    action.Reset();finishing=false;hold_finish=true;fail_finish=true;
    action.Tick();action.Halt();
    Until([&] { return finishing.load(); });
    const bool drained=action.Drained();
    bool rejected=false;
    try { action.Reset(); } catch (const std::logic_error&) { rejected=true; }
    const auto before=calls;
    hold_finish=false; // Release before assertions so failure cannot hang destruction.
    Check(!drained && rejected && before==2);
    Until([&] { return action.Drained(); });
    Check(result.state==TaskState::failed && bool(result.error) && calls==3);
    try { std::rethrow_exception(result.error); }
    catch (const std::runtime_error& e) { Check(std::string(e.what())=="finish failed"); }
    Check(action.Drained() && action.Tick()==bt::Status::failure && calls==3);

    // Reset also delivers a completed result, once, before allowing the next goal.
    action.Reset();fail_finish=false;
    action.Tick();action.Halt();server.WaitIdle();action.Reset();
    Check(result.state==TaskState::canceled && calls==4);
    goal=9;
    Until([&] { return action.Tick()!=bt::Status::running; });
    Check(result.state==TaskState::succeeded && result.value==9 && calls==5 && on_owner);
    action.Reset();fail_finish=true;
    Until([&] { return action.Tick()!=bt::Status::running; });
    Check(result.state==TaskState::failed && bool(result.error) && calls==6);
    Check(action.Drained() && action.Tick()==bt::Status::failure && calls==6 && on_owner);
    server.Shutdown();
}
