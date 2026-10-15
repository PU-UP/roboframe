#include <roboframe/core/server.hpp>
#include <atomic>
#include <stdexcept>
#include <thread>
#include <vector>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("server check failed"); }
template<class F> void Until(F f) {
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (!f()) { Check(std::chrono::steady_clock::now()<end);std::this_thread::yield(); }
}
int main() {
    using S=Server<int,int,int>;
    std::atomic<bool> entered{false},release{false};
    std::thread::id hook_owner;
    std::atomic<int> finished{0};
    S server(std::chrono::milliseconds(10),{
        [&](const int&) { hook_owner=std::this_thread::get_id(); },
        [&](const int& goal,const S::Context& context) {
            Check(std::this_thread::get_id()==hook_owner);
            entered=true;
            if (goal==1) while (!release) std::this_thread::yield();
            if (goal==3) { context.WaitFor(std::chrono::seconds(10));return S::Step{TaskState::running,9,0}; }
            if (goal==4) throw std::runtime_error("algorithm fault");
            return S::Step{goal==2 ? TaskState::succeeded : TaskState::running,7,goal};
        },
        [&] { Check(std::this_thread::get_id()==hook_owner);++finished; }
    });
    server.Activate();auto first=server.Submit(1);Until([&] { return entered.load(); });
    bool busy=false;try { server.Submit(2); } catch(const std::logic_error&) { busy=true; }Check(busy);
    Check(server.RequestCancel(first));Check(server.GetStatus(first).state==TaskState::canceling);
    Check(server.GetStatus(first).feedback==0);release=true;
    Until([&] { return server.GetStatus(first).state==TaskState::canceled; });
    Check(server.GetStatus(first).feedback==0 && finished==1); // Late nonzero step suppressed.
    S::Result result;Check(server.TakeResult(first,result) && result.state==TaskState::canceled);
    auto second=server.Submit(2);Until([&] { return server.TakeResult(second,result); });Check(result.value==2);
    Check(!server.RequestCancel(first)); // Old operation cannot cancel the new one.
    entered=false;auto third=server.Submit(3);Until([&] { return entered.load(); });
    const auto before=S::Clock::now();server.Deactivate();
    Check(S::Clock::now()-before<std::chrono::seconds(1));
    Check(server.TakeResult(third,result) && result.state==TaskState::canceled);
    server.Activate();auto fourth=server.Submit(4);
    Until([&] { return server.TakeResult(fourth,result); });
    Check(result.state==TaskState::failed && bool(result.error) && fourth.epoch!=first.epoch);
    server.Shutdown();

    // Independent periodic execution, no caller Poll; no burst after a long step.
    std::vector<S::Clock::time_point> starts;
    S periodic(std::chrono::milliseconds(20),{{},
        [&](const int&,const S::Context&) {
            starts.push_back(S::Clock::now());
            if (starts.size()==1) std::this_thread::sleep_for(std::chrono::milliseconds(65));
            return S::Step{starts.size()==4 ? TaskState::succeeded : TaskState::running,1,0};
        },{}});
    periodic.Activate();auto handle=periodic.Submit(0);
    Until([&] { return periodic.TakeResult(handle,result); });
    Check(starts.size()==4 && periodic.timing().overruns==1);
    Check(starts[2]-starts[1]>std::chrono::milliseconds(5));
    periodic.Shutdown();
}
