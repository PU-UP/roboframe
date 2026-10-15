#include <roboframe/core/integration.hpp>
#include <roboframe/core/input_gate.hpp>
#include <memory>
#include <thread>
using namespace roboframe;
using Clock=std::chrono::steady_clock;
void Check(bool ok) { if (!ok) throw std::runtime_error("runtime skeleton failed"); }
template<class F> void Until(F done) {
    const auto deadline=Clock::now()+std::chrono::seconds(2);
    while (!done()) { Check(Clock::now()<deadline); std::this_thread::yield(); }
}
// Compatibility test for integration-owned execution; new components own their own execution.
class Product final : public Integration {
public:
    using Payload=std::shared_ptr<const int>;
    Product(): slow(2,[](Payload p,const Cancellation& token) {
        token.WaitFor(std::chrono::seconds(5)); return *p;
    }),fast(2,[](int x,const Cancellation&) { if (x<0) throw std::runtime_error("backend failed"); return x+1; }) {
        UseExecution(slow); UseExecution(fast);
    }
    ~Product() { Shutdown(); }
    ExecutionChannel<Payload,int> slow;
    ExecutionChannel<int,int> fast;
    int received=0;
    bool fail_start=false,open=false;
private:
    void OnStart(std::uint64_t) override {
        open=true;
        if (fail_start) throw std::runtime_error("activation failed");
    }
    void CloseInputs() noexcept override { open=false; }
    void OnStop() noexcept override {}
    void OnReset() override { received=0; }
    void OnPoll() override {
        Dispatch(slow,2,[&](ExecutionChannel<Payload,int>::Result) { throw std::logic_error("stale escaped"); });
        Dispatch(fast,2,[&](ExecutionChannel<int,int>::Result result) { if (result.error) Fail(); else received=result.output; });
    }
};
int main() {
    InputGate gate;
    bool invalid=false;
    try { gate.Open(0); } catch (const std::logic_error&) { invalid=true; }
    Check(invalid);
    gate.Open(1);
    invalid=false;
    try { gate.Open(2); } catch (const std::logic_error&) { invalid=true; }
    Check(invalid);
    gate.Close();
    { auto access=gate.Lock(); Check(!access.open()); }

    Product p;
    p.Start(); const auto first=p.runtime_status().generation;
    auto payload=std::make_shared<const int>(7);
    std::weak_ptr<const int> retained=payload;
    Check(p.slow.Submit(first,payload)==Admission::accepted); payload.reset();
    Until([&] { return p.slow.status().active==1; });
    Check(p.fast.Submit(first,41)==Admission::accepted);
    Until([&] { p.Poll(); return p.received==42; }); // Independent channel is not blocked.
    auto before=Clock::now(); p.Stop();
    Check(Clock::now()-before<std::chrono::milliseconds(500) && !p.open);
    p.Reset(); p.Start(); p.Poll();
    Check(p.runtime_status().stale_results==1 && retained.expired());
    Check(p.slow.Submit(first,std::make_shared<const int>(0))==Admission::stale);
    Check(p.fast.Submit(p.runtime_status().generation,-1)==Admission::accepted);
    Until([&] { p.Poll(); return p.runtime_status().state==RuntimeState::error; });
    Check(!p.open);
    p.Reset(); p.fail_start=true;
    bool failed=false; try { p.Start(); } catch (const std::runtime_error&) { failed=true; }
    Check(failed && !p.open && p.runtime_status().state==RuntimeState::error);
    p.Reset(); p.fail_start=false; p.Start(); p.Shutdown();
}
