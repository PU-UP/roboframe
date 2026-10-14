#include "product_integration.hpp"
#include <atomic>
#include <thread>
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("integration template failed"); }
template<class F> void Until(F ready) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!ready()) { Check(std::chrono::steady_clock::now()<deadline); std::this_thread::yield(); }
}
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

    std::atomic<bool> slow{true},entered{false},finish{false};
    integration_template::ProductIntegration product([&](const auto& input,const Cancellation& cancel) {
        entered=true;
        if (slow) cancel.WaitFor(std::chrono::seconds(5));
        return input->size();
    });
    const auto owner=std::this_thread::get_id();
    std::size_t received=0;
    product.SetOutput([&](std::size_t n) { Check(std::this_thread::get_id()==owner); received+=n; });
    auto data=std::make_shared<const std::vector<unsigned char>>(1024,7);
    std::weak_ptr<const std::vector<unsigned char>> old=data;
    Check(product.Push(data)==Admission::closed);
    product.Start();
    Check(product.Push(data)==Admission::accepted);
    Until([&] { return entered.load(); });
    std::atomic<unsigned> closed{0};
    std::vector<std::thread> producers;
    for (int i=0;i<4;++i) producers.emplace_back([&] {
        while (!finish) {
            auto admission=product.Push(data);
            if (admission==Admission::closed) ++closed;
            else Check(admission==Admission::accepted || admission==Admission::full);
            std::this_thread::yield();
        }
    });
    const auto before=std::chrono::steady_clock::now();
    product.Stop();
    Check(std::chrono::steady_clock::now()-before<std::chrono::milliseconds(500));
    Until([&] { return closed.load()>0; });
    finish=true;
    for (auto& thread:producers) thread.join();
    data.reset();
    product.Reset(); slow=false; product.Start();
    Check(product.Push(std::make_shared<const std::vector<unsigned char>>(3,1))==Admission::accepted);
    Until([&] { product.Poll(); return received==3; });
    Check(product.runtime_status().stale_results>=1 && old.expired());
    product.Shutdown();
}
