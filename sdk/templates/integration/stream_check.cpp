#include "stream_integration.hpp"
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
    std::atomic<bool> slow{true},entered{false},finish{false};
    integration_template::StreamIntegration product([&](const auto& input,const Cancellation& cancel) {
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
    Check(old.expired()); // The component discards its old results during Deactivate.
    product.Stop();
    product.Start();
    Check(product.Push(std::make_shared<const std::vector<unsigned char>>(5,1))==Admission::accepted);
    Until([&] { product.Poll(); return received==8; });
    product.Shutdown();

    integration_template::StreamIntegration failing([](const auto&,const Cancellation&) -> std::size_t {
        throw std::runtime_error("backend failed");
    });
    failing.Start();
    Check(failing.Push(std::make_shared<const std::vector<unsigned char>>(1,1))==Admission::accepted);
    Until([&] { failing.Poll(); return failing.runtime_status().state==RuntimeState::error; });
    Check(failing.Push(std::make_shared<const std::vector<unsigned char>>(1,1))==Admission::closed);
    failing.Reset();failing.Start();failing.Stop();
}
