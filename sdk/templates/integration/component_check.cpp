#include "stream_component.hpp"
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <thread>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("direct component check failed"); }
template<class F> void Until(F ready) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (!ready()) {
        Check(std::chrono::steady_clock::now()<deadline);
        std::this_thread::yield();
    }
}
int main() {
    using C=integration_template::StreamComponent;
    std::atomic<bool> entered{false},canceled{false};
    C component([&](const C::Input& input,const Cancellation& cancel) {
        if (input->empty()) throw std::runtime_error("injected backend failure");
        if (input->size()==1) {
            entered=true;
            canceled=!cancel.WaitFor(std::chrono::seconds(5));
        }
        return input->size();
    });
    auto input=[](std::size_t n) {
        return std::make_shared<const std::vector<unsigned char>>(n,7);
    };
    C::Execution::Result result;
    Check(component.Push(input(3))==Admission::closed);
    component.Activate();
    Check(component.Push(input(3))==Admission::accepted);
    Until([&] { return component.Take(result); });
    Check(!result.error && !result.canceled && result.output==3);

    Check(component.Push(input(1))==Admission::accepted);
    Until([&] { return entered.load(); });
    component.Deactivate(); // Cancels the backend wait and discards the old result itself.
    Check(canceled && component.lifecycle()==Lifecycle::inactive && !component.Take(result));
    Check(component.Push(input(3))==Admission::closed);

    component.Activate();
    Check(component.Push(input(0))==Admission::accepted);
    Until([&] { return component.Take(result); });
    Check(bool(result.error));
    Check(component.Push(input(4))==Admission::accepted);
    Until([&] { return component.Take(result); });
    Check(!result.error && result.output==4);
    component.Deactivate();
}
