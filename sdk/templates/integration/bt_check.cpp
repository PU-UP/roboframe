#include "bt_integration.hpp"
#include <atomic>
#include <thread>
#include <string>
using namespace roboframe;
void Check(bool b) { if (!b) throw std::runtime_error("server template failed"); }
template<class F> void Until(F f) {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!f()) { Check(std::chrono::steady_clock::now()<deadline);std::this_thread::yield(); }
}
int main() {
    using Work=integration_template::BtIntegration::Work;
    std::atomic<unsigned> cycles{0};
    std::atomic<bool> fail_finish{false};
    integration_template::BtIntegration product({{},
        [&](const int& goal,const Work::Context&) {
            ++cycles;return Work::Step{goal ? TaskState::succeeded : TaskState::running,1,goal};
        },[&] { if (fail_finish) throw std::runtime_error("finish failed"); }});
    product.Start();product.Begin(0);product.Poll();
    Until([&] { return cycles>=3; });
    product.Cancel();Until([&] { product.Poll();return product.settled(); });
    Check(product.result().state==TaskState::canceled && !product.result().error);
    product.Begin(7);Until([&] { product.Poll();return product.settled(); });
    Check(product.result().state==TaskState::succeeded && product.result().value==7 && !product.result().error);
    product.Begin(0);product.Poll();
    product.Stop();Check(product.result().state==TaskState::canceled);
    product.Start();product.Begin(9);
    Until([&] { product.Poll();return product.settled(); });
    Check(product.result().value==9);
    fail_finish=true;
    product.Begin(0);product.Poll();product.Cancel();
    Until([&] { product.Poll();return product.settled(); });
    Check(product.result().state==TaskState::failed && bool(product.result().error));
    try { std::rethrow_exception(product.result().error); }
    catch (const std::runtime_error& e) { Check(std::string(e.what())=="finish failed"); }
    product.Begin(0);product.Poll();product.Stop();
    Check(product.result().state==TaskState::failed && bool(product.result().error));
}
