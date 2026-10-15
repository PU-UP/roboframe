#include "server_product.hpp"
#include <atomic>
#include <thread>
using namespace roboframe;
void Check(bool b) { if (!b) throw std::runtime_error("server template failed"); }
template<class F> void Until(F f) {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while (!f()) { Check(std::chrono::steady_clock::now()<deadline);std::this_thread::yield(); }
}
int main() {
    using Work=integration_template::ServerProduct::Work;
    std::atomic<unsigned> cycles{0};
    integration_template::ServerProduct product({{},
        [&](const int& goal,const Work::Context&) {
            ++cycles;return Work::Step{goal ? TaskState::succeeded : TaskState::running,1,goal};
        },{}});
    product.Start();product.Begin(0);product.Poll();
    Until([&] { return cycles>=3; });
    product.Cancel();Until([&] { product.Poll();return product.settled(); });
    product.Begin(7);Until([&] { product.Poll();return product.settled(); });
    Check(product.result().value==7);
    product.Begin(0);product.Poll();
    product.Stop();product.Start();product.Begin(9);
    Until([&] { product.Poll();return product.settled(); });
    Check(product.result().value==9);
}
