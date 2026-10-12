#include <roboframe/execution.hpp>
#include <memory>
#include <vector>
#include <atomic>
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("execution contract failed"); }
template<class F> void Until(F done) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!done()) {
        Check(std::chrono::steady_clock::now() < deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
struct Input { std::shared_ptr<const std::vector<int>> data; int index = 0; };
struct Output { int value = 0; const void* address = nullptr; };
int main() {
    std::atomic<int> entered{0};
    ExecutionChannel<Input, Output> slow(8, [&](const Input& in, const Cancellation& cancel) {
        ++entered; cancel.WaitFor(std::chrono::seconds(10));
        return Output{in.data->at(in.index), in.data.get()};
    });
    ExecutionChannel<Input, Output> fast(2, [](const Input& in, const Cancellation&) {
        // Two small processing stages share one execution group, without per-component threads.
        const int decoded = in.data->at(in.index);
        return Output{decoded * 2, in.data.get()};
    });
    slow.Reset(1); fast.Reset(1);
    const auto data = std::make_shared<const std::vector<int>>(std::vector<int>{3, 5});
    Check(slow.Submit(1, {data, 0}) == Admission::accepted);
    Until([&] { return entered == 1; });
    Check(fast.Submit(1, {data, 1}) == Admission::accepted);
    Until([&] { return fast.status().completed == 1; });
    ExecutionChannel<Input, Output>::Result result;
    Check(fast.Take(result) && !result.error && result.output.value == 10 && result.output.address == data.get());
    Check(slow.status().active == 1 && !slow.status().completed); // Independent progress while slow is blocked.
    std::atomic<int> accepted{1}, rejected{0};
    std::vector<std::thread> producers;
    for (int t = 0; t < 4; ++t) producers.emplace_back([&] {
        for (int i = 0; i < 100; ++i) {
            const auto r = slow.Submit(1, {data, 0});
            if (r == Admission::accepted) ++accepted; else { Check(r == Admission::full); ++rejected; }
        }
    });
    for (auto& thread : producers) thread.join();
    Check(accepted == 8 && rejected == 393 && slow.status().retained == 8 && slow.status().peak == 8);
    slow.Reset(2);
    Check(slow.Submit(1, {data, 0}) == Admission::stale);
    bool reused = false;
    try { slow.Reset(1); } catch (const std::invalid_argument&) { reused = true; }
    Check(reused); // Prevent a canceled token becoming valid again after epoch reuse.
    Until([&] { return slow.status().completed == 1; });
    Check(slow.Take(result) && result.generation == 1 && result.canceled);
    Check(slow.status().retained == 0); // Cancel discarded queued payloads and completed credit was released.
    Check(fast.Submit(1, {data, 99}) == Admission::accepted);
    Until([&] { return fast.status().completed == 1; });
    Check(fast.Take(result) && result.error && !result.canceled);
    bool caught = false;
    try { std::rethrow_exception(result.error); } catch (const std::out_of_range&) { caught = true; }
    Check(caught); // Worker survives handler failure.
    Check(fast.Submit(1, {data, 0}) == Admission::accepted);
    Until([&] { return fast.status().completed == 1; });
    Check(fast.Take(result) && result.output.value == 6);
    Check(slow.Submit(2, {data, 0}) == Admission::accepted);
    Until([&] { return entered == 2; });
    const auto before = std::chrono::steady_clock::now();
    slow.Stop(); slow.Stop();
    Check(std::chrono::steady_clock::now() - before < std::chrono::milliseconds(500));
    Check(slow.Submit(2, {data, 0}) == Admission::closed && slow.status().retained == 0);
}
