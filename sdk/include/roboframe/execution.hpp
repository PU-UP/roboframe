#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <exception>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

namespace roboframe {
enum class Admission { accepted, replaced, full, closed, stale };
enum class Overload { reject_newest, latest_pending };
namespace detail {
struct ExecutionState {
    std::mutex mutex;
    std::condition_variable changed;
    std::atomic<std::uint64_t> generation{0};
    std::atomic<bool> stopping{false};
};
}
// Valid only during the handler call. Never retain this token or its references.
class Cancellation {
public:
    bool canceled() const { return state_.stopping || state_.generation != generation_; }
    std::uint64_t generation() const { return generation_; }
    bool WaitFor(std::chrono::milliseconds duration) const {
        std::unique_lock<std::mutex> lock(state_.mutex);
        return !state_.changed.wait_for(lock, duration, [&] { return canceled(); });
    }
private:
    template<class, class> friend class ExecutionChannel;
    Cancellation(detail::ExecutionState& state, std::uint64_t generation) : state_(state), generation_(generation) {}
    detail::ExecutionState& state_;
    std::uint64_t generation_;
};

// Opt-in serial execution group, not one thread per component. A handler may wire multiple
// fast stages; independent slow groups need separate channels. No product/algorithm types.
// Submit/Take/status are concurrent-safe. Reset/Stop/destruction have one lifecycle owner.
// Capacity includes queued + executing + completed. Handlers run without the queue lock.
template<class Input, class Output>
class ExecutionChannel {
public:
    using Handler = std::function<Output(const Input&, const Cancellation&)>;
    struct Result {
        std::uint64_t generation = 0;
        Input input{};
        Output output{};
        std::exception_ptr error;
        bool canceled = false;
    };
    struct Status { std::size_t queued, active, completed, retained, peak; };
    ExecutionChannel(std::size_t capacity, Handler handler, Overload policy = Overload::reject_newest)
        : capacity_(capacity), handler_(std::move(handler)), policy_(policy) {
        if (!capacity_ || !handler_) throw std::invalid_argument("execution needs capacity and handler");
        thread_ = std::thread([this] { Run(); });
    }
    ~ExecutionChannel() { Stop(); }
    ExecutionChannel(const ExecutionChannel&) = delete;
    ExecutionChannel& operator=(const ExecutionChannel&) = delete;
    void Reset(std::uint64_t generation) {
        std::lock_guard<std::mutex> lock(state_.mutex);
        if (state_.stopping) throw std::logic_error("execution closed");
        if (generation && generation <= last_generation_) throw std::invalid_argument("generation must increase");
        if (generation) last_generation_ = generation;
        state_.generation = generation;
        retained_ -= inputs_.size(); inputs_.clear();
        state_.changed.notify_all();
    }
    Admission Submit(std::uint64_t generation, Input input) {
        std::lock_guard<std::mutex> lock(state_.mutex);
        if (state_.stopping) return Admission::closed;
        if (!generation || generation != state_.generation) return Admission::stale;
        if (policy_ == Overload::latest_pending && !inputs_.empty()) {
            inputs_.back() = {generation, std::move(input)};
            return Admission::replaced;
        }
        if (retained_ == capacity_) return Admission::full;
        inputs_.push_back({generation, std::move(input)});
        ++retained_; peak_ = std::max(peak_, retained_);
        state_.changed.notify_one();
        return Admission::accepted;
    }
    bool Take(Result& result) {
        std::lock_guard<std::mutex> lock(state_.mutex);
        if (results_.empty()) return false;
        result = std::move(results_.front()); results_.pop_front(); --retained_;
        return true;
    }
    Status status() const {
        std::lock_guard<std::mutex> lock(state_.mutex);
        return {inputs_.size(), busy_ ? 1u : 0u, results_.size(), retained_, peak_};
    }
    // For standalone clients only; receive/business event loops must not wait here.
    void WaitIdle() {
        std::unique_lock<std::mutex> lock(state_.mutex);
        state_.changed.wait(lock, [&] { return !busy_ && inputs_.empty(); });
    }
    void Stop() {
        {
            std::lock_guard<std::mutex> lock(state_.mutex);
            state_.stopping = true; state_.generation = 0;
            retained_ -= inputs_.size(); inputs_.clear();
            state_.changed.notify_all();
        }
        if (thread_.joinable()) thread_.join();
        std::lock_guard<std::mutex> lock(state_.mutex);
        results_.clear(); retained_ = 0;
    }
private:
    struct Entry { std::uint64_t generation; Input input; };
    void Run() {
        for (;;) {
            Result result;
            {
                std::unique_lock<std::mutex> lock(state_.mutex);
                state_.changed.wait(lock, [&] { return state_.stopping || !inputs_.empty(); });
                if (state_.stopping) break;
                result.generation = inputs_.front().generation;
                result.input = std::move(inputs_.front().input); inputs_.pop_front(); busy_ = true;
            }
            const Cancellation token(state_, result.generation);
            try { if (!token.canceled()) result.output = handler_(result.input, token); }
            catch (...) { result.error = std::current_exception(); }
            result.canceled = token.canceled();
            {
                std::lock_guard<std::mutex> lock(state_.mutex);
                busy_ = false;
                if (state_.stopping) { --retained_; state_.changed.notify_all(); break; }
                results_.push_back(std::move(result)); // Uses the admission credit, never waits for a consumer.
                state_.changed.notify_all();
            }
        }
    }
    const std::size_t capacity_;
    const Handler handler_;
    const Overload policy_;
    mutable detail::ExecutionState state_;
    std::deque<Entry> inputs_;
    std::deque<Result> results_;
    std::size_t retained_ = 0, peak_ = 0;
    std::uint64_t last_generation_ = 0;
    bool busy_ = false;
    std::thread thread_;
};
} // namespace roboframe
