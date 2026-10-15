#pragma once
#include <stdexcept>
#include <cstdint>
#include <roboframe/core/task.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace roboframe {
struct ServerHandle {
    std::uint64_t epoch=0, id=0;
    explicit operator bool() const { return epoch && id; }
};
inline bool operator==(ServerHandle a,ServerHandle b) { return a.epoch==b.epoch && a.id==b.id; }
inline bool operator!=(ServerHandle a,ServerHandle b) { return !(a==b); }
inline bool ServerBusy(TaskState s) { return s==TaskState::running || s==TaskState::canceling; }

// Optional component-owned execution, with no algorithm or product policy.
// One admitted goal and one retained result. No implicit replacement or goal queue.
// Lifecycle has one owner; goal/status/cancel/result access is concurrent-safe.
// All algorithm hooks run serially on this server's worker, never under its state lock.
template<class Goal,class Feedback,class Value>
class Server {
public:
    using GoalType=Goal;
    using Clock=std::chrono::steady_clock;
    struct Step {
        TaskState state=TaskState::running;
        Feedback feedback{};
        Value value{};
    };
    struct Snapshot {
        ServerHandle handle;
        TaskState state=TaskState::idle;
        Feedback feedback{};
        std::uint64_t sequence=0;
        Clock::time_point updated{};
    };
    struct Result {
        ServerHandle handle;
        TaskState state=TaskState::idle;
        Value value{};
        std::exception_ptr error;
    };
    struct Timing {
        std::uint64_t cycles=0, overruns=0;
        Clock::duration last_duration{};
    };
    class Context {
    public:
        bool canceled() const { return server_.cancel_.load(); }
        bool WaitFor(std::chrono::milliseconds duration) const {
            std::unique_lock<std::mutex> lock(server_.mutex_);
            return !server_.changed_.wait_for(lock,duration,[&] { return canceled(); });
        }
    private:
        friend class Server;
        explicit Context(Server& server):server_(server) {}
        Server& server_;
    };
    struct Hooks {
        std::function<void(const Goal&)> start;
        std::function<Step(const Goal&,const Context&)> step;
        // Called for every admitted goal, even if canceled before start; must tolerate that.
        std::function<void()> finish;
    };
    // Zero period means a single call; positive period repeats RUNNING steps.
    Server(std::chrono::milliseconds period,Hooks hooks):period_(period),hooks_(std::move(hooks)) {
        if (period.count()<0 || !hooks_.step) throw std::invalid_argument("invalid server execution");
        worker_=std::thread([this] { Run(); });
    }
    // Owners must Shutdown before destroying members captured by hooks.
    ~Server() { Shutdown(); }
    Server(const Server&)=delete;
    Server& operator=(const Server&)=delete;
    void Activate() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (closed_ || active_ || ServerBusy(snapshot_.state)) throw std::logic_error("server not inactive");
        ++epoch_; active_=true; result_pending_=false; result_={}; snapshot_={}; goal_={};
    }
    void RequestDeactivate() noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        active_=false;
        CancelLocked();
    }
    void WaitIdle() {
        std::unique_lock<std::mutex> lock(mutex_);
        changed_.wait(lock,[&] { return !ServerBusy(snapshot_.state); });
    }
    void Deactivate() { RequestDeactivate(); WaitIdle(); }
    bool active() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return active_;
    }
    ServerHandle Submit(Goal goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || closed_) throw std::logic_error("server inactive");
        if (ServerBusy(snapshot_.state) || result_pending_) throw std::logic_error("server busy or result unread");
        goal_=std::move(goal); cancel_=false; pending_=true;
        snapshot_={}; snapshot_.handle={epoch_,++next_id_}; snapshot_.state=TaskState::running;
        changed_.notify_all(); return snapshot_.handle;
    }
    bool RequestCancel(ServerHandle handle) noexcept {
        std::lock_guard<std::mutex> lock(mutex_);
        if (handle!=snapshot_.handle || !ServerBusy(snapshot_.state)) return false;
        CancelLocked(); return true;
    }
    Snapshot GetStatus(ServerHandle handle) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!handle || handle!=snapshot_.handle) throw std::logic_error("unknown server handle");
        return snapshot_;
    }
    bool TakeResult(ServerHandle handle,Result& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (handle!=snapshot_.handle || !result_pending_) return false;
        result=std::move(result_); result_={}; result_pending_=false; return true;
    }
    Timing timing() const { std::lock_guard<std::mutex> lock(mutex_); return timing_; }
    void Shutdown() noexcept {
        RequestDeactivate(); WaitIdle();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_=true; changed_.notify_all();
        }
        if (worker_.joinable()) worker_.join();
    }
private:
    void CancelLocked() {
        if (!ServerBusy(snapshot_.state)) return;
        cancel_=true; snapshot_.state=TaskState::canceling;
        snapshot_.feedback={}; snapshot_.updated=Clock::now(); ++snapshot_.sequence;
        changed_.notify_all();
    }
    void Run() {
        for (;;) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                changed_.wait(lock,[&] { return pending_ || closed_; });
                if (closed_) return;
                pending_=false;
            }
            TaskState terminal=TaskState::canceled;
            Value value{};
            std::exception_ptr error;
            const Context context(*this);
            try {
                if (!context.canceled()) {
                    if (hooks_.start) hooks_.start(goal_);
                    auto next=Clock::now();
                    while (!context.canceled()) {
                        const auto before=Clock::now();
                        auto step=hooks_.step(goal_,context);
                        const auto after=Clock::now();
                        std::unique_lock<std::mutex> lock(mutex_);
                        ++timing_.cycles; timing_.last_duration=after-before;
                        if (period_.count() && after-before>period_) ++timing_.overruns;
                        if (cancel_) break;
                        if (step.state!=TaskState::running && step.state!=TaskState::succeeded &&
                            step.state!=TaskState::failed) throw std::logic_error("invalid server step state");
                        snapshot_.feedback=std::move(step.feedback);
                        snapshot_.updated=after; ++snapshot_.sequence;
                        if (step.state!=TaskState::running) {
                            terminal=step.state; value=std::move(step.value); break;
                        }
                        if (!period_.count()) throw std::logic_error("one-shot server returned running");
                        next+=period_;
                        if (next<=after) {
                            const auto missed=(after-next)/period_+1;
                            next+=period_*missed;
                        }
                        changed_.wait_until(lock,next,[&] { return cancel_.load(); });
                    }
                }
            } catch (...) { terminal=TaskState::failed; error=std::current_exception(); }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                snapshot_.feedback={};snapshot_.updated=Clock::now();++snapshot_.sequence;
            }
            try { if (hooks_.finish) hooks_.finish(); }
            catch (...) { terminal=TaskState::failed; error=std::current_exception(); }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (cancel_ && !error) terminal=TaskState::canceled;
                result_={snapshot_.handle,terminal,std::move(value),error};
                snapshot_.state=terminal; snapshot_.feedback={};
                snapshot_.updated=Clock::now(); ++snapshot_.sequence;
                result_pending_=true; goal_={}; changed_.notify_all();
            }
        }
    }
    const std::chrono::milliseconds period_;
    const Hooks hooks_;
    mutable std::mutex mutex_;
    mutable std::condition_variable changed_;
    std::atomic<bool> cancel_{false};
    std::thread worker_;
    Goal goal_{};
    Snapshot snapshot_;
    Result result_;
    Timing timing_;
    std::uint64_t epoch_=0,next_id_=0;
    bool active_=false,closed_=false,pending_=false,result_pending_=false;
};
} // namespace roboframe
