#pragma once
#include <roboframe/core/execution.hpp>
#include <roboframe/core/runtime.hpp>
#include <roboframe/core/component.hpp>
#include <vector>
namespace roboframe {
// Lifecycle, Poll and status have one control owner. Sensor ingress is a product-owned
// typed boundary. Derived destructors call Shutdown before destroying bound channels.
class Integration {
public:
    Integration() = default;
    Integration(const Integration&) = delete;
    Integration& operator=(const Integration&) = delete;
    virtual ~Integration() = default;
    RuntimeStatus runtime_status() const { return status_; }
    void Start() {
        if (closed_ || status_.state != RuntimeState::idle) throw std::logic_error("runtime is not idle");
        ValidateStart();
        try {
            ++status_.generation;
            for (auto& execution : executions_) execution.reset(status_.generation);
            for (auto& server : servers_) server.activate();
            OnStart(status_.generation);
            status_.state = RuntimeState::running;
        } catch (...) {
            Fail(); throw;
        }
        OnStateChanged();
    }
    void Stop() {
        if (status_.state == RuntimeState::idle) return;
        Quiesce(); status_.state = RuntimeState::idle; OnStateChanged();
    }
    void Reset() {
        if (closed_) throw std::logic_error("runtime closed");
        Stop();
        try { OnReset(); }
        catch (...) { status_.state = RuntimeState::error; OnStateChanged(); throw; }
    }
    void Poll() { if (!closed_) OnPoll(); }
    void Shutdown() {
        if (closed_) return;
        Stop();
        for (auto& execution : executions_) execution.stop();
        for (auto& server : servers_) server.shutdown();
        closed_ = true;
    }
protected:
    // Bind once, after configuration. Channels remain owned by the product.
    template<class Input, class Output>
    void UseExecution(ExecutionChannel<Input, Output>& channel) {
        if (closed_ || status_.generation) throw std::logic_error("bind execution before starting");
        executions_.push_back({[&channel](std::uint64_t g) { channel.Reset(g); },
                               [&channel] { channel.WaitIdle(); },
                               [&channel] { channel.Stop(); }});
    }
    // Optional capability: the component owns its executor and algorithm lifecycle.
    template<class T>
    void UseServer(T& server) {
        if (closed_ || status_.generation) throw std::logic_error("bind server before starting");
        servers_.push_back({[&server] { server.Activate(); },
            [&server] { server.server().RequestDeactivate(); },[&server] { server.Deactivate(); },
            [&server] { server.server().Shutdown(); }});
    }
    template<class Input, class Output, class Handler>
    void Dispatch(ExecutionChannel<Input, Output>& channel, std::size_t budget, Handler handler) {
        typename ExecutionChannel<Input, Output>::Result result;
        for (std::size_t i=0; i<budget && channel.Take(result); ++i) {
            if (status_.state != RuntimeState::running || result.generation != status_.generation) {
                ++status_.stale_results; continue;
            }
            handler(std::move(result));
        }
    }
    void Fail() {
        Quiesce();
        status_.state = RuntimeState::error;
        OnStateChanged();
    }
    void RequireRunning() const {
        if (status_.state != RuntimeState::running) throw std::logic_error("runtime not running");
    }
    virtual void ValidateStart() const {}
    virtual void OnStateChanged() {}
    virtual void OnStart(std::uint64_t generation) = 0;
    virtual void CloseInputs() noexcept = 0;
    virtual void OnStop() noexcept = 0;
    virtual void OnReset() = 0;
    virtual void OnPoll() = 0;
private:
    void Quiesce() {
        CloseInputs(); // Close concurrent ingress before invalidating computation.
        for (auto& server : servers_) server.cancel();
        for (auto& execution : executions_) execution.reset(0);
        for (auto& execution : executions_) execution.wait();
        for (auto& server : servers_) server.wait();
        OnStop(); // No handler is still mutating components.
    }
    struct Execution {
        std::function<void(std::uint64_t)> reset;
        std::function<void()> wait, stop;
    };
    struct ServerLifecycle { std::function<void()> activate,cancel,wait,shutdown; };
    std::vector<ServerLifecycle> servers_;
    std::vector<Execution> executions_;
    RuntimeStatus status_;
    bool closed_ = false;
};
} // namespace roboframe
