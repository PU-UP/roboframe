#pragma once
#include <roboframe/core/server.hpp>
#include <roboframe/orchestration/sequence.hpp>
namespace roboframe { namespace bt {
// BT owns only an operation handle. Halt requests cancellation; Drained confirms cleanup.
// Goal preparation and business interpretation of results stay with the product.
template<class T>
class ServerAction {
public:
    using Goal=typename T::GoalType;
    using Result=typename T::Result;
    ServerAction(T& server,std::function<Goal()> goal,std::function<void(const Result&)> result)
        :server_(server),goal_(std::move(goal)),result_(std::move(result)) {}
    Status Tick() {
        if (terminal_!=Status::idle) return terminal_;
        if (!handle_) handle_=server_.Submit(goal_());
        const auto status=server_.GetStatus(handle_);
        if (ServerBusy(status.state)) return Status::running;
        Result result;
        if (!server_.TakeResult(handle_,result)) throw std::logic_error("server action result missing");
        handle_={};
        terminal_=result.state==TaskState::succeeded ? Status::success : Status::failure;
        if (result_) result_(result);
        return terminal_;
    }
    void Halt() noexcept { if (handle_) server_.RequestCancel(handle_); }
    bool Drained() {
        if (!handle_) return true;
        if (ServerBusy(server_.GetStatus(handle_).state)) return false;
        Result ignored;server_.TakeResult(handle_,ignored);handle_={};return true;
    }
    void Reset() {
        if (!Drained()) throw std::logic_error("previous action is canceling");
        terminal_=Status::idle;
    }
    ServerHandle handle() const { return handle_; }
private:
    T& server_;
    const std::function<Goal()> goal_;
    const std::function<void(const Result&)> result_;
    ServerHandle handle_;
    Status terminal_=Status::idle;
};
} }
