#pragma once
#include <functional>
#include <initializer_list>
#include <stdexcept>
#include <vector>

namespace roboframe { namespace bt {
enum class Status { idle, running, success, failure };
struct NodeStatus { Status status=Status::idle; const char* node="none"; };
// shortcut: only a memory Sequence and callback leaves; use a dedicated BT library
// when a product needs reactive branches, decorators or externally authored trees.
class Sequence {
public:
    struct Action {
        const char* name;
        std::function<Status()> tick;
        std::function<void()> halt; // Must not throw; releases this task's work only.
    };
    explicit Sequence(std::initializer_list<Action> actions) : actions_(actions) {
        if (actions_.empty()) throw std::invalid_argument("empty behavior sequence");
        for (const auto& action:actions_)
            if (!action.name || !action.tick) throw std::invalid_argument("invalid behavior action");
    }
    Sequence(const Sequence&) = delete;
    Sequence& operator=(const Sequence&) = delete;
    Status Tick() {
        if (status_==Status::success || status_==Status::failure) return status_;
        while (current_<actions_.size()) {
            status_=Status::running;
            const auto result=actions_[current_].tick();
            if (result==Status::idle) throw std::logic_error("action returned idle");
            status_=result;
            if (result!=Status::success) return status_;
            ++current_;
        }
        return status_;
    }
    void Halt() noexcept {
        if (status_==Status::running && actions_[current_].halt) actions_[current_].halt();
        current_=0; status_=Status::idle;
    }
    NodeStatus status() const {
        return {status_,status_==Status::idle || current_==actions_.size() ? "none" : actions_[current_].name};
    }
private:
    const std::vector<Action> actions_;
    std::size_t current_=0;
    Status status_=Status::idle;
};
} } // namespace roboframe::bt
