#pragma once
#include <cstdint>
#include <stdexcept>

namespace roboframe {
using GoalId = std::uint64_t;
enum class TaskState { idle, running, canceling, succeeded, failed, canceled };
struct TaskStatus {
    GoalId id = 0;
    TaskState state = TaskState::idle;
};

// Optional helper for task components. No executor, queue, payload or transport.
class Task {
public:
    TaskStatus status() const { return status_; }
    bool busy() const {
        return status_.state == TaskState::running || status_.state == TaskState::canceling;
    }
    void Begin(GoalId id) {
        if (busy()) throw std::logic_error("component busy");
        if (id == 0 || id <= status_.id) throw std::invalid_argument("goal id must increase");
        status_ = {id, TaskState::running};
    }
    void RequestCancel(GoalId id) {
        if (!busy()) throw std::logic_error("no active goal");
        if (id != status_.id) throw std::invalid_argument("wrong goal id");
        status_.state = TaskState::canceling;
    }
    void Finish(TaskState terminal) {
        const bool valid =
            (status_.state == TaskState::running &&
             (terminal == TaskState::succeeded || terminal == TaskState::failed)) ||
            (status_.state == TaskState::canceling &&
             (terminal == TaskState::canceled || terminal == TaskState::failed));
        if (!valid) throw std::logic_error("invalid task completion");
        status_.state = terminal;
    }
private:
    TaskStatus status_;
};
} // namespace roboframe
