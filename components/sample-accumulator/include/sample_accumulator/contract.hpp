#pragma once
#include <cstdint>
#include <roboframe/task.hpp>
#include <roboframe/version.hpp>

namespace sample_accumulator {
constexpr roboframe::Version contract_version{1, 0, 0};
struct Goal {
    roboframe::GoalId id;
    std::uint64_t sample_count; // Count inputs received after acceptance, not a time window.
};
struct Sample { std::int64_t value; };
struct Feedback {
    roboframe::GoalId goal_id;
    std::uint64_t count;
    std::int64_t sum;
};
enum class Error { none, overflow };
struct Result {
    roboframe::GoalId goal_id;
    roboframe::TaskState state; // Terminal only.
    std::uint64_t count;
    std::int64_t sum;
    Error error;
};
} // namespace sample_accumulator
