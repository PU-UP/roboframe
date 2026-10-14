#pragma once
#include <cstdint>

namespace demo_robot {
// Demo mapping task status; no component types or workflow implementation.
enum class MappingPhase { idle, mapping, ready, finishing };
struct MappingStatus {
    MappingPhase phase = MappingPhase::idle;
    std::uint64_t session = 0, received = 0, integrated = 0, ignored = 0;
    std::uint64_t dropped = 0, outside_rays = 0, used_rays = 0;
    std::uint64_t drop_sync = 0, drop_queue = 0, drop_bounds = 0, drop_cancel = 0;
};
// Counters describe only the current/last task. ignored is reserved (zero);
// inactive runtime input is rejected, while taskless RUNNING input feeds continuous capabilities.
// A new session resets counters. This is task status, not component lifecycle.
} // namespace demo_robot
