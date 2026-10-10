#pragma once
#include <roboframe/grid.hpp>
#include <cstdint>

namespace roboframe {
// Optional mapping API: public data only, no product orchestration or component types.
struct MappingOptions {
    GridSpec grid;
    std::int64_t max_pose_gap_us = 250000; // Positive interpolation bracket limit; no extrapolation.
};
enum class MappingPhase { idle, mapping, ready };
struct MappingStatus {
    MappingPhase phase = MappingPhase::idle;
    std::uint64_t session = 0, received = 0, integrated = 0, ignored = 0;
    std::uint64_t dropped = 0, outside_rays = 0, used_rays = 0;
    std::uint64_t drop_sync = 0, drop_queue = 0, drop_bounds = 0, drop_cancel = 0;
};
// Counters describe the current/last session; ignored also counts pre/post-session input.
// A new session resets counters. This is task status, not component lifecycle.
} // namespace roboframe
