#pragma once
#include <cstdint>
namespace roboframe {
enum class RuntimeState { idle, running, error };
struct RuntimeStatus {
    RuntimeState state = RuntimeState::idle;
    std::uint64_t generation = 0, stale_results = 0;
};
}
