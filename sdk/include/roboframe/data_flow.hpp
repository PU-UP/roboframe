#pragma once
#include <cstdint>
namespace roboframe {
// Product resource diagnostics; values are counts/bytes, never ownership-bearing pointers.
struct DataFlowStatus {
    std::uint64_t compute_pending = 0, compute_active = 0, compute_results = 0, compute_retained = 0;
    std::uint64_t compute_peak_retained = 0, compute_rejected = 0, stale_results = 0;
    std::uint64_t slam_scans = 0, correction_outputs = 0, missed_references = 0, tf_dropped = 0;
    std::uint64_t submaps = 0, retained_points = 0, pose_nodes = 0, history_rejected = 0;
    std::uint64_t shared_scans = 0, live_updates = 0, live_dropped = 0, publications = 0;
    std::uint64_t copied_bytes = 0, map_bytes = 0, table_bytes = 0;
};
} // namespace roboframe
