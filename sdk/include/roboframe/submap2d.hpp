#pragma once
#include <roboframe/scan2d.hpp>
#include <roboframe/transforms2d.hpp>
#include <memory>
#include <vector>
namespace roboframe {
// IDs are scoped to an execution generation. T_parent_child maps child into parent.
struct PoseNode2D {
    std::uint64_t id = 0, revision = 0;
    TimedPose2D odom_from_node;
    Pose2D map_from_node;
};
struct SubmapScan2D {
    Scan2DPtr scan;
    Pose2D local_from_base;
};
// Immutable growing snapshot. Only the small scan-reference table is copied;
// scan payloads remain shared. anchor_id names its local-frame origin.
struct Submap2D {
    std::uint64_t id = 0, revision = 0, anchor_id = 0;
    std::vector<SubmapScan2D> scans;
};
using Submap2DPtr = std::shared_ptr<const Submap2D>;
inline void ValidateNode(const PoseNode2D& node) {
    if (!node.id || !node.revision) throw std::invalid_argument("invalid pose node identity");
    ValidatePose(node.odom_from_node);
    ValidatePose({node.odom_from_node.stamp_us, node.map_from_node});
}
} // namespace roboframe
