#pragma once
#include <roboframe/component.hpp>
#include <roboframe/submap2d.hpp>
#include <localmapbuilder/version.hpp>
namespace localmapbuilder {
roboframe::ComponentInfo GetComponentInfo();
// Short odometry-frame scan stacks; no global pose or SLAM knowledge.
// Synchronous, serialized by its execution group. Snapshots never mutate.
class LocalMapComponent final : public roboframe::Component {
public:
    explicit LocalMapComponent(std::size_t scans_per_submap = 16);
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { Reset(); lifecycle_ = roboframe::Lifecycle::inactive; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Reset();
    roboframe::Submap2DPtr PushScan(roboframe::Scan2DPtr scan,
                                  const roboframe::TimedPose2D& odom, std::uint64_t node_id, bool new_anchor = false);
private:
    const std::size_t limit_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    roboframe::Submap2DPtr current_;
    roboframe::Pose2D odom_from_local_;
    std::uint64_t next_id_ = 0;
    std::int64_t last_stamp_ = -1;
};
} // namespace localmapbuilder
