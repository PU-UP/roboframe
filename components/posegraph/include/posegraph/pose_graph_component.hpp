#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/datatype/submap2d.hpp>
#include <posegraph/version.hpp>
#include <map>
namespace posegraph {
roboframe::ComponentInfo GetComponentInfo();
// Recorded correction propagation and bounded node storage, NOT a graph optimizer.
// Synchronous serialized calls; the product assigns an execution context.
class PoseGraphComponent final : public roboframe::Component {
public:
    explicit PoseGraphComponent(std::int64_t max_correction_age_us, bool global = true);
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { Reset(); lifecycle_ = roboframe::Lifecycle::inactive; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Reset();
    void PushOdometry(const roboframe::TimedPose2D& pose);
    void PushLocalization(const roboframe::MapPose2D& corrected, const roboframe::TimedPose2D& odom);
    roboframe::TransformStatus2D Resolve(const roboframe::TimedPose2D& odom) const;
    roboframe::TransformStatus2D current() const;
    roboframe::MapCorrection2D correction() const { return correction_; }
    roboframe::PoseNode2D RegisterNode(const roboframe::TimedPose2D& odom);
    roboframe::PoseNode2D Node(std::uint64_t id) const;
    // An explicit historical result; does not reinterpret it as a current localization.
    void ReviseNode(std::uint64_t id, std::uint64_t revision, const roboframe::MapPose2D& pose);
    std::size_t node_count() const { return nodes_.size(); }
    static constexpr std::size_t max_nodes = 32768;
private:
    void RequireActive() const;
    const std::int64_t max_age_;
    const bool global_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    roboframe::TimedPose2D odom_;
    roboframe::MapCorrection2D correction_;
    bool has_odom_ = false, has_correction_ = false;
    std::map<std::uint64_t, roboframe::PoseNode2D> nodes_;
};
} // namespace posegraph
