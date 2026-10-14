#include <posegraph/pose_graph_component.hpp>
namespace posegraph {
using namespace roboframe;
ComponentInfo GetComponentInfo() { return {component_name, source_id, sdk_version}; }
PoseGraphComponent::PoseGraphComponent(std::int64_t max_age, bool global) : max_age_(max_age), global_(global) {
    if (max_age <= 0 || max_age > 60000000) throw std::invalid_argument("invalid correction age");
}
void PoseGraphComponent::RequireActive() const {
    if (lifecycle_ != Lifecycle::active) throw std::logic_error("posegraph inactive");
}
void PoseGraphComponent::Reset() { nodes_.clear(); has_odom_ = has_correction_ = false; }
void PoseGraphComponent::PushOdometry(const TimedPose2D& pose) {
    RequireActive(); ValidatePose(pose);
    if (has_odom_ && pose.stamp_us <= odom_.stamp_us) throw std::invalid_argument("unordered odometry");
    odom_ = pose; has_odom_ = true;
}
void PoseGraphComponent::PushLocalization(const MapPose2D& corrected, const TimedPose2D& odom) {
    RequireActive(); ValidatePose({corrected.stamp_us, corrected.pose}); ValidatePose(odom);
    if (!global_ || corrected.stamp_us != odom.stamp_us ||
        (has_correction_ && corrected.stamp_us <= correction_.stamp_us))
        throw std::invalid_argument("localization requires matching time and ordered corrections");
    correction_ = {corrected.stamp_us, Compose(corrected.pose, Inverse(odom.pose))};
    has_correction_ = true;
}
TransformStatus2D PoseGraphComponent::Resolve(const TimedPose2D& odom) const {
    RequireActive(); ValidatePose(odom);
    if (global_ && (!has_correction_ || correction_.stamp_us > odom.stamp_us ||
                   odom.stamp_us - correction_.stamp_us > max_age_)) return {};
    return {true, {odom, global_ ? correction_ : MapCorrection2D{odom.stamp_us, {}}}};
}
TransformStatus2D PoseGraphComponent::current() const { return has_odom_ ? Resolve(odom_) : TransformStatus2D{}; }
PoseNode2D PoseGraphComponent::RegisterNode(const TimedPose2D& odom) {
    const auto tf = Resolve(odom);
    if (!tf.valid) throw std::logic_error("no valid localization for node");
    const auto id = static_cast<std::uint64_t>(odom.stamp_us) + 1;
    if (nodes_.count(id)) throw std::invalid_argument("duplicate pose node");
    if (nodes_.size() == max_nodes) throw std::length_error("posegraph node capacity reached");
    PoseNode2D node{id, 1, odom, global_ ? MapFromBase(tf.transforms) : odom.pose};
    nodes_.emplace(id, node); return node;
}
PoseNode2D PoseGraphComponent::Node(std::uint64_t id) const {
    RequireActive(); return nodes_.at(id);
}
void PoseGraphComponent::ReviseNode(std::uint64_t id, std::uint64_t revision, const MapPose2D& pose) {
    RequireActive(); ValidatePose({pose.stamp_us, pose.pose});
    auto& node = nodes_.at(id);
    if (!global_ || revision <= node.revision || pose.stamp_us != node.odom_from_node.stamp_us)
        throw std::invalid_argument("invalid historical node revision");
    node.map_from_node = pose.pose; node.revision = revision;
}
} // namespace posegraph
