#pragma once
#include "products/demo_robot/data_flow/pose_buffer.hpp"
#include <roboframe/datatype/grid.hpp>
#include <demo_robot/diagnostics.hpp>
#include <roboframe/core/execution.hpp>
#include <odom/passthrough_odom_component.hpp>
#include <slam/recorded_slam_component.hpp>
#include <posegraph/pose_graph_component.hpp>
#include <roboframe/core/input_gate.hpp>
namespace demo_robot {
// Product connection payload. Task tags attribute results; they never enable algorithms.
struct LidarInput {
    roboframe::Scan2DPtr scan;
    roboframe::TimedPose2D odom;
    roboframe::TransformStatus2D tf;
    roboframe::PoseNode2D node;
    bool has_node = false;
    std::uint64_t task = 0;
};
// This product's odometry/scan/reference ingress, synchronization and lightweight dispatch.
// Connections and matching policy are product-owned; shared execution/ownership is SDK-owned.
// The short lock covers recorded synchronous backends and bounded admission only.
// Heavy backends need roboframe::ExecutionChannel and explicitly associated completion results.
class SensorIngress {
public:
    using Sink = std::function<roboframe::Admission(std::uint64_t, LidarInput)>;
    struct Options {
        roboframe::GridFrame frame = roboframe::GridFrame::odom;
        std::int64_t max_pose_gap_us=250000;
    };
    struct Counts { std::uint64_t received=0, dropped=0, drop_sync=0, drop_queue=0; };
    struct State {
        Counts counts;
        DataFlowStatus resources;
        std::size_t pending=0;
        std::uint64_t submitted=0;
        roboframe::TransformStatus2D transforms;
    };
    struct Events {
        bool has_odom=false, has_correction=false, has_tf=false;
        roboframe::TimedPose2D odom;
        roboframe::MapCorrection2D correction;
        roboframe::TransformStatus2D tf;
    };
    SensorIngress(Options options, odom::PassthroughOdomComponent& odom,
                slam::RecordedSlamComponent& slam, posegraph::PoseGraphComponent& graph, Sink sink);
    void Open(std::uint64_t generation);
    void Close() noexcept;
    void Clear(); // Closed ingress and quiescent execution required.
    void PushOdometry(const roboframe::TimedPose2D& pose);
    void PushReference(const roboframe::MapPose2D& pose);
    void PushScan(roboframe::Scan2DPtr scan);
    void OnOdometry(const roboframe::TimedPose2D& pose); // Synchronous under ingress lock.
    void OnLocalization(const roboframe::MapPose2D& pose);
    void BeginTag(std::uint64_t tag);
    State EndTag(bool flush); // Task fence, never closes continuous admission.
    State status() const;
    Events TakeEvents();
private:
    struct Pending { roboframe::Scan2DPtr scan; std::uint64_t tag; };
    void MatchScans(std::uint64_t generation);
    void Drop(std::uint64_t tag, bool sync);
    State StateLocked() const;
    roboframe::InputGate gate_;
    const Options options_;
    const Sink sink_;
    PoseBuffer poses_;
    odom::PassthroughOdomComponent& odometry_;
    slam::RecordedSlamComponent& slam_;
    posegraph::PoseGraphComponent& graph_;
    std::deque<Pending> pending_;
    std::uint64_t submitted_=0, tag_=0;
    Counts counts_;
    DataFlowStatus resources_;
    roboframe::TransformStatus2D transforms_;
    std::int64_t last_scan_=-1;
    Events events_;
};
} // namespace demo_robot
