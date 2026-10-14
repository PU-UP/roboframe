#include "products/demo_robot/data_flow/sensor_ingress.hpp"
namespace demo_robot {
using namespace roboframe;
SensorIngress::SensorIngress(Options options, odom::PassthroughOdomComponent& odom,
        slam::RecordedSlamComponent& slam, posegraph::PoseGraphComponent& graph, Sink sink)
    : options_(options), sink_(std::move(sink)), poses_({512, options.max_pose_gap_us}),
      odometry_(odom), slam_(slam), graph_(graph) {}
void SensorIngress::Open(std::uint64_t generation) { gate_.Open(generation); }
void SensorIngress::Close() noexcept {
    gate_.Close();
    auto access=gate_.Lock();
    pending_.clear(); events_={}; tag_=0; transforms_={};
}
void SensorIngress::Clear() {
    auto access=gate_.Lock();
    if (access.open()) throw std::logic_error("close before clear");
    poses_.Clear(); pending_.clear(); last_scan_=-1; submitted_=0; tag_=0;
    counts_={}; resources_={}; transforms_={}; events_={};
}
void SensorIngress::PushOdometry(const TimedPose2D& pose) {
    auto access=gate_.Lock(); access.RequireOpen(); odometry_.PushOdometry(pose);
    MatchScans(access.generation());
}
void SensorIngress::OnOdometry(const TimedPose2D& pose) {
    poses_.PushOdometry(pose); graph_.PushOdometry(pose);
    events_.odom=pose; events_.has_odom=true;
    transforms_=graph_.current(); events_.tf=transforms_; events_.has_tf=true;
}
void SensorIngress::OnLocalization(const MapPose2D& pose) {
    TimedPose2D odom;
    if (poses_.At(pose.stamp_us,odom)!=PoseMatch::ready)
        throw std::logic_error("localization has no matching odometry");
    graph_.PushLocalization(pose,odom);
    events_.correction=graph_.correction(); events_.has_correction=true;
}
void SensorIngress::PushReference(const MapPose2D& pose) {
    auto access=gate_.Lock(); access.RequireOpen();
    if (options_.frame!=GridFrame::map) throw std::logic_error("reference requires map frame");
    slam_.PushReferencePose(pose);
}
void SensorIngress::PushScan(Scan2DPtr scan) {
    if (!scan) throw std::invalid_argument("null scan");
    if (scan->hits.size()>2048) throw std::invalid_argument("scan exceeds product capacity");
    ValidateScan2D(*scan);
    for (const auto& hit : scan->hits)
        if (std::hypot(hit.x,hit.y)>20.001 || std::hypot(hit.x,hit.y)<0.01)
            throw std::invalid_argument("invalid scan endpoint");
    auto access=gate_.Lock(); access.RequireOpen();
    if (scan->stamp_us<=last_scan_) throw std::invalid_argument("scan timestamps must strictly increase");
    last_scan_=scan->stamp_us;
    if (tag_) ++counts_.received;
    if (pending_.size()==64) { Drop(pending_.front().tag,false); pending_.pop_front(); }
    pending_.push_back({std::move(scan),tag_}); MatchScans(access.generation());
}
void SensorIngress::Drop(std::uint64_t tag,bool sync) {
    if (tag && tag==tag_) {
        ++counts_.dropped;
        if (sync) ++counts_.drop_sync; else ++counts_.drop_queue;
    } else ++resources_.live_dropped;
}
void SensorIngress::MatchScans(std::uint64_t generation) {
    while (!pending_.empty()) {
        const auto entry=pending_.front();
        TimedPose2D pose;
        const auto match=poses_.At(entry.scan->stamp_us,pose);
        if (match==PoseMatch::waiting) break;
        if (match!=PoseMatch::ready) { Drop(entry.tag,true); pending_.pop_front(); continue; }
        slam_.PushScan(entry.scan,pose);
        const auto tf=graph_.Resolve(pose);
        LidarInput work{entry.scan,pose,tf,{},false,entry.tag};
        if (tf.valid) {
            if (graph_.node_count()<posegraph::PoseGraphComponent::max_nodes) {
                work.node=graph_.RegisterNode(pose); work.has_node=true;
            } else ++resources_.history_rejected;
        } else ++resources_.tf_dropped;
        // Local mapping receives data even without global TF or graph storage.
        if (sink_(generation,std::move(work))==Admission::accepted) ++submitted_;
        else { ++resources_.compute_rejected; Drop(entry.tag,false); }
        transforms_=tf; events_.tf=tf; events_.has_tf=true;
        pending_.pop_front();
    }
}
void SensorIngress::BeginTag(std::uint64_t tag) {
    auto access=gate_.Lock(); access.RequireOpen();
    if (!tag || tag_) throw std::logic_error("invalid task tag");
    tag_=tag; counts_={};
}
SensorIngress::State SensorIngress::EndTag(bool flush) {
    auto access=gate_.Lock();
    if (flush) {
        for (auto& entry : pending_) {
            if (entry.tag==tag_) { Drop(entry.tag,true); entry.tag=0; }
        }
    }
    const auto state=StateLocked(); tag_=0; return state;
}
SensorIngress::State SensorIngress::StateLocked() const {
    auto r=resources_;
    r.slam_scans=slam_.received(); r.correction_outputs=slam_.published();
    r.pose_nodes=graph_.node_count(); r.missed_references=slam_.missed_references();
    return {counts_,r,pending_.size(),submitted_,transforms_};
}
SensorIngress::State SensorIngress::status() const {
    auto access=gate_.Lock(); return StateLocked();
}
SensorIngress::Events SensorIngress::TakeEvents() {
    auto access=gate_.Lock();
    const auto result=events_; events_={}; return result;
}
} // namespace demo_robot
