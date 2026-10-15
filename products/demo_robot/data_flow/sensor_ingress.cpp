#include "products/demo_robot/data_flow/sensor_ingress.hpp"
namespace demo_robot {
using namespace roboframe;
SensorIngress::SensorIngress(Options options, odom::PassthroughOdomComponent& odom,
        slam::RecordedSlamComponent& slam, posegraph::PoseGraphComponent& graph, TransformSink sink)
    : options_(options), transform_sink_(std::move(sink)), poses_({512, options.max_pose_gap_us}),
      odometry_(odom), slam_(slam), graph_(graph),
      localization_(8,[this](const ScanWork& input,const Cancellation& cancel) { return Localize(input,cancel); }) {}
void SensorIngress::Open(std::uint64_t generation) { gate_.Open(generation); }
void SensorIngress::Close() noexcept {
    gate_.Close();
    auto access=gate_.Lock();
    pending_.clear(); references_.clear(); events_={}; tag_=0; transforms_={};
}
void SensorIngress::Clear() {
    auto access=gate_.Lock();
    if (access.open()) throw std::logic_error("close before clear");
    poses_.Clear(); pending_.clear(); last_scan_=-1; submitted_=0; tag_=0;
    counts_={}; resources_={}; transforms_={}; events_={}; odometry_received_={};
    references_.clear();latest_odom_={};working_odom_={};correction_={};
    has_odom_=has_correction_=worker_has_correction_=false;last_reference_=-1;
}
void SensorIngress::PushOdometry(const TimedPose2D& pose) {
    auto access=gate_.Lock(); access.RequireOpen(); odometry_.PushOdometry(pose);
    MatchScans(access.generation());
}
void SensorIngress::OnOdometry(const TimedPose2D& pose) {
    poses_.PushOdometry(pose);latest_odom_=pose;has_odom_=true;
    odometry_received_=std::chrono::steady_clock::now();
    events_.odom=pose; events_.has_odom=true;
    RefreshTransforms();
}
void SensorIngress::RefreshTransforms() {
    const bool global=options_.frame==GridFrame::map;
    const bool valid=has_odom_ && (!global || (has_correction_ &&
        correction_.stamp_us<=latest_odom_.stamp_us &&
        latest_odom_.stamp_us-correction_.stamp_us<=options_.max_correction_age_us));
    transforms_={valid,{latest_odom_,global ? correction_ : MapCorrection2D{latest_odom_.stamp_us,{}}}};
    events_.tf=transforms_;events_.has_tf=true;
    if (transform_sink_) transform_sink_(transforms_,odometry_received_);
}
void SensorIngress::OnLocalization(const MapPose2D& pose) {
    if (pose.stamp_us!=working_odom_.stamp_us) throw std::logic_error("localization time mismatch");
    graph_.PushLocalization(pose,working_odom_);worker_has_correction_=true;
}
void SensorIngress::PushReference(const MapPose2D& pose) {
    ValidatePose({pose.stamp_us,pose.pose});
    auto access=gate_.Lock(); access.RequireOpen();
    if (options_.frame!=GridFrame::map) throw std::logic_error("reference requires map frame");
    if (pose.stamp_us<=last_reference_ || pose.stamp_us<=last_scan_)
        throw std::invalid_argument("reference must precede scan in time order");
    if (references_.size()==64) throw std::length_error("reference queue full");
    references_.push_back(pose);last_reference_=pose.stamp_us;
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
        ScanWork work{entry.scan,pose,{},entry.tag};
        for (const auto& reference:references_) {
            if (reference.stamp_us>pose.stamp_us) break;
            work.references.push_back(reference);
        }
        if (localization_.Submit(generation,std::move(work))==Admission::accepted) {
            ++submitted_;
            while (!references_.empty() && references_.front().stamp_us<=pose.stamp_us) references_.pop_front();
        } else { ++resources_.compute_rejected;Drop(entry.tag,false); }
        pending_.pop_front();
    }
}
LidarInput SensorIngress::Localize(const ScanWork& input,const Cancellation& cancel) {
    if (!cancel.WaitFor(options_.test_delay)) return {};
    working_odom_=input.odom;
    graph_.PushOdometry(input.odom);
    for (const auto& reference:input.references) slam_.PushReferencePose(reference);
    slam_.PushScan(input.scan,input.odom);
    const auto tf=graph_.Resolve(input.odom);
    LidarInput work{input.scan,input.odom,tf,{},false,input.tag};
    bool rejected=false;
    if (tf.valid) {
        if (graph_.node_count()<posegraph::PoseGraphComponent::max_nodes) {
            work.node=graph_.RegisterNode(input.odom);work.has_node=true;
        } else rejected=true;
    }
    auto access=gate_.Lock();
    if (!access.open() || access.generation()!=cancel.generation()) return work;
    resources_.slam_scans=slam_.received();resources_.correction_outputs=slam_.published();
    resources_.pose_nodes=graph_.node_count();resources_.missed_references=slam_.missed_references();
    if (!tf.valid) ++resources_.tf_dropped;
    if (rejected) ++resources_.history_rejected;
    if (worker_has_correction_) {
        const auto correction=graph_.correction();
        if (!has_correction_ || correction.stamp_us>correction_.stamp_us) {
            correction_=correction;has_correction_=true;
            events_.correction=correction;events_.has_correction=true;
        }
    }
    RefreshTransforms(); // Combine correction with newest odometry, never rewind to this scan.
    return work;
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
    return {counts_,resources_,pending_.size(),submitted_,transforms_,odometry_received_};
}
SensorIngress::State SensorIngress::status() const {
    auto access=gate_.Lock(); return StateLocked();
}
SensorIngress::Events SensorIngress::TakeEvents() {
    auto access=gate_.Lock();
    const auto result=events_; events_={}; return result;
}
} // namespace demo_robot
