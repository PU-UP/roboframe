#include <controller/controller_component.hpp>
#include <algorithm>
namespace controller {
using namespace roboframe;
ComponentInfo GetComponentInfo() { return {component_name,source_id,sdk_version}; }
ControllerComponent::ControllerComponent(std::chrono::milliseconds period,std::chrono::milliseconds timeout,Output output)
    : tf_timeout_(timeout),output_(std::move(output)),execution_(period,{
        [this](const FollowPathGoal& goal) { SetPath(goal.path,goal.planned_map); },
        [this](const FollowPathGoal&,const Execution::Context& context) { return StepControl(context); },
        [this] {
            ClearPath();
            if (output_) output_({}); // Also runs on pre-start cancellation and exceptions.
        }
    }) {
    if (period.count()<=0 || timeout.count()<=0) throw std::invalid_argument("invalid control timing");
}
void ControllerComponent::UpdateTransforms(TransformStatus2D tf,Clock::time_point received) {
    std::lock_guard<std::mutex> lock(input_mutex_);tf_=tf;odometry_received_=received;
}
void ControllerComponent::UpdateMap(GridSnapshot map) {
    std::lock_guard<std::mutex> lock(input_mutex_);map_=std::move(map);
}
void ControllerComponent::ClearInputs() {
    if (lifecycle()!=Lifecycle::inactive) throw std::logic_error("deactivate before clearing control inputs");
    execution_.WaitIdle();
    std::lock_guard<std::mutex> lock(input_mutex_);tf_={};odometry_received_={};map_.reset();
}
ControllerComponent::Execution::Step ControllerComponent::StepControl(const Execution::Context& context) {
    TransformStatus2D tf;GridSnapshot map;Clock::time_point received;
    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        tf=tf_;map=map_;received=odometry_received_;
    }
    const auto result=Clock::now()-received>tf_timeout_ ? ControlResult{ControlStatus::invalid_tf,{}} : Compute(tf,map);
    const auto state=result.status==ControlStatus::tracking ? TaskState::running :
        result.status==ControlStatus::reached ? TaskState::succeeded : TaskState::failed;
    const ControlFeedback feedback{result,tf.transforms.odom_from_base.stamp_us};
    if (output_ && !context.canceled()) output_(feedback);
    return {state,feedback,result.status};
}
void ControllerComponent::SetPath(Path2DPtr path,GridSnapshot map) {
    if (!path || !map || path->frame!=GridFrame::map || map->spec.frame!=path->frame ||
        path->map_revision!=map->revision || path->poses.empty() || path->poses.size()>16384 ||
        !std::isfinite(path->robot_radius) || path->robot_radius<0 || path->robot_radius>2)
        throw std::invalid_argument("invalid path/map association");
    ValidateGridSpec(map->spec);
    if (map->cells.size()!=std::size_t(map->spec.width)*map->spec.height) throw std::invalid_argument("invalid map cells");
    for (const auto& pose:path->poses) ValidatePose({path->start_stamp_us,pose});
    path_=std::move(path);planned_map_=std::move(map);index_=0;
}
bool ControllerComponent::Clear(const Grid& map,const Pose2D& pose) const {
    const auto& s=map.spec;
    const double px=std::floor((pose.x-s.origin_x)/s.resolution),py=std::floor((pose.y-s.origin_y)/s.resolution);
    if (px<0 || py<0 || px>=s.width || py>=s.height) return false;
    const int x=static_cast<int>(px),y=static_cast<int>(py),r=static_cast<int>(std::ceil(path_->robot_radius/s.resolution))+1;
    if (x<r || y<r || x+r>=static_cast<int>(s.width) || y+r>=static_cast<int>(s.height)) return false;
    for (int dy=-r;dy<=r;++dy) for (int dx=-r;dx<=r;++dx)
        if (map.cells[(y+dy)*s.width+x+dx]!=254) return false;
    return true;
}
ControlResult ControllerComponent::Compute(const TransformStatus2D& tf,GridSnapshot current) {
    if (!path_) return {};
    if (!tf.valid || tf.transforms.odom_from_base.stamp_us<path_->start_stamp_us) return {ControlStatus::invalid_tf,{}};
    Pose2D pose;
    try { pose=MapFromBase(tf.transforms); ValidatePose({tf.transforms.odom_from_base.stamp_us,pose}); }
    catch (const std::invalid_argument&) { return {ControlStatus::invalid_tf,{}}; }
    const auto& spec=planned_map_->spec;
    if (!current || current->spec.frame!=GridFrame::map || current->spec.width!=spec.width ||
        current->spec.height!=spec.height || current->spec.resolution!=spec.resolution ||
        current->spec.origin_x!=spec.origin_x || current->spec.origin_y!=spec.origin_y ||
        current->cells.size()!=planned_map_->cells.size()) return {ControlStatus::blocked,{}};
    if (!Clear(*current,pose)) return {ControlStatus::blocked,{}};
    auto distance=[&](const Pose2D& p) { return std::hypot(p.x-pose.x,p.y-pose.y); };
    const double tolerance=std::min(0.05,spec.resolution*0.25);
    while (index_+1<path_->poses.size() && distance(path_->poses[index_])<=tolerance) ++index_;
    const auto& goal=path_->poses.back();
    const auto& target=path_->poses[index_];
    const double d=distance(target);
    // shortcut: bounded waypoint tracking without dynamic prediction; replace before physical control.
    if (d>1.0) return {ControlStatus::blocked,{}}; // Do not cut across a path after a localization jump.
    const int samples=std::max(1,static_cast<int>(std::ceil(d/(spec.resolution*0.5))));
    for (int i=1;i<=samples;++i) {
        const double t=double(i)/samples;
        if (!Clear(*current,{pose.x+t*(target.x-pose.x),pose.y+t*(target.y-pose.y),0}))
            return {ControlStatus::blocked,{}};
    }
    const bool at_goal=index_+1==path_->poses.size() && distance(goal)<=tolerance;
    const double error=at_goal ? WrapAngle(goal.yaw-pose.yaw) : WrapAngle(std::atan2(target.y-pose.y,target.x-pose.x)-pose.yaw);
    if (at_goal && std::abs(error)<=0.05) return {ControlStatus::reached,{}};
    const double angular=std::max(-1.0,std::min(1.0,2*error));
    // Rotate in place before translating; linear output is body-forward and never reverse.
    const double linear=at_goal || std::abs(error)>0.2 ? 0 : std::min(0.3,d);
    return {ControlStatus::tracking,{linear,linear>0 ? 0 : angular}};
}
} // namespace controller
