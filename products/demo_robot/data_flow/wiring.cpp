#include "products/demo_robot/demo_robot_integration_impl.hpp"
#include <roboframe/core/log.hpp>
namespace demo_robot {
using namespace roboframe;
namespace {
void CheckComponent(const ComponentInfo& info) {
    CheckCompatibility(info);
    ROBO_LOG_INFO("component") << info.name << " source=" << info.source_id << " built_with_sdk=" << ToString(info.built_with_sdk);
}
}
DemoRobotIntegration::Impl::Impl(DemoRobotOptions options,std::string directory,DemoRobotOutputs& outputs,std::chrono::milliseconds delay)
    : outputs_(outputs),
      odom_([this](const TimedPose2D& p) { inputs_.OnOdometry(p); }),
      slam_([this](const MapPose2D& p) { inputs_.OnLocalization(p); }),
      graph_(options.max_correction_age_us,options.grid.frame==GridFrame::map),
      grid_(options.grid),store_(std::move(directory)),
      controller_(std::chrono::milliseconds(options.control_period_ms),std::chrono::milliseconds(options.tf_timeout_ms),
          [this](const controller::ControlFeedback& feedback) { EmitVelocity(feedback.control.velocity,feedback.pose_stamp_us); }),
      computation_(compute_capacity,[this,delay](const LidarInput& input,const Cancellation& cancel) {
          if (!cancel.WaitFor(delay)) return MapResult{};
          const auto correction=input.tf.transforms.map_from_odom.stamp_us;
          const auto pose=frame_==GridFrame::map ? MapFromBase(input.tf.transforms) : input.odom.pose;
          const bool placeable=input.has_node && grid_.Contains(pose);
          const bool new_anchor=placeable && (!had_node_ || (frame_==GridFrame::map && correction!=correction_us_));
          const auto submap=local_.PushScan(input.scan,input.odom,
              static_cast<std::uint64_t>(input.scan->stamp_us)+1,new_anchor);
          if (input.has_node && submap->anchor_id==input.node.id) anchor_=input.node;
          had_node_=placeable; correction_us_=correction;
          MapResult result;
          if (input.has_node && anchor_.id==submap->anchor_id) {
              result.outside=!grid_.Contains(pose);
              if (!result.outside) {
                  result.update=grid_.PushSubmap(submap,anchor_,[&] { return cancel.canceled(); });
                  result.map=grid_.Snapshot(); result.integrated=true;
              }
          }
          result.copied_bytes=grid_.copied_bytes();
          result.submaps=grid_.submap_count(); result.points=grid_.retained_points();
          return result;
      }),
      inputs_({options.grid.frame,options.max_pose_gap_us,options.max_correction_age_us,
          std::chrono::milliseconds(options.localization_test_delay_ms)},odom_,slam_,graph_,
          [this](TransformStatus2D tf,std::chrono::steady_clock::time_point received) { controller_.UpdateTransforms(tf,received); }),
      frame_(options.grid.frame),tf_timeout_(options.tf_timeout_ms),
      plan_action_(planner_.server(),[this] { return navigation_.input; },[this](const planner::PlannerComponent::Execution::Result& r) {
          if (r.error) navigation_.status.reason="planning_error";
          else if (r.state!=TaskState::succeeded) navigation_.status.reason="no_path";
          else navigation_.path=r.value.path;
      }),
      follow_action_(controller_.server(),[this] {
          navigation_.status.phase=NavigationPhase::following;
          if (outputs_.path) outputs_.path({navigation_.status.session,generation_,navigation_.path});
          EmitNavigation();
          return controller::FollowPathGoal{navigation_.path,navigation_.input.map};
      },[this](const controller::ControllerComponent::Execution::Result& r) {
          if (r.error) navigation_.status.reason="control_error";
          else if (r.value==controller::ControlStatus::invalid_tf) navigation_.status.reason="tf_unavailable";
          else if (r.state!=TaskState::succeeded) navigation_.status.reason="control_unavailable";
      }),
      mapping_tree_({
          {"Collect",[this] {
              return task_.status.phase==MappingPhase::finishing ? bt::Status::success : bt::Status::running;
          },[this] { HaltMapping(); }},
          {"Drain",[this] {
              const auto s=Status();
              return s.integrated+s.dropped==s.received ? bt::Status::success : bt::Status::running;
          },[this] { HaltMapping(); }},
          {"Freeze",[this] { return CompleteFinish(); },{}}
      }),
      navigation_tree_({
          {"ComputePath",[this] { return plan_action_.Tick(); },[this] { ReleaseNavigation(); }},
          {"FollowPath",[this] { return follow_action_.Tick(); },[this] { ReleaseNavigation(); }}
      }) {
    if (delay.count()<0 || delay.count()>5000) throw std::invalid_argument("invalid compute test delay");
    CheckComponent(odom::GetComponentInfo()); CheckComponent(slam::GetComponentInfo());
    CheckComponent(posegraph::GetComponentInfo()); CheckComponent(localmapbuilder::GetComponentInfo());
    CheckComponent(map_updater::GetComponentInfo()); CheckComponent(map_server::GetComponentInfo());
    CheckComponent(planner::GetComponentInfo()); CheckComponent(controller::GetComponentInfo());
    velocity_output_=outputs_.velocity;
    store_.Activate(); // Assets are accessible while runtime is idle.
}
} // namespace demo_robot
