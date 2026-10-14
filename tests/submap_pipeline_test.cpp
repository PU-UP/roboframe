#include <odom/passthrough_odom_component.hpp>
#include <slam/recorded_slam_component.hpp>
#include <posegraph/pose_graph_component.hpp>
#include <localmapbuilder/local_map_component.hpp>
#include <map_updater/occupancy_map_component.hpp>
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("pipeline check failed"); }
template<class F> void Reject(F f) { bool yes=false; try { f(); } catch (const std::exception&) { yes=true; } Check(yes); }
int main() {
    posegraph::PoseGraphComponent graph(100);
    TimedPose2D matched;
    odom::PassthroughOdomComponent odom([&](const TimedPose2D& p){matched=p; graph.PushOdometry(p);});
    slam::RecordedSlamComponent slam([&](const MapPose2D& p){graph.PushLocalization(p,matched);});
    localmapbuilder::LocalMapComponent local(2);
    map_updater::OccupancyMapComponent map({30,20,1,-5,-5,GridFrame::map});
    graph.Activate(); odom.Activate(); slam.Activate(); local.Activate(); map.Activate(); map.Reset();
    auto scan = std::make_shared<const Scan2D>(1,std::vector<ScanPoint2D>{{2,0}});
    odom.PushOdometry({1,{10,0,0}});
    slam.PushReferencePose({1,{0,0,0}}); slam.PushScan(scan,matched);
    auto node=graph.RegisterNode(matched);
    auto submap=local.PushScan(scan,matched,node.id);
    Check(submap->scans.front().scan == scan);
    Check(map.PushSubmap(submap,graph.Node(submap->anchor_id)).used_rays==1);
    auto before=map.Snapshot();
    Check(before->cells[5*30+7]==0);
    odom.PushOdometry({2,{11,0,0}});
    auto next=std::make_shared<const Scan2D>(2,std::vector<ScanPoint2D>{{2,0}});
    slam.PushScan(next,matched); auto n2=graph.RegisterNode(matched);
    submap=local.PushScan(next,matched,n2.id);
    map.PushSubmap(submap,graph.Node(submap->anchor_id));
    Check(map.submap_count()==1 && map.retained_points()==2 && map.revision()==2);
    auto frozen=map.Snapshot();
    graph.ReviseNode(node.id,2,{1,{5,0,0}});
    map.UpdateNode(graph.Node(node.id));
    auto revised=map.Snapshot();
    Check(revised->cells[5*30+7]==205 && revised->cells[5*30+12]==0);
    Check(before->cells[5*30+7]==0 && frozen->revision==2 && revised->revision==3);
    Reject([&]{ map.UpdateNode(graph.Node(node.id)); });
    Check(map.Snapshot()==revised);
    auto bad=graph.Node(node.id); ++bad.revision; bad.map_from_node.x=100;
    Reject([&]{map.UpdateNode(bad);});
    Check(map.Snapshot()==revised);
    graph.ReviseNode(node.id,3,{1,{4,0,0}});
    Reject([&]{map.UpdateNode(graph.Node(node.id),[]{return true;});});
    Check(revised->cells[5*30+12]==0);
    map.Reset(); local.Reset(); graph.Reset();
    Check(map.submap_count()==0 && map.retained_points()==0 && map.Snapshot()->revision==0);
}
