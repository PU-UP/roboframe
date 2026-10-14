#include <map_updater/occupancy_map_component.hpp>
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("submap map check failed"); }
template<class F> void Reject(F f) { bool yes=false; try {f();} catch(const std::exception&){yes=true;} Check(yes); }
Submap2DPtr One(std::uint64_t id) {
    auto sub=std::make_shared<Submap2D>(); sub->id=id; sub->revision=1; sub->anchor_id=id;
    sub->scans.push_back({std::make_shared<const Scan2D>(id,std::vector<ScanPoint2D>{{2,0}}),{}});
    return sub;
}
PoseNode2D Node(std::uint64_t id) { return {id,1,{static_cast<std::int64_t>(id),{}},{}}; }
int main() {
    map_updater::OccupancyMapComponent map({10,10,1,-5,-5,GridFrame::map});
    Reject([&]{map.PushSubmap(One(1),Node(1));});
    map.Activate(); map.Reset();
    map.PushSubmap(One(1),Node(1)); auto first=map.Snapshot();
    Reject([&]{map.PushSubmap(One(1),Node(1));});
    auto wrong=Node(2); wrong.odom_from_node.stamp_us=99;
    Reject([&]{map.PushSubmap(One(2),wrong);});
    Check(map.Snapshot()==first && map.retained_points()==1);
    auto node=Node(1); node.revision=2; node.map_from_node.y=2;
    const auto copied = map.copied_bytes();
    map.UpdateNode(node);
    Check(map.copied_bytes() >= copied);
    Check(map.Snapshot()->cells[5*10+7]==205 && map.Snapshot()->cells[7*10+7]==0);
    Check(first->cells[5*10+7]==0);
    map.Reset();
    for(std::uint64_t i=1; i<=map.max_submaps; ++i) map.PushSubmap(One(i),Node(i));
    auto full=map.Snapshot();
    Reject([&]{map.PushSubmap(One(map.max_submaps+1),Node(map.max_submaps+1));});
    Check(map.submap_count()==map.max_submaps && map.Snapshot()==full);
    map.Reset(); Check(map.retained_points()==0);
}
