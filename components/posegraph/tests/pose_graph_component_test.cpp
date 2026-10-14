#include <posegraph/pose_graph_component.hpp>
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("posegraph check failed"); }
template<class F> void Reject(F f) { bool yes=false; try { f(); } catch (const std::exception&) { yes=true; } Check(yes); }
int main() {
    posegraph::PoseGraphComponent graph(20);
    Reject([&] { graph.PushOdometry({0, {}}); });
    graph.Activate(); graph.PushOdometry({10, {2, 0, 0}});
    Check(!graph.current().valid);
    graph.PushLocalization({10, {5, 0, 0}}, {10, {2, 0, 0}});
    auto first = graph.RegisterNode({10, {2, 0, 0}});
    Check(first.id == 11 && first.map_from_node.x == 5);
    graph.PushOdometry({20, {3, 0, 0}});
    Check(MapFromBase(graph.current().transforms).x == 6);
    graph.ReviseNode(first.id, 2, {10, {7, 0, 0}});
    Check(graph.Node(first.id).map_from_node.x == 7);
    Check(MapFromBase(graph.current().transforms).x == 6); // Historical correction is not current localization.
    Reject([&] { graph.ReviseNode(first.id, 2, {10, {}}); });
    Reject([&] { graph.ReviseNode(first.id, 3, {11, {}}); });
    Reject([&] { graph.PushLocalization({21, {}}, {20, {}}); });
    Check(!graph.Resolve({31, {}}).valid);
    graph.Reset(); Check(!graph.current().valid && graph.node_count() == 0);
    Reject([&] { graph.Node(first.id); });
    posegraph::PoseGraphComponent local(20, false); local.Activate();
    for (std::size_t i=0; i<local.max_nodes; ++i) local.RegisterNode({static_cast<std::int64_t>(i), {}});
    Reject([&] { local.RegisterNode({40000, {}}); });
    Check(local.node_count() == local.max_nodes);
    local.Reset(); Check(local.node_count() == 0);
}
