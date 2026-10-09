#include <occupancy_grid/component.hpp>
#include <stdexcept>
void Check(bool ok) { if (!ok) throw std::runtime_error("grid check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
int main() {
    occupancy_grid::Component grid({10, 10, 1, 0, 0});
    auto scan = std::make_shared<const occupancy_grid::Scan>(occupancy_grid::Scan{1, {{3, 0}}});
    Reject([&] { grid.PushScan(scan, {1, {1.5, 1.5, 0}}); });
    grid.Activate();
    grid.Reset();
    const auto empty = grid.Snapshot();
    Check(grid.PushScan(scan, {1, {1.5, 1.5, 0}}).used_rays == 1);
    const auto map = grid.Snapshot();
    Check(map->cells[14] == 0 && map->cells[11] == 254 && map->cells[12] == 254);
    Check(empty->cells[14] == 205 && empty->revision == 0);
    Check(scan->hits[0].x == 3);
    Reject([&] { grid.PushScan(scan, {1, {}}); });
    auto second = std::make_shared<const occupancy_grid::Scan>(occupancy_grid::Scan{2, {{3, 0}}});
    Reject([&] { grid.PushScan(second, {3, {}}); });
    Reject([&] { grid.PushScan(second, {2, {-2, 0, 0}}); });
    Check(grid.revision() == 1);
    auto invalid = std::make_shared<const occupancy_grid::Scan>(occupancy_grid::Scan{2, {{3, 0}, {30, 0}}});
    Reject([&] { grid.PushScan(invalid, {2, {1.5, 1.5, 0}}); });
    Check(grid.revision() == 1);
    const auto outside = grid.PushScan(second, {2, {9.5, 1.5, 0}});
    Check(outside.used_rays == 0 && outside.outside_rays == 1);
    grid.Reset();
    Check(map->cells[14] == 0 && grid.Snapshot()->cells[14] == 205);
}
