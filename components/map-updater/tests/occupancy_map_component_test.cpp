#include <map_updater/occupancy_map_component.hpp>
#include <stdexcept>
#include <algorithm>
void Check(bool ok) { if (!ok) throw std::runtime_error("grid check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
int main() {
    map_updater::OccupancyMapComponent grid({10, 10, 1, 0, 0});
    auto scan = std::make_shared<const map_updater::Scan>(map_updater::Scan{1, {{3, 0}}});
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
    auto second = std::make_shared<const map_updater::Scan>(map_updater::Scan{2, {{3, 0}}});
    Reject([&] { grid.PushScan(second, {3, {}}); });
    Reject([&] { grid.PushScan(second, {2, {-2, 0, 0}}); });
    Check(grid.revision() == 1);
    auto invalid = std::make_shared<const map_updater::Scan>(map_updater::Scan{2, {{3, 0}, {30, 0}}});
    Reject([&] { grid.PushScan(invalid, {2, {1.5, 1.5, 0}}); });
    Check(grid.revision() == 1);
    const auto outside = grid.PushScan(second, {2, {9.5, 1.5, 0}});
    Check(outside.used_rays == 0 && outside.outside_rays == 1);
    grid.Reset();
    auto crossing = std::make_shared<const map_updater::Scan>(
        map_updater::Scan{3, {{3, 0}, {5, 0}, {5, 0}, {5, 0}, {5, 0}, {5, 0}, {5, 0}}});
    grid.PushScan(crossing, {3, {1.5, 1.5, 0}});
    const auto forward = grid.Snapshot();
    Check(forward->cells[14] == 0); // An observed endpoint wins over crossing rays in the same scan.
    grid.Reset();
    auto reversed = std::make_shared<map_updater::Scan>(*crossing);
    std::reverse(reversed->hits.begin(), reversed->hits.end());
    grid.PushScan(reversed, {3, {1.5, 1.5, 0}});
    Check(grid.Snapshot()->cells == forward->cells);
    grid.Reset();
    const double half_pi = 1.5707963267948966;
    grid.PushScan(scan, {1, {1.5, 1.5, half_pi}});
    Check(grid.Snapshot()->cells[41] == 0); // +x in base becomes +y in odom.
    grid.Reset();
    Check(map->cells[14] == 0 && grid.Snapshot()->cells[14] == 205);
}
