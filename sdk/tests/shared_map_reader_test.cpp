#include "support/shared_map_reader.hpp"
#include <stdexcept>
#include <thread>
#include <vector>
void Check(bool ok) { if (!ok) throw std::runtime_error("shared reader resource check failed"); }
int main() {
    test_support::SharedMapReader reader(std::chrono::milliseconds(50));
    reader.Activate();
    auto scan = std::make_shared<const roboframe::Scan2D>(1,
        std::vector<roboframe::ScanPoint2D>(2048, {1, 0}));
    auto map = std::make_shared<roboframe::Grid>();
    map->spec = {512, 512, 0.1, 0, 0};
    map->cells = roboframe::GridCells(512 * 512);
    map->revision = 1; map->stamp_us = 1;
    std::weak_ptr<const roboframe::Scan2D> old_scan = scan;
    std::weak_ptr<const roboframe::Grid> old_map = map;
    auto initial = map;
    reader.Submit(map, scan);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!reader.status().active_read && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    Check(reader.status().active_read);
    std::vector<std::weak_ptr<const roboframe::Grid>> old_versions;
    for (int i = 2; i <= 30; ++i) {
        auto next = std::make_shared<roboframe::Grid>(*map);
        next->cells.Set(i, 0);
        next->revision = i;
        reader.Submit(next, scan);
        old_versions.push_back(next);
        map = std::move(next);
    }
    const auto expected_map = reinterpret_cast<std::uintptr_t>(map.get());
    const auto expected_scan = reinterpret_cast<std::uintptr_t>(scan.get());
    const auto expected_data = reinterpret_cast<std::uintptr_t>(scan->hits.data());
    Check(map->cells[1] == 205 && old_map.lock()->cells[2] == 205);
    map.reset(); scan.reset(); initial.reset();
    reader.WaitIdle();
    const auto s = reader.status();
    Check(s.received == 30 && s.completed + s.replaced == 30 && s.replaced > 0);
    Check(s.peak_retained == 2 && !s.pending && !s.active_read && !s.inconsistent && s.revision == 30);
    Check(s.map_address == expected_map && s.scan_address == expected_scan && s.scan_data_address == expected_data);
    Check(old_map.expired() && old_scan.expired());
    for (const auto& weak : old_versions) Check(weak.expired());
    reader.Deactivate();
    reader.Activate();
    reader.Deactivate();
}
