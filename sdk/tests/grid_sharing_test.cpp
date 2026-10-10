#include <roboframe/grid.hpp>
#include <roboframe/scan2d.hpp>
#include <set>
#include <stdexcept>
#include <type_traits>
void Check(bool ok) { if (!ok) throw std::runtime_error("grid sharing check failed"); }
int main() {
    static_assert(!std::is_copy_constructible<roboframe::Scan2D>::value, "large scan objects must not copy");
    roboframe::Grid working;
    working.cells = roboframe::GridCells(8 * 1000 * 1000);
    auto original = std::make_shared<const roboframe::Grid>(working);
    std::vector<roboframe::GridSnapshot> versions{original};
    std::uint64_t copied = 0;
    for (std::size_t i = 0; i < 20; ++i) {
        copied += working.cells.Set(i * 4096, 0);
        Check(working.cells.Set(i * 4096, 0) == 0);
        ++working.revision;
        versions.push_back(std::make_shared<const roboframe::Grid>(working));
    }
    Check(copied == 20 * 4096);
    Check(original->cells[0] == 205 && versions.back()->cells[0] == 0);
    Check(original->cells.BlockData(100) == versions.back()->cells.BlockData(100));
    Check(original->cells.BlockData(0) != versions.back()->cells.BlockData(0));
    std::set<const void*> blocks;
    std::vector<std::weak_ptr<const roboframe::Grid>> lifetimes;
    for (const auto& map : versions) {
        lifetimes.push_back(map);
        for (std::size_t i = 0; i < map->cells.block_count(); ++i) blocks.insert(map->cells.BlockData(i));
    }
    Check(blocks.size() == working.cells.block_count() + 20); // 21 logical maps, ~one map's payload.
    original.reset();
    versions.clear();
    for (const auto& weak : lifetimes) Check(weak.expired());
    Check(working.cells[0] == 0);
    bool rejected = false;
    try { working.cells.Set(working.cells.size(), 0); } catch (const std::out_of_range&) { rejected = true; }
    Check(rejected);
    auto moved = std::move(working.cells);
    Check(working.cells.size() == 0 && working.cells.block_count() == 0);
    Check(moved.size() == 8000000 && moved[0] == 0);
    working.cells = std::move(moved);
    Check(moved.size() == 0 && moved.block_count() == 0 && working.cells[0] == 0);
}
