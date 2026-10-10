#include <planner/read_only_planner_component.hpp>
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace planner {
roboframe::ComponentInfo GetComponentInfo() { return {component_name, source_id, roboframe::sdk_version}; }
namespace {
std::uint64_t Hash(const roboframe::Grid& map, const roboframe::Scan2D* scan) {
    std::uint64_t hash = 14695981039346656037ULL;
    auto bytes = [&](const void* data, std::size_t size) {
        const auto* p = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < size; ++i) { hash ^= p[i]; hash *= 1099511628211ULL; }
    };
    for (std::size_t i = 0; i < map.cells.block_count(); ++i)
        bytes(map.cells.BlockData(i), map.cells.BlockSize(i));
    bytes(&map.revision, sizeof(map.revision));
    if (scan) {
        bytes(&scan->stamp_us, sizeof(scan->stamp_us));
        for (const auto& point : scan->hits) { bytes(&point.x, sizeof(point.x)); bytes(&point.y, sizeof(point.y)); }
    }
    return hash;
}
}
ReadOnlyPlannerComponent::ReadOnlyPlannerComponent(std::chrono::milliseconds delay) : read_delay_(delay) {
    if (delay.count() < 0 || delay.count() > 1000) throw std::invalid_argument("planner read delay must be 0..1000 ms");
}
ReadOnlyPlannerComponent::~ReadOnlyPlannerComponent() { Deactivate(); }
void ReadOnlyPlannerComponent::Activate() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) return;
    status_ = {};
    stopping_ = false;
    worker_ = std::thread(&ReadOnlyPlannerComponent::Run, this);
    active_ = true;
}
void ReadOnlyPlannerComponent::Deactivate() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_) return;
        stopping_ = true;
        pending_ = {};
    }
    changed_.notify_all();
    worker_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
}
roboframe::Lifecycle ReadOnlyPlannerComponent::lifecycle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_ ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
}
void ReadOnlyPlannerComponent::Submit(roboframe::GridSnapshot map, roboframe::Scan2DPtr scan) {
    if (!map) throw std::invalid_argument("planner needs map");
    roboframe::ValidateGridSpec(map->spec);
    if (scan) roboframe::ValidateScan2D(*scan);
    if (!map->revision || map->stamp_us < 0 || map->cells.size() != std::size_t(map->spec.width) * map->spec.height ||
        (scan && map->stamp_us != scan->stamp_us)) throw std::invalid_argument("planner input mismatch");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || stopping_) throw std::logic_error("planner inactive");
    ++status_.received;
    if (pending_.map) ++status_.replaced;
    pending_ = {std::move(map), std::move(scan)};
    status_.peak_retained = std::max(status_.peak_retained, std::uint64_t(1 + (busy_ ? 1 : 0)));
    changed_.notify_all();
}
ReadStatus ReadOnlyPlannerComponent::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto result = status_;
    result.active_read = busy_;
    result.pending = bool(pending_.map);
    return result;
}
void ReadOnlyPlannerComponent::WaitIdle() {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait(lock, [&] { return !busy_ && !pending_.map; });
}
void ReadOnlyPlannerComponent::Run() {
    for (;;) {
        Input input;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [&] { return stopping_ || pending_.map; });
            if (stopping_) break;
            input = std::move(pending_);
            busy_ = true;
        }
        bool consistent = false;
        std::uint64_t checksum = 0;
        try {
            checksum = Hash(*input.map, input.scan.get());
            {
                std::unique_lock<std::mutex> lock(mutex_);
                changed_.wait_for(lock, read_delay_, [&] { return stopping_; });
            }
            consistent = checksum == Hash(*input.map, input.scan.get());
        } catch (...) { consistent = false; }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            ++status_.completed;
            if (!consistent) ++status_.inconsistent;
            status_.revision = input.map->revision;
            status_.checksum = checksum;
            status_.map_address = reinterpret_cast<std::uintptr_t>(input.map.get());
            status_.scan_address = reinterpret_cast<std::uintptr_t>(input.scan.get());
            status_.scan_data_address = input.scan ? reinterpret_cast<std::uintptr_t>(input.scan->hits.data()) : 0;
            input = {}; // WaitIdle means the worker has released its resource references.
            busy_ = false;
        }
        changed_.notify_all();
    }
    changed_.notify_all();
}
} // namespace planner
