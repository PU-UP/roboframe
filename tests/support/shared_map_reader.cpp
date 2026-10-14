#include "tests/support/shared_map_reader.hpp"
#include <algorithm>
#include <stdexcept>
#include <utility>

namespace test_support {
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
SharedMapReader::SharedMapReader(std::chrono::milliseconds delay) : read_delay_(delay) {
    if (delay.count() < 0 || delay.count() > 1000) throw std::invalid_argument("shared reader read delay must be 0..1000 ms");
}
SharedMapReader::~SharedMapReader() { Deactivate(); }
void SharedMapReader::Activate() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_) return;
    status_ = {};
    execution_ = std::make_shared<Execution>(2,
        [this](const Input& input, const roboframe::Cancellation& cancel) { return Read(input, cancel); },
        roboframe::Overload::latest_pending);
    execution_->Reset(1); active_ = true;
}
void SharedMapReader::Deactivate() {
    std::shared_ptr<Execution> execution;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_) return;
        active_ = false; execution = execution_;
    }
    execution->Stop();
    std::lock_guard<std::mutex> lock(mutex_);
    execution_.reset(); status_.pending = status_.active_read = false;
}
roboframe::Lifecycle SharedMapReader::lifecycle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_ ? roboframe::Lifecycle::active : roboframe::Lifecycle::inactive;
}
void SharedMapReader::Submit(roboframe::GridSnapshot map, roboframe::Scan2DPtr scan) {
    if (!map) throw std::invalid_argument("shared reader needs map");
    roboframe::ValidateGridSpec(map->spec);
    if (scan) roboframe::ValidateScan2D(*scan);
    if (!map->revision || map->stamp_us < 0 || map->cells.size() != std::size_t(map->spec.width) * map->spec.height ||
        (scan && map->stamp_us != scan->stamp_us)) throw std::invalid_argument("shared reader input mismatch");
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_) throw std::logic_error("shared reader inactive");
    Collect();
    auto accepted = execution_->Submit(1, {map, scan});
    // A completion may consume the last credit between Collect and Submit; collecting it is nonblocking.
    if (accepted == roboframe::Admission::full) {
        Collect(); accepted = execution_->Submit(1, {std::move(map), std::move(scan)});
    }
    if (accepted != roboframe::Admission::accepted && accepted != roboframe::Admission::replaced)
        throw std::logic_error("shared reader admission unavailable");
    ++status_.received;
    if (accepted == roboframe::Admission::replaced) ++status_.replaced;
}
void SharedMapReader::Collect() const {
    if (!execution_) return;
    Execution::Result result;
    while (execution_->Take(result)) {
        if (result.canceled) continue;
        ++status_.completed;
        if (result.error || result.output.inconsistent) ++status_.inconsistent;
        status_.revision = result.output.revision; status_.checksum = result.output.checksum;
        status_.map_address = result.output.map_address;
        status_.scan_address = result.output.scan_address; status_.scan_data_address = result.output.scan_data_address;
    }
    const auto work = execution_->status();
    status_.peak_retained = work.peak;
    status_.pending = work.queued != 0; status_.active_read = work.active != 0;
}
ReadStatus SharedMapReader::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Collect();
    return status_;
}
void SharedMapReader::WaitIdle() {
    std::shared_ptr<Execution> execution;
    { std::lock_guard<std::mutex> lock(mutex_); execution = execution_; }
    if (execution) execution->WaitIdle();
    std::lock_guard<std::mutex> lock(mutex_); Collect();
}
ReadStatus SharedMapReader::Read(const Input& input, const roboframe::Cancellation& cancel) const {
    ReadStatus result;
    result.checksum = Hash(*input.map, input.scan.get());
    cancel.WaitFor(read_delay_);
    result.inconsistent = result.checksum != Hash(*input.map, input.scan.get());
    result.revision = input.map->revision;
    result.map_address = reinterpret_cast<std::uintptr_t>(input.map.get());
    result.scan_address = reinterpret_cast<std::uintptr_t>(input.scan.get());
    result.scan_data_address = input.scan ? reinterpret_cast<std::uintptr_t>(input.scan->hits.data()) : 0;
    return result;
}
} // namespace test_support
