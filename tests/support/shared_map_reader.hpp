#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/datatype/grid.hpp>
#include <roboframe/datatype/scan2d.hpp>
#include <chrono>
#include <roboframe/core/execution.hpp>
#include <memory>
#include <cstdint>
#include <mutex>

namespace test_support {
struct ReadStatus {
    std::uint64_t received = 0, completed = 0, replaced = 0, peak_retained = 0, inconsistent = 0;
    std::uint64_t revision = 0, checksum = 0;
    std::uintptr_t scan_address = 0, scan_data_address = 0, map_address = 0;
    bool active_read = false, pending = false;
};
// Resource-validation implementation: reads immutable inputs; no path planning.
// Lifecycle changes have one owner. Submit/status/WaitIdle are thread-safe.
// At most one active read + one latest pending input; Submit never waits for a read.
class SharedMapReader final : public roboframe::Component {
public:
    explicit SharedMapReader(std::chrono::milliseconds read_delay = std::chrono::milliseconds(0));
    ~SharedMapReader() override;
    void Activate() override;
    void Deactivate() override;
    roboframe::Lifecycle lifecycle() const override;
    void Submit(roboframe::GridSnapshot map, roboframe::Scan2DPtr scan = {});
    void WaitIdle();
    ReadStatus status() const;
private:
    struct Input {
        roboframe::GridSnapshot map;
        roboframe::Scan2DPtr scan;
    };
    using Execution = roboframe::ExecutionChannel<Input, ReadStatus>;
    ReadStatus Read(const Input& input, const roboframe::Cancellation& cancel) const;
    void Collect() const;
    const std::chrono::milliseconds read_delay_;
    mutable std::mutex mutex_;
    std::shared_ptr<Execution> execution_;
    mutable ReadStatus status_;
    bool active_ = false;
};
} // namespace test_support
