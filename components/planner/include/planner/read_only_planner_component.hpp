#pragma once
#include <planner/version.hpp>
#include <roboframe/component.hpp>
#include <roboframe/grid.hpp>
#include <roboframe/scan2d.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

namespace planner {
roboframe::ComponentInfo GetComponentInfo();
struct ReadStatus {
    std::uint64_t received = 0, completed = 0, replaced = 0, peak_retained = 0, inconsistent = 0;
    std::uint64_t revision = 0, checksum = 0;
    std::uintptr_t scan_address = 0, scan_data_address = 0, map_address = 0;
    bool active_read = false, pending = false;
};
// Resource-validation implementation: reads immutable inputs; no path planning.
// Lifecycle/Submit calls are serialized by the owner. Status/WaitIdle are thread-safe.
// At most one active read + one latest pending input; Submit never waits for a read.
class ReadOnlyPlannerComponent final : public roboframe::Component {
public:
    explicit ReadOnlyPlannerComponent(std::chrono::milliseconds read_delay = std::chrono::milliseconds(0));
    ~ReadOnlyPlannerComponent() override;
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
    void Run();
    const std::chrono::milliseconds read_delay_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::thread worker_;
    Input pending_;
    ReadStatus status_;
    bool active_ = false, stopping_ = false, busy_ = false;
};
} // namespace planner
