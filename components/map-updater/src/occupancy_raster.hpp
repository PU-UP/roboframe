#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/datatype/transforms2d.hpp>
#include <roboframe/datatype/scan2d.hpp>
#include <roboframe/datatype/grid.hpp>
#include <map_updater/occupancy_map_component.hpp>
#include <memory>
#include <functional>
#include <stdexcept>
#include <vector>

namespace map_updater {
void ValidateScan(const roboframe::Scan2D& scan);
// These implementation limits are deliberately separate from the shared scan contract.
// Synchronous, single-owner algorithm; no internal thread or concurrent method calls.
class OccupancyRaster final : public roboframe::Component {
public:
    explicit OccupancyRaster(roboframe::GridSpec spec);
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { lifecycle_ = roboframe::Lifecycle::inactive; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Reset();
    // Once per scan per cell; occupied endpoints override free rays within that scan.
    Update PushScan(const roboframe::Scan2DPtr& scan, const roboframe::ScanTransforms2D& transforms,
                    const std::function<bool()>& canceled = {}); // After cancellation, Reset before reuse.
    roboframe::GridSnapshot Snapshot() const;
    std::uint64_t revision() const { return revision_; }
    std::uint64_t copied_bytes() const { return copied_bytes_; }
private:
    bool Cell(double x, double y, int& ix, int& iy) const;
    void Ray(int x, int y, int end_x, int end_y);
    roboframe::GridSpec spec_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    std::vector<std::int16_t> evidence_;
    std::vector<std::uint8_t> observations_;
    std::vector<std::size_t> touched_;
    roboframe::Grid current_;
    mutable roboframe::GridSnapshot published_;
    std::uint64_t copied_bytes_ = 0;
    std::uint64_t revision_ = 0;
    std::int64_t stamp_us_ = -1;
};
} // namespace map_updater
