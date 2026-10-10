#pragma once
#include <roboframe/component.hpp>
#include <roboframe/geometry2d.hpp>
#include <roboframe/scan2d.hpp>
#include <roboframe/grid.hpp>
#include <map_updater/version.hpp>
#include <memory>
#include <vector>

namespace map_updater {
roboframe::ComponentInfo GetComponentInfo();
// These implementation limits are deliberately separate from the shared scan contract.
void ValidateScan(const roboframe::Scan2D& scan);
struct Update { std::size_t used_rays = 0, outside_rays = 0; };
class OccupancyMapComponent final : public roboframe::Component {
public:
    explicit OccupancyMapComponent(roboframe::GridSpec spec);
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { lifecycle_ = roboframe::Lifecycle::inactive; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Reset();
    // Once per scan per cell; occupied endpoints override free rays within that scan.
    Update PushScan(const roboframe::Scan2DPtr& scan, const roboframe::TimedPose2D& odometry);
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
