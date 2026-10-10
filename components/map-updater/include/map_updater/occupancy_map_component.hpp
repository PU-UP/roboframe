#pragma once
#include <roboframe/component.hpp>
#include <roboframe/geometry2d.hpp>
#include <roboframe/grid.hpp>
#include <roboframe/version.hpp>
#include <memory>
#include <vector>

namespace map_updater {
constexpr roboframe::Version contract_version{1, 0, 0};
roboframe::ComponentInfo GetComponentInfo();
struct Point { double x, y; };
// Finite hit endpoints in base coordinates, <=20m. Acquisition time shares the odom clock.
// Immutable while retained. Driver/adapter owns calibration; this algorithm never mutates a scan.
struct Scan { std::int64_t stamp_us; std::vector<Point> hits; };
using ScanPtr = std::shared_ptr<const Scan>;
void ValidateScan(const Scan& scan);
struct Update { std::size_t used_rays = 0, outside_rays = 0; };
class OccupancyMapComponent final : public roboframe::Component {
public:
    explicit OccupancyMapComponent(roboframe::GridSpec spec);
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { lifecycle_ = roboframe::Lifecycle::inactive; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Reset();
    // Once per scan per cell; occupied endpoints override free rays within that scan.
    Update PushScan(const ScanPtr& scan, const roboframe::TimedPose2D& odometry);
    roboframe::GridSnapshot Snapshot() const;
    std::uint64_t revision() const { return revision_; }
private:
    bool Cell(double x, double y, int& ix, int& iy) const;
    void Ray(int x, int y, int end_x, int end_y);
    roboframe::GridSpec spec_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    std::vector<std::int16_t> evidence_;
    std::vector<std::uint8_t> observations_;
    std::vector<std::size_t> touched_;
    std::uint64_t revision_ = 0;
    std::int64_t stamp_us_ = -1;
};
} // namespace map_updater
