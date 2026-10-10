#include <map_updater/occupancy_map_component.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace map_updater {
roboframe::ComponentInfo GetComponentInfo() {
    return {component_name, implementation_version, roboframe::sdk_version, contract_version};
}
void ValidateScan(const roboframe::Scan2D& scan) {
    roboframe::ValidateScan2D(scan);
    if (scan.hits.size() > 2048) throw std::invalid_argument("scan exceeds mapper capacity");
    for (const auto& hit : scan.hits)
        if (std::hypot(hit.x, hit.y) > 20.001 || std::hypot(hit.x, hit.y) < 0.01)
            throw std::invalid_argument("invalid scan endpoint");
}
OccupancyMapComponent::OccupancyMapComponent(roboframe::GridSpec spec) : spec_(spec) {
    roboframe::ValidateGridSpec(spec);
    evidence_.resize(std::size_t(spec.width) * spec.height);
    observations_.resize(evidence_.size());
}
void OccupancyMapComponent::Reset() {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("grid inactive");
    std::fill(evidence_.begin(), evidence_.end(), 0);
    revision_ = 0;
    stamp_us_ = -1;
}
bool OccupancyMapComponent::Cell(double x, double y, int& ix, int& iy) const {
    const double gx = (x - spec_.origin_x) / spec_.resolution;
    const double gy = (y - spec_.origin_y) / spec_.resolution;
    if (gx < 0 || gy < 0 || gx >= spec_.width || gy >= spec_.height) return false;
    ix = static_cast<int>(gx);
    iy = static_cast<int>(gy);
    return true;
}
void OccupancyMapComponent::Ray(int x, int y, int end_x, int end_y) {
    const int dx = std::abs(end_x - x), dy = -std::abs(end_y - y);
    const int sx = x < end_x ? 1 : -1, sy = y < end_y ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        const auto index = std::size_t(y) * spec_.width + x;
        auto& observation = observations_[index];
        if (!observation) touched_.push_back(index);
        const bool endpoint = x == end_x && y == end_y;
        observation = std::max<std::uint8_t>(observation, endpoint ? 2 : 1);
        if (endpoint) break;
        const int twice = 2 * error;
        if (twice >= dy) { error += dy; x += sx; }
        if (twice <= dx) { error += dx; y += sy; }
    }
}
Update OccupancyMapComponent::PushScan(const roboframe::Scan2DPtr& scan, const roboframe::TimedPose2D& pose) {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("grid inactive");
    if (!scan) throw std::invalid_argument("null scan");
    ValidateScan(*scan);
    roboframe::ValidatePose(pose);
    if (scan->stamp_us != pose.stamp_us) throw std::invalid_argument("scan/pose timestamps differ");
    if (scan->stamp_us <= stamp_us_) throw std::invalid_argument("grid scan timestamps must increase");
    int x, y;
    if (!Cell(pose.pose.x, pose.pose.y, x, y)) throw std::out_of_range("robot outside configured grid");
    Update update;
    const double c = std::cos(pose.pose.yaw), s = std::sin(pose.pose.yaw);
    for (const auto& point : scan->hits) {
        int ex, ey;
        if (!Cell(pose.pose.x + c * point.x - s * point.y,
                  pose.pose.y + s * point.x + c * point.y, ex, ey)) {
            ++update.outside_rays;
            continue; // No artificial occupied boundary or invented no-return rays.
        }
        Ray(x, y, ex, ey);
        ++update.used_rays;
    }
    for (auto index : touched_) {
        auto& value = evidence_[index];
        value = static_cast<std::int16_t>(std::max(-30, std::min(30,
            int(value) + (observations_[index] == 2 ? 4 : -1))));
        observations_[index] = 0;
    }
    touched_.clear();
    ++revision_;
    stamp_us_ = scan->stamp_us;
    return update;
}
roboframe::GridSnapshot OccupancyMapComponent::Snapshot() const {
    auto map = std::make_shared<roboframe::Grid>();
    map->spec = spec_;
    map->revision = revision_;
    map->stamp_us = stamp_us_;
    map->cells.reserve(evidence_.size());
    for (auto value : evidence_) map->cells.push_back(value >= 3 ? 0 : value < 0 ? 254 : 205);
    return map;
}
} // namespace map_updater
