#include <occupancy_grid/component.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace occupancy_grid {
static_assert(roboframe::Compatible(roboframe::sdk_version, {1, 1, 0}), "OccupancyGrid requires SDK >=1.1, same major");
roboframe::ComponentInfo GetComponentInfo() {
    return {"occupancy-grid", ROBOFRAME_COMPONENT_VERSION, roboframe::sdk_version, contract_version};
}
void ValidateScan(const Scan& scan) {
    if (scan.stamp_us < 0 || scan.hits.empty() || scan.hits.size() > 2048)
        throw std::invalid_argument("invalid scan time or hit count");
    for (const auto& hit : scan.hits)
        if (!std::isfinite(hit.x) || !std::isfinite(hit.y) ||
            std::hypot(hit.x, hit.y) > 20.001 || std::hypot(hit.x, hit.y) < 0.01)
            throw std::invalid_argument("invalid scan endpoint");
}
Component::Component(roboframe::GridSpec spec) : spec_(spec) {
    roboframe::ValidateGridSpec(spec);
    evidence_.resize(std::size_t(spec.width) * spec.height);
}
void Component::Reset() {
    if (lifecycle_ != roboframe::Lifecycle::active) throw std::logic_error("grid inactive");
    std::fill(evidence_.begin(), evidence_.end(), 0);
    revision_ = 0;
    stamp_us_ = -1;
}
bool Component::Cell(double x, double y, int& ix, int& iy) const {
    const double gx = (x - spec_.origin_x) / spec_.resolution;
    const double gy = (y - spec_.origin_y) / spec_.resolution;
    if (gx < 0 || gy < 0 || gx >= spec_.width || gy >= spec_.height) return false;
    ix = static_cast<int>(gx);
    iy = static_cast<int>(gy);
    return true;
}
void Component::Ray(int x, int y, int end_x, int end_y) {
    const int dx = std::abs(end_x - x), dy = -std::abs(end_y - y);
    const int sx = x < end_x ? 1 : -1, sy = y < end_y ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        auto& value = evidence_[std::size_t(y) * spec_.width + x];
        const bool endpoint = x == end_x && y == end_y;
        value = static_cast<std::int16_t>(std::max(-30, std::min(30, int(value) + (endpoint ? 4 : -1))));
        if (endpoint) break;
        const int twice = 2 * error;
        if (twice >= dy) { error += dy; x += sx; }
        if (twice <= dx) { error += dx; y += sy; }
    }
}
Update Component::PushScan(const ScanPtr& scan, const roboframe::TimedPose2D& pose) {
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
    ++revision_;
    stamp_us_ = scan->stamp_us;
    return update;
}
roboframe::GridSnapshot Component::Snapshot() const {
    auto map = std::make_shared<roboframe::Grid>();
    map->spec = spec_;
    map->revision = revision_;
    map->stamp_us = stamp_us_;
    map->cells.reserve(evidence_.size());
    for (auto value : evidence_) map->cells.push_back(value >= 3 ? 0 : value < 0 ? 254 : 205);
    return map;
}
} // namespace occupancy_grid
