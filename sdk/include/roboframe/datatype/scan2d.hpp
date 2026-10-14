#pragma once
#include <roboframe/datatype/geometry2d.hpp>
#include <memory>
#include <vector>
#include <utility>

namespace roboframe {
struct ScanPoint2D { double x, y; };
// Finite hit endpoints in metres, in base_link at stamp_us (the odom clock).
// Rays originate at the base origin. This is not raw ranges or a deskewed scan.
// No-return beams are absent; a range cutoff must discard, never clamp to a false hit.
// Producers must not mutate any alias after publishing; consumers may retain the shared data.
// Nonzero sensor origins/per-beam timing require a richer contract before using this mapper.
struct Scan2D {
    Scan2D() = default;
    Scan2D(std::int64_t stamp, std::vector<ScanPoint2D> points)
        : stamp_us(stamp), hits(std::move(points)) {}
    Scan2D(const Scan2D&) = delete;
    Scan2D& operator=(const Scan2D&) = delete;
    Scan2D(Scan2D&&) = default;
    Scan2D& operator=(Scan2D&&) = default;
    std::int64_t stamp_us = 0;
    std::vector<ScanPoint2D> hits;
};
using Scan2DPtr = std::shared_ptr<const Scan2D>;

// Structural validity only. Consumers enforce their own resource/range limits.
inline void ValidateScan2D(const Scan2D& scan) {
    if (scan.stamp_us < 0 || scan.hits.empty())
        throw std::invalid_argument("invalid scan timestamp or empty hits");
    for (const auto& point : scan.hits)
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            throw std::invalid_argument("nonfinite scan endpoint");
}
} // namespace roboframe
