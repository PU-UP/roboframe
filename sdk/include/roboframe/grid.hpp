#pragma once
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <roboframe/grid_cells.hpp>

namespace roboframe {
enum class GridFrame { odom, map };
// Cell (0,0) is bottom-left in the explicitly named frame; cells are row-major, y increasing upwards.
// Values: occupied=0, free=254, unknown=205. No navigation inflation.
struct GridSpec {
    std::uint32_t width = 0, height = 0;
    double resolution = 0.1, origin_x = 0, origin_y = 0;
    GridFrame frame = GridFrame::odom;
};
inline void ValidateGridSpec(const GridSpec& spec) {
    if ((spec.frame != GridFrame::odom && spec.frame != GridFrame::map) || !spec.width || !spec.height || spec.width > 4096 || spec.height > 4096 ||
        std::uint64_t(spec.width) * spec.height > 8000000 ||
        !std::isfinite(spec.resolution) || spec.resolution < 0.02 || spec.resolution > 2 ||
        !std::isfinite(spec.origin_x) || !std::isfinite(spec.origin_y) ||
        std::abs(spec.origin_x) > 1e6 || std::abs(spec.origin_y) > 1e6)
        throw std::invalid_argument("invalid grid bounds or resolution");
}
struct Grid {
    GridSpec spec;
    std::uint64_t revision = 0;
    std::int64_t stamp_us = -1;
    GridCells cells;
};
using GridSnapshot = std::shared_ptr<const Grid>;
} // namespace roboframe
