#pragma once
#include <roboframe/datatype/grid.hpp>
#include <roboframe/datatype/geometry2d.hpp>
#include <vector>
namespace roboframe {
// Immutable producer-to-consumer path, metres/radians in frame. The last pose includes goal yaw.
// map_revision refers to the planning snapshot; runtime/task identity belongs to the product.
struct Path2D {
    GridFrame frame=GridFrame::map;
    std::uint64_t map_revision=0;
    std::int64_t start_stamp_us=0;
    double robot_radius=0.2;
    std::vector<Pose2D> poses;
};
using Path2DPtr=std::shared_ptr<const Path2D>;
// Body frame (base): forward m/s and counterclockwise rad/s. Zero means stop request, not confirmation.
struct Twist2D { double linear=0, angular=0; };
} // namespace roboframe
