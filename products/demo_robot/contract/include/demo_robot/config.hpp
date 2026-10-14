#pragma once
#include <roboframe/datatype/grid.hpp>
namespace demo_robot {
// Shipped product configuration, not a mandatory SDK capability set.
struct DemoRobotOptions {
    roboframe::GridSpec grid;
    std::int64_t max_pose_gap_us = 250000; // Positive interpolation bracket limit; no extrapolation.
    std::int64_t max_correction_age_us = 20000000; // map-frame mode; no future correction interpolation.
};
}
