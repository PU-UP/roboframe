#pragma once
#include <roboframe/datatype/grid.hpp>
namespace demo_robot {
// Shipped product configuration, not a mandatory SDK capability set.
struct DemoRobotOptions {
    roboframe::GridSpec grid;
    std::int64_t max_pose_gap_us = 250000; // Positive interpolation bracket limit; no extrapolation.
    std::int64_t max_correction_age_us = 20000000; // map-frame mode; no future correction interpolation.
    std::int64_t control_period_ms=50;
    std::int64_t tf_timeout_ms=500;
    std::int64_t localization_test_delay_ms=0; // Test-only cooperative delay; not an IPC option.
};
}
