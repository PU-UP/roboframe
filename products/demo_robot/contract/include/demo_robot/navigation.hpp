#pragma once
#include <roboframe/datatype/navigation2d.hpp>
#include <string>
namespace demo_robot {
enum class NavigationPhase { idle, planning, following, canceling, succeeded, failed, canceled };
struct NavigationGoal { std::uint64_t session=0; roboframe::Pose2D pose; }; // map frame
struct NavigationStatus {
    NavigationPhase phase=NavigationPhase::idle;
    std::uint64_t session=0,generation=0;
    std::string reason;
};
struct NavigationPath { std::uint64_t session=0,generation=0; roboframe::Path2DPtr path; };
struct VelocityCommand {
    std::uint64_t session=0,generation=0,sequence=0;
    std::int64_t pose_stamp_us=0; // Sensor clock; sequence distinguishes repeated output.
    roboframe::Twist2D velocity; // base frame; no device acknowledgement implied.
};
} // namespace demo_robot
