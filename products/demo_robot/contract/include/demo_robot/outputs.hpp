#pragma once
#include <roboframe/core/runtime.hpp>
#include <demo_robot/mapping.hpp>
#include <demo_robot/navigation.hpp>
#include <roboframe/datatype/grid.hpp>
#include <roboframe/datatype/transforms2d.hpp>
#include <functional>
#include <string>
namespace demo_robot {
// Generation distinguishes maps within one runtime; it is not a persistent asset ID.
struct MapView { std::uint64_t generation = 0; roboframe::GridSnapshot grid; };
enum class MappingEventKind { started, progress, finished, canceled, saved, loaded, stopped, finishing, failed };
struct MappingEvent {
    MappingEventKind kind;
    MappingStatus status;
    std::uint64_t generation = 0;
    std::string name;
};
// Outputs run on the command/Poll owner, except velocity also runs on the controller worker.
// Sensor telemetry may coalesce to the latest value before Poll. Must not block, throw or reenter.
// Receivers may retain immutable snapshots. Clear callbacks before destroying receivers.
struct DemoRobotOutputs {
    std::function<void(const roboframe::RuntimeStatus&)> runtime;
    std::function<void(const roboframe::TimedPose2D&)> odometry;
    std::function<void(const roboframe::MapCorrection2D&)> correction;
    std::function<void(const roboframe::TransformStatus2D&)> transforms;
    std::function<void(const MapView&)> map;
    std::function<void(const MappingEvent&)> mapping;
    std::function<void(const NavigationStatus&)> navigation;
    std::function<void(const NavigationPath&)> path;
    // Must be thread-safe and bounded; SetOutputs({}) waits for in-flight delivery.
    std::function<void(const VelocityCommand&)> velocity;
};
} // namespace demo_robot
