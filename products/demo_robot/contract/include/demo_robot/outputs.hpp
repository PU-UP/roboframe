#pragma once
#include <roboframe/core/runtime.hpp>
#include <demo_robot/mapping.hpp>
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
// Invoked on the business command/Poll owner, never on sensor producer or compute threads.
// Sensor telemetry may coalesce to the latest value before Poll. Must not block, throw or reenter.
// Receivers may retain immutable snapshots. Clear callbacks before destroying receivers.
struct DemoRobotOutputs {
    std::function<void(const roboframe::RuntimeStatus&)> runtime;
    std::function<void(const roboframe::TimedPose2D&)> odometry;
    std::function<void(const roboframe::MapCorrection2D&)> correction;
    std::function<void(const roboframe::TransformStatus2D&)> transforms;
    std::function<void(const MapView&)> map;
    std::function<void(const MappingEvent&)> mapping;
};
} // namespace demo_robot
