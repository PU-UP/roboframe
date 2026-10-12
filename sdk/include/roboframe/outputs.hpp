#pragma once
#include <roboframe/mapping.hpp>
#include <roboframe/transforms2d.hpp>
#include <functional>
#include <string>
namespace roboframe {
// Generation distinguishes maps within one runtime; it is not a persistent asset ID.
struct MapView { std::uint64_t generation = 0; GridSnapshot grid; };
enum class MappingEventKind { started, progress, finished, canceled, saved, loaded, stopped, finishing, failed };
struct MappingEvent {
    MappingEventKind kind;
    MappingStatus status;
    std::uint64_t generation = 0;
    std::string name;
};
// Invoked on the serialized integration caller thread. Must not block, throw or reenter.
// Receivers may retain immutable snapshots. Clear callbacks before destroying receivers.
struct ProductOutputs {
    std::function<void(const TimedPose2D&)> odometry;
    std::function<void(const MapCorrection2D&)> correction;
    std::function<void(const TransformStatus2D&)> transforms;
    std::function<void(const MapView&)> map;
    std::function<void(const MappingEvent&)> mapping;
};
} // namespace roboframe
