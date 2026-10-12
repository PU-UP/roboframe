#pragma once
#include <roboframe/outputs.hpp>
#include <roboframe/scan2d.hpp>
#include <roboframe/data_flow.hpp>
#include <functional>
#include <cstddef>
namespace roboframe {
// Bind once before receiving. Captured product objects must outlive the interface.
// These callbacks are synchronous delivery, not an executor or a thread-safety wrapper.
// A transport calls commands/queries on the business owner; sensor concurrency is a product contract.
// Empty callbacks mean unsupported. Implementations may reject requests by throwing std::exception.
struct SensorInputs {
    std::function<void(const TimedPose2D&)> odometry;
    std::function<void(Scan2DPtr)> scan;
    std::function<void(const MapPose2D&)> map_reference;
};
struct MappingReport {
    MappingStatus status;
    std::size_t pending = 0;
    std::uint64_t map_revision = 0;
};
struct MappingRequests {
    std::function<void(MappingOptions)> configure;
    std::function<void(std::uint64_t)> start;
    std::function<void()> finish, cancel;
    std::function<MappingReport()> status;
};
struct MapRequests {
    std::function<void(const std::string&)> save, load, snapshot;
    std::function<MapView(bool baseline)> view;
};
struct DiagnosticRequests {
    std::function<TransformStatus2D()> transforms;
    std::function<DataFlowStatus()> resources;
    std::function<void()> check_readers_idle;
};
// Current protocol's groups, not a universal list of robot capabilities.
struct ProductInputs {
    SensorInputs sensors;
    MappingRequests mapping;
    MapRequests maps;
    DiagnosticRequests diagnostics;
};
} // namespace roboframe
