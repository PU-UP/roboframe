#pragma once
#include <demo_robot/outputs.hpp>
#include <roboframe/core/runtime.hpp>
#include <demo_robot/config.hpp>
#include <roboframe/datatype/scan2d.hpp>
#include <demo_robot/diagnostics.hpp>
#include <functional>
#include <cstddef>
namespace demo_robot {
// Bind once before receiving. Captured product objects must outlive the interface.
// These callbacks are synchronous delivery, not an executor or a thread-safety wrapper.
// A transport calls commands/queries on the business owner; sensor concurrency is a product contract.
// Empty callbacks mean unsupported. Implementations may reject requests by throwing std::exception.
struct SensorInputs {
    std::function<void(const roboframe::TimedPose2D&)> odometry;
    std::function<void(roboframe::Scan2DPtr)> scan;
    std::function<void(const roboframe::MapPose2D&)> map_reference;
};
struct MappingReport {
    MappingStatus status;
    std::size_t pending = 0;
    std::uint64_t map_revision = 0;
};
struct MappingRequests {
    std::function<void(std::uint64_t)> start;
    std::function<void()> finish, cancel;
    std::function<MappingReport()> status;
};
struct MapRequests {
    std::function<void(const std::string&)> save, load, snapshot;
    std::function<MapView(bool baseline)> view;
};
struct DiagnosticRequests {
    std::function<roboframe::TransformStatus2D()> transforms;
    std::function<DataFlowStatus()> resources;
};
// Current protocol's groups, not a universal list of robot capabilities.
struct RuntimeRequests {
    std::function<void()> start, stop, reset;
    std::function<roboframe::RuntimeStatus()> status;
};
struct DemoRobotInputs {
    std::function<void(DemoRobotOptions)> configure;
    RuntimeRequests runtime;
    SensorInputs sensors;
    MappingRequests mapping;
    MapRequests maps;
    DiagnosticRequests diagnostics;
};
} // namespace demo_robot
