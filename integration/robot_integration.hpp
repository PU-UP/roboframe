#pragma once
#include <roboframe/scan2d.hpp>
#include <roboframe/mapping.hpp>
#include <roboframe/data_flow.hpp>
#include <roboframe/transforms2d.hpp>
#include <roboframe/outputs.hpp>
#include <memory>
#include <chrono>
#include <string>

namespace roboframe {
// Configure before starting producers; destroy only after producers have joined.
// PushOdometry/PushScan/PushMapReference may run concurrently with each other and commands.
// Commands, queries, SetOutputs and Poll have one business owner; output callbacks run there.
class RobotIntegration {
public:
    explicit RobotIntegration(std::string directory, std::chrono::milliseconds mapper_test_delay = std::chrono::milliseconds(0));
    ~RobotIntegration();
    RobotIntegration(const RobotIntegration&) = delete;
    RobotIntegration& operator=(const RobotIntegration&) = delete;
    // Commit only after successful activation; failed configuration may be retried.
    void Configure(MappingOptions options);
    void SetOutputs(ProductOutputs outputs); // Business owner only.
    MapView map_view(bool baseline) const;
    void StartMapping(std::uint64_t id);
    void FinishMapping(); // Accept finish; finished/failed event is emitted by Poll.
    void Poll(); // Owner thread only; drains a bounded batch of worker completions.
    void CancelMapping();
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name); // Inspection checkpoint; does not finish mapping.
    void LoadMap(const std::string& name);
    void PushOdometry(const TimedPose2D& pose);
    void PushMapReference(const MapPose2D& pose);
    TransformStatus2D transforms() const;
    void PushScan(roboframe::Scan2DPtr scan);
    void Shutdown();
    MappingStatus mapping_status() const;
    std::size_t pending() const;
    GridSnapshot map() const; // Saved baseline captured by FINISH or LOAD.
    GridSnapshot live_map() const;
    DataFlowStatus data_flow_status() const;
    void WaitPlanner(); // Nonblocking idle check; throws while computation/reading is pending.
private:
    struct Impl;
    Impl& Configured() const;
    std::string map_directory_;
    const std::chrono::milliseconds mapper_test_delay_;
    ProductOutputs outputs_; // Outlives Impl; callbacks never run from its destructor.
    std::unique_ptr<Impl> impl_;
};
} // namespace roboframe
