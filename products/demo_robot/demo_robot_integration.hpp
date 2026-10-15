#pragma once
#include <roboframe/datatype/scan2d.hpp>
#include <roboframe/core/integration.hpp>
#include <demo_robot/mapping.hpp>
#include <demo_robot/navigation.hpp>
#include <demo_robot/config.hpp>
#include <demo_robot/diagnostics.hpp>
#include <roboframe/datatype/transforms2d.hpp>
#include <demo_robot/outputs.hpp>
#include <memory>
#include <chrono>
#include <string>

namespace demo_robot {
// Configure before starting producers. Start opens ingress; idle inputs are rejected.
// Stop may overlap producers; destroy only after producers have joined.
// PushOdometry/PushScan/PushMapReference may run concurrently with each other and commands.
// Commands, queries, SetOutputs and Poll have one business owner; output callbacks run there.
class DemoRobotIntegration final : public roboframe::Integration {
public:
    explicit DemoRobotIntegration(std::string directory, std::chrono::milliseconds mapper_test_delay = std::chrono::milliseconds(0));
    ~DemoRobotIntegration();
    DemoRobotIntegration(const DemoRobotIntegration&) = delete;
    DemoRobotIntegration& operator=(const DemoRobotIntegration&) = delete;
    // Configuration does not start algorithms. Call Start explicitly afterwards.
    void Configure(DemoRobotOptions options);
    void SetOutputs(DemoRobotOutputs outputs); // Business owner only.
    MapView map_view(bool baseline) const;
    void StartMapping(std::uint64_t id);
    void FinishMapping(); // Accept finish; finished/failed event is emitted by Poll.
    void CancelMapping();
    void Navigate(NavigationGoal goal);
    void CancelNavigation();
    NavigationStatus navigation_status() const;
    NavigationPath navigation_path() const;
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name); // Inspection checkpoint; does not finish mapping.
    void LoadMap(const std::string& name);
    void PushOdometry(const roboframe::TimedPose2D& pose);
    void PushMapReference(const roboframe::MapPose2D& pose);
    roboframe::TransformStatus2D transforms() const;
    void PushScan(roboframe::Scan2DPtr scan);
    MappingStatus mapping_status() const;
    std::size_t pending() const;
    roboframe::GridSnapshot map() const; // Saved baseline captured by FINISH or LOAD.
    roboframe::GridSnapshot live_map() const;
    DataFlowStatus data_flow_status() const;
private:
    void ValidateStart() const override;
    void OnStateChanged() override;
    void OnStart(std::uint64_t generation) override;
    void CloseInputs() noexcept override;
    void OnStop() noexcept override;
    void OnReset() override;
    void OnPoll() override;
    struct Impl;
    Impl& Configured() const;
    std::string map_directory_;
    const std::chrono::milliseconds mapper_test_delay_;
    DemoRobotOutputs outputs_; // Outlives Impl; callbacks never run from its destructor.
    std::unique_ptr<Impl> impl_;
};
} // namespace demo_robot
