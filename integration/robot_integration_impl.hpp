#pragma once
#include "integration/robot_integration.hpp"
#include "integration/data_flow/sensor_flow.hpp"
#include <roboframe/execution.hpp>
#include <map_server/file_map_component.hpp>
#include <map_updater/occupancy_map_component.hpp>
#include <planner/read_only_planner_component.hpp>
namespace roboframe {
// Business state belongs to the command/Poll owner. SensorFlow owns its own synchronized
// routing state; the compute channel exclusively owns algorithm mutations while active.
struct RobotIntegration::Impl {
    struct ScanWork { Scan2DPtr scan; ScanTransforms2D transforms; };
    struct MapResult { GridSnapshot map; map_updater::Update update; std::uint64_t copied_bytes = 0; };
    using MapExecution = ExecutionChannel<ScanWork, MapResult>;
    Impl(MappingOptions options, std::string directory, ProductOutputs& outputs, std::chrono::milliseconds delay);
    ~Impl();
    void StartMapping(std::uint64_t id);
    void FinishMapping();
    void CancelMapping();
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name);
    void LoadMap(const std::string& name);
    void Shutdown();
    void RequireActive() const;
    void Publish(const Scan2DPtr& scan, bool force = false);
    void WaitPlanner();
    void Poll();
    void CompleteFinish();
    void OnMapResult(MapExecution::Result result);
    void OnMapPublished();
    void AdvanceWorkflows();
    void CountInputs(const SensorFlow::Counts& input, MappingStatus& task, DataFlowStatus& resources) const;
    void CaptureInputs(const SensorFlow::State& input);
    std::size_t PendingCompute() const;
    MappingStatus Status() const;
    DataFlowStatus Resources() const;
    void EmitMapping(MappingEventKind kind, const std::string& name = {});
    ProductOutputs& outputs_;
    map_updater::OccupancyMapComponent grid_;
    std::uint64_t computed_generation_ = 0; // Accessed only by the compute handler.
    MapExecution computation_;
    SensorFlow inputs_;
    map_server::FileMapComponent store_;
    planner::ReadOnlyPlannerComponent planner_{std::chrono::milliseconds(5)};
    const GridFrame frame_;
    std::uint64_t generation_ = 0, completed_ = 0, copied_bytes_ = 0;
    GridSnapshot computed_, frozen_, live_;
    Scan2DPtr latest_scan_;
    MappingStatus status_;
    SensorFlow::Counts accounted_inputs_;
    DataFlowStatus resources_;
    bool updating_ = false, active_ = false;
    std::int64_t last_publication_us_ = -1;
    static constexpr std::size_t compute_capacity = 8;
};
} // namespace roboframe
