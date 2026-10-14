#pragma once
#include "products/demo_robot/demo_robot_integration.hpp"
#include "products/demo_robot/data_flow/sensor_ingress.hpp"
#include <map_server/file_map_component.hpp>
#include <map_updater/occupancy_map_component.hpp>
#include <localmapbuilder/local_map_component.hpp>
namespace demo_robot {
struct DemoRobotIntegration::Impl {
    struct MapResult {
        roboframe::GridSnapshot map;
        map_updater::Update update{};
        std::uint64_t copied_bytes=0;
        std::size_t submaps=0, points=0;
        bool integrated=false, outside=false;
    };
    using MapExecution=roboframe::ExecutionChannel<LidarInput,MapResult>;
    // Task-owned state, never read by input or compute threads.
    struct MappingTask {
        MappingStatus status;
        SensorIngress::Counts inputs;
        roboframe::GridSnapshot candidate;
        std::uint64_t fence=0;
        bool busy() const { return status.phase==MappingPhase::mapping || status.phase==MappingPhase::finishing; }
        bool accepts(std::uint64_t tag) const { return busy() && tag==status.session; }
    } task_;
    Impl(DemoRobotOptions options,std::string directory,DemoRobotOutputs& outputs,std::chrono::milliseconds delay);
    void Start(std::uint64_t generation);
    void Stop() noexcept;
    void ResetData();
    void StartMapping(std::uint64_t id);
    void FinishMapping();
    void CancelMapping();
    void AbortMapping() noexcept;
    void CompleteFinish();
    void RecordTaskResult(const LidarInput& input,const MapResult& result);
    MappingStatus Status() const;
    void EmitMapping(MappingEventKind kind,const std::string& name={});
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name);
    void LoadMap(const std::string& name);
    void Publish(const roboframe::Scan2DPtr& scan,bool force=false);
    void OnMapResult(MapExecution::Result result);
    void PublishInputs();
    void AdvanceTasks();
    std::size_t PendingCompute() const;
    DataFlowStatus Resources() const;
    bool task_interrupted_=false;
    DemoRobotOutputs& outputs_;
    // Assembly is visible here; routes/tasks do not privately construct components.
    odom::PassthroughOdomComponent odom_;
    slam::RecordedSlamComponent slam_;
    posegraph::PoseGraphComponent graph_;
    localmapbuilder::LocalMapComponent local_;
    map_updater::OccupancyMapComponent grid_;
    map_server::FileMapComponent store_;
    MapExecution computation_;
    SensorIngress inputs_;
    // Compute-group-only association: carry node values, never look back into ingress.
    roboframe::PoseNode2D anchor_;
    std::int64_t correction_us_=-1;
    bool had_node_=false;
    const roboframe::GridFrame frame_;
    std::uint64_t generation_=0,baseline_generation_=0,completed_=0,copied_bytes_=0;
    roboframe::GridSnapshot computed_,frozen_,live_;
    roboframe::Scan2DPtr latest_scan_;
    DataFlowStatus resources_;
    std::int64_t last_publication_us_=-1;
    static constexpr std::size_t compute_capacity=8;
};
} // namespace demo_robot
