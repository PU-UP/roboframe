#include "products/demo_robot/demo_robot_integration_impl.hpp"
#include <roboframe/core/log.hpp>
namespace demo_robot {
using namespace roboframe;
void DemoRobotIntegration::Impl::EmitMapping(MappingEventKind kind,const std::string& name) {
    if (outputs_.mapping) outputs_.mapping({kind,Status(),generation_,name});
}
void DemoRobotIntegration::Impl::StartMapping(std::uint64_t id) {
    if (task_.busy()) throw std::logic_error("mapping already running");
    if (!id || id<=task_.status.session) throw std::invalid_argument("session id must increase");
    mapping_tree_.Halt();
    inputs_.BeginTag(id);
    task_={}; task_.status.session=id; task_.status.phase=MappingPhase::mapping;
    AdvanceTasks();
    EmitMapping(MappingEventKind::started);
}
void DemoRobotIntegration::Impl::FinishMapping() {
    if (task_.status.phase!=MappingPhase::mapping) throw std::logic_error("no mapping session");
    if (!inputs_.status().counts.received) throw std::logic_error("cannot finish an empty task");
    const auto fence=inputs_.EndTag(true);
    task_.inputs=fence.counts;
    task_.status.phase=MappingPhase::finishing; EmitMapping(MappingEventKind::finishing);
}
bt::Status DemoRobotIntegration::Impl::CompleteFinish() {
    if (!task_.candidate) {
        task_.status.phase=MappingPhase::idle; EmitMapping(MappingEventKind::failed);
        return bt::Status::failure;
    }
    frozen_=std::move(task_.candidate); baseline_generation_=generation_;controller_.UpdateMap(frozen_);
    task_.status.phase=MappingPhase::ready; EmitMapping(MappingEventKind::finished);
    return bt::Status::success;
}
void DemoRobotIntegration::Impl::CancelMapping() {
    if (!task_.busy()) throw std::logic_error("no mapping session");
    mapping_tree_.Halt();
    EmitMapping(MappingEventKind::canceled);
}
void DemoRobotIntegration::Impl::HaltMapping() noexcept {
    if (task_.status.phase==MappingPhase::mapping) task_.inputs=inputs_.EndTag(false).counts;
    AbortMapping();
}
void DemoRobotIntegration::Impl::AbortMapping() noexcept {
    if (!task_.busy()) return;
    if (task_.status.phase==MappingPhase::mapping) task_.inputs=inputs_.status().counts;
    const auto status=Status();
    const auto abandoned=status.received-status.integrated-status.dropped;
    task_.status.dropped+=abandoned; task_.status.drop_cancel+=abandoned;
    task_.status.phase=MappingPhase::idle; task_.candidate.reset();
}
void DemoRobotIntegration::Impl::RecordTaskResult(const LidarInput& input,const MapResult& result) {
    if (!task_.accepts(input.task)) return; // Late results update live data, never another task.
    if (result.integrated) {
        ++task_.status.integrated; task_.status.used_rays+=result.update.used_rays;
        task_.status.outside_rays+=result.update.outside_rays; task_.candidate=result.map;
    } else {
        ++task_.status.dropped;
        if (result.outside) ++task_.status.drop_bounds;
        else ++task_.status.drop_sync;
    }
}
MappingStatus DemoRobotIntegration::Impl::Status() const {
    auto status=task_.status;
    const auto counts=status.phase==MappingPhase::mapping ? inputs_.status().counts : task_.inputs;
    status.received=counts.received;
    status.dropped+=counts.dropped; status.drop_sync+=counts.drop_sync; status.drop_queue+=counts.drop_queue;
    return status;
}
void DemoRobotIntegration::Impl::AdvanceTasks() {
    if (!task_.busy()) return;
    const auto before=mapping_tree_.status();
    mapping_tree_.Tick();
    const auto after=mapping_tree_.status();
    if (before.status!=after.status || before.node!=after.node) {
        const char* state=after.status==bt::Status::success ? "SUCCESS" :
            after.status==bt::Status::failure ? "FAILURE" : "RUNNING";
        ROBO_LOG_INFO("mapping-bt") << "session=" << task_.status.session << " " << state << " node=" << after.node;
    }
}
void DemoRobotIntegration::Impl::SaveMap(const std::string& name) {
    if (navigation_.busy()) throw std::logic_error("map file I/O disabled during navigation");
    if (!frozen_) throw std::logic_error("finish or load before save");
    store_.Save(name,frozen_); EmitMapping(MappingEventKind::saved,name);
}
void DemoRobotIntegration::Impl::SnapshotMap(const std::string& name) {
    if (navigation_.busy()) throw std::logic_error("map file I/O disabled during navigation");
    if (!computed_) throw std::logic_error("snapshot requires a nonempty map");
    store_.Save(name,computed_);
}
void DemoRobotIntegration::Impl::LoadMap(const std::string& name) {
    auto map=store_.Load(name);
    if (map->spec.frame!=frame_) throw std::invalid_argument("loaded map frame differs from product frame");
    frozen_=std::move(map); baseline_generation_=0;controller_.UpdateMap(frozen_);
    // Loaded asset is not a live atlas or localization.
    task_.status.phase=MappingPhase::ready; EmitMapping(MappingEventKind::loaded,name);
}
} // namespace demo_robot
