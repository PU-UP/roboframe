#include "products/demo_robot/demo_robot_integration.hpp"
#include <thread>
#include <stdexcept>
#include <unistd.h>
using namespace roboframe;
using namespace demo_robot;
void Check(bool ok) { if (!ok) throw std::runtime_error("mapping BT check failed"); }
template<class F> void Until(F done) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (!done()) {
        Check(std::chrono::steady_clock::now()<deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
void Feed(DemoRobotIntegration& robot,std::int64_t stamp) {
    robot.PushOdometry({stamp,{}});
    robot.PushScan(std::make_shared<const Scan2D>(Scan2D{stamp,{{2,0}}}));
}
void Phase(DemoRobotIntegration& robot,MappingPhase phase) {
    Check(robot.mapping_status().phase==phase);
}
int main() {
    char directory[]="/tmp/roboframe-bt-XXXXXX"; Check(::mkdtemp(directory)!=nullptr);
    {
        DemoRobotIntegration robot(directory,std::chrono::milliseconds(50));
        std::size_t finished=0,failed=0,canceled=0,stopped=0;
        const auto owner=std::this_thread::get_id();
        DemoRobotOutputs outputs;
        outputs.mapping=[&](const MappingEvent& event) {
            Check(std::this_thread::get_id()==owner);
            if (event.kind==MappingEventKind::finished) ++finished;
            if (event.kind==MappingEventKind::failed) ++failed;
            if (event.kind==MappingEventKind::canceled) ++canceled;
            if (event.kind==MappingEventKind::stopped) ++stopped;
        };
        robot.SetOutputs(outputs); robot.Configure({{20,20,1,-5,-5},100}); robot.Start();
        Phase(robot,MappingPhase::idle);
        Feed(robot,0);
        Until([&] { robot.Poll(); return !robot.pending(); });
        Phase(robot,MappingPhase::idle); Check(robot.live_map()->revision==1);

        robot.StartMapping(1); Phase(robot,MappingPhase::mapping);
        bool rejected=false;
        try { robot.FinishMapping(); } catch (const std::logic_error&) { rejected=true; }
        Check(rejected); Phase(robot,MappingPhase::mapping);
        Feed(robot,1); robot.FinishMapping(); robot.Poll();
        Phase(robot,MappingPhase::finishing);
        Feed(robot,2); // Later work must not extend the frozen boundary.
        Until([&] { robot.Poll(); return !robot.pending(); });
        Phase(robot,MappingPhase::ready);
        Check(finished==1 && robot.map()->revision==2 && robot.live_map()->revision==3);
        const auto baseline=robot.map();
        robot.Poll(); Check(finished==1);

        robot.StartMapping(2); Feed(robot,3); robot.FinishMapping(); robot.Poll();
        Phase(robot,MappingPhase::finishing);
        const auto generation=robot.runtime_status().generation;
        robot.CancelMapping(); Phase(robot,MappingPhase::idle);
        robot.StartMapping(3);
        Until([&] { robot.Poll(); return !robot.pending(); });
        Check(robot.mapping_status().integrated==0 && robot.data_flow_status().detached_task_results==1);
        Check(robot.runtime_status().generation==generation && robot.live_map()->revision==4);
        Check(robot.map()==baseline && finished==1 && canceled==1);
        robot.CancelMapping(); Phase(robot,MappingPhase::idle); // Halt Collect too.

        robot.StartMapping(4);
        robot.PushScan(std::make_shared<const Scan2D>(Scan2D{10,{{2,0}}}));
        robot.FinishMapping(); robot.Poll(); // No associated pose: Freeze fails without overwriting baseline.
        Phase(robot,MappingPhase::idle);
        Check(failed==1 && robot.map()==baseline);
        robot.Poll(); Check(failed==1 && finished==1);
        robot.PushOdometry({10,{}});
        Until([&] { robot.Poll(); return !robot.pending(); });
        Check(robot.live_map()->revision==5);

        robot.StartMapping(5); Feed(robot,11); robot.FinishMapping(); robot.Poll();
        Phase(robot,MappingPhase::finishing);
        Until([&] { robot.Poll(); return robot.data_flow_status().compute_active==1; });
        robot.Stop(); Phase(robot,MappingPhase::idle); Check(stopped==1);
        robot.Reset(); robot.Start(); robot.StartMapping(6); robot.Poll();
        Phase(robot,MappingPhase::mapping);
        Check(robot.mapping_status().integrated==0 && !robot.live_map() && robot.map()==baseline);
        Check(robot.data_flow_status().stale_results>=1);
        Feed(robot,0); robot.FinishMapping();
        Until([&] { robot.Poll(); return robot.mapping_status().phase==MappingPhase::ready; });
        Check(finished==2 && robot.map()->revision==1);
        robot.Shutdown(); Phase(robot,MappingPhase::ready);
    }
    Check(::rmdir(directory)==0);
}
