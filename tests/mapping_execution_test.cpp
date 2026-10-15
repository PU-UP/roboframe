#include "products/demo_robot/demo_robot_integration.hpp"
#include <thread>
#include <vector>
#include <stdexcept>
#include <unistd.h>
using namespace roboframe;
using namespace demo_robot;
using Clock=std::chrono::steady_clock;
void Check(bool ok) { if (!ok) throw std::runtime_error("execution check failed"); }
template<class F> void Until(F done) {
    const auto deadline=Clock::now()+std::chrono::seconds(5);
    while (!done()) { Check(Clock::now()<deadline); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
Scan2DPtr Scan(std::int64_t stamp) { return std::make_shared<const Scan2D>(Scan2D{stamp,{{2,0}}}); }
int main() {
    char directory[]="/tmp/roboframe-execution-XXXXXX"; Check(::mkdtemp(directory)!=nullptr);
    {
        DemoRobotIntegration robot(directory,std::chrono::milliseconds(10));
        const auto owner=std::this_thread::get_id();
        std::size_t maps=0,odoms=0;
        DemoRobotOutputs outputs;
        outputs.odometry=[&](const TimedPose2D&) { Check(std::this_thread::get_id()==owner); ++odoms; };
        outputs.map=[&](const MapView&) { Check(std::this_thread::get_id()==owner); ++maps; };
        robot.SetOutputs(outputs); robot.Configure({{20,20,1,-5,-5},100}); robot.Start(); robot.StartMapping(1);
        std::vector<std::weak_ptr<const Scan2D>> inputs;
        for (int i=0;i<1000;++i) {
            auto scan=Scan(i); inputs.push_back(scan);
            robot.PushScan(scan); robot.PushOdometry({i,{}});
        }
        Until([&] { return robot.data_flow_status().localization_results==8; });
        Check(robot.data_flow_status().localization_retained==8);
        Check(robot.data_flow_status().compute_rejected==992 && odoms==0 && maps==0);
        std::size_t retained=0;
        for (const auto& input:inputs) if (!input.expired()) ++retained;
        Check(retained==8);
        robot.CancelMapping(); Check(robot.mapping_status().drop_cancel==8);
        const auto generation=robot.runtime_status().generation;
        robot.StartMapping(2);
        Until([&] { robot.Poll();return robot.data_flow_status().pipeline_pending==0; });
        Check(robot.data_flow_status().detached_task_results==8);
        Check(robot.mapping_status().integrated==0 && maps>0 && odoms==1);
        Check(robot.runtime_status().generation==generation); // Task cancellation never resets algorithms.
        robot.PushOdometry({1000,{3,0,0}}); robot.PushScan(Scan(1000)); robot.FinishMapping();
        Until([&] { robot.Poll(); return robot.mapping_status().phase==MappingPhase::ready; });
        Check(robot.map()->revision==9 && robot.mapping_status().integrated==1);
        const auto baseline=robot.map();
        robot.Reset(); robot.Start(); robot.Poll();
        // Old snapshots deliberately retain their inputs, independently of runtime state.
        Check(robot.map()==baseline && !robot.live_map());
        robot.Shutdown();
    }
    {
        DemoRobotIntegration robot(directory,std::chrono::milliseconds(5000));
        robot.Configure({{30,20,1,-5,-5,GridFrame::map},100,20}); robot.Start();
        robot.PushOdometry({0,{}}); robot.PushMapReference({0,{10,0,0}}); robot.PushScan(Scan(0));
        Until([&] { robot.Poll(); return robot.data_flow_status().compute_active==1; });
        auto start=Clock::now();
        robot.StartMapping(1); robot.CancelMapping();
        robot.PushOdometry({1,{1,0,0}}); robot.Poll();
        Check(robot.transforms().valid && MapFromBase(robot.transforms().transforms).x==11);
        Check(Clock::now()-start<std::chrono::milliseconds(500));
        robot.PushOdometry({21,{2,0,0}}); robot.Poll(); Check(!robot.transforms().valid);
        start=Clock::now(); robot.Stop(); robot.Reset(); robot.Start();
        Check(Clock::now()-start<std::chrono::milliseconds(500));
        robot.Poll(); Check(robot.data_flow_status().stale_results==1 && !robot.live_map());
        robot.PushOdometry({0,{}}); robot.PushScan(Scan(0));
        Until([&] { robot.Poll(); return robot.data_flow_status().compute_active==1; });
        start=Clock::now(); robot.Shutdown();
        Check(Clock::now()-start<std::chrono::milliseconds(500));
        Check(robot.data_flow_status().compute_retained==0);
    }
    {
        DemoRobotIntegration robot(directory);
        robot.Configure({{30,20,1,-5,-5,GridFrame::map},100,20}); robot.Start();
        robot.PushOdometry({0,{}}); robot.PushScan(Scan(0));
        Until([&] { robot.Poll(); return robot.data_flow_status().local_updates==1; });
        Check(!robot.live_map() && robot.data_flow_status().tf_dropped==1);
        robot.PushOdometry({1,{}}); robot.PushMapReference({1,{10,0,0}}); robot.PushScan(Scan(1));
        Until([&] { robot.Poll(); return bool(robot.live_map()); });
        Check(robot.data_flow_status().local_updates==2 && robot.mapping_status().session==0);
    }
    {
        DemoRobotIntegration robot(directory,std::chrono::milliseconds(5));
        robot.Configure({{20,20,1,-5,-5},100}); robot.Start();
        std::size_t finished=0;
        std::uint64_t published_revision=0;
        DemoRobotOutputs output;
        output.map=[&](const MapView& map) { published_revision=map.grid->revision; };
        output.mapping=[&](const MappingEvent& event) {
            if (event.kind==MappingEventKind::finished) {
                ++finished;
                Check(published_revision==1);
            }
        };
        robot.SetOutputs(output); robot.StartMapping(1);
        robot.PushOdometry({0,{}}); robot.PushScan(Scan(0)); robot.FinishMapping();
        robot.PushOdometry({100000,{}}); robot.PushScan(Scan(100000));
        Until([&] { robot.Poll();return robot.data_flow_status().pipeline_pending==0; });
        Check(finished==1 && robot.map()->revision==1 && robot.live_map()->revision==2);
        // A global bounds rejection does not stop or poison the next local submap.
        robot.PushOdometry({200000,{100,0,0}}); robot.PushScan(Scan(200000));
        Until([&] { robot.Poll(); return !robot.data_flow_status().pipeline_pending; });
        robot.PushOdometry({300000,{}}); robot.PushScan(Scan(300000));
        Until([&] { robot.Poll(); return !robot.data_flow_status().pipeline_pending; });
        Check(robot.runtime_status().state==RuntimeState::running);
        Check(robot.data_flow_status().local_updates==4 && robot.live_map()->revision==3);
    }
    Check(::rmdir(directory)==0);
}
