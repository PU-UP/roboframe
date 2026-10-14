#include "products/demo_robot/demo_robot_integration.hpp"
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>
using namespace roboframe;
using namespace demo_robot;
void Check(bool ok) { if (!ok) throw std::runtime_error("integration check failed"); }
template<class F> void Reject(F f) {
    bool caught=false; try { f(); } catch (const std::exception&) { caught=true; } Check(caught);
}
Scan2DPtr Scan(std::int64_t t) { return std::make_shared<const Scan2D>(Scan2D{t,{{2,0}}}); }
void Drain(DemoRobotIntegration& robot) {
    const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    do {
        robot.Poll();
        if (!robot.data_flow_status().compute_retained) { robot.Poll(); return; }
        Check(std::chrono::steady_clock::now()<until);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (true);
}
int main() {
    char directory[]="/tmp/roboframe-mapping-XXXXXX";
    Check(::mkdtemp(directory)!=nullptr);
    {
        const auto path=std::string(directory)+"/late";
        DemoRobotIntegration retry(path);
        Reject([&] { retry.Configure({{20,20,1,-5,-5},100}); });
        Check(::mkdir(path.c_str(),0700)==0);
        retry.Configure({{20,20,1,-5,-5},100}); retry.Start();
        Check(::rmdir(path.c_str())==0);
    }
    DemoRobotIntegration robot(directory);
    Reject([&] { robot.StartMapping(1); });
    Reject([&] { robot.Start(); });
    Check(robot.runtime_status().state==RuntimeState::idle);
    Reject([&] { robot.Configure({{0,20,1,-5,-5},100}); });
    robot.Configure({{20,20,1,-5,-5},100});
    Check(robot.runtime_status().state==RuntimeState::idle);
    Reject([&] { robot.PushOdometry({0,{}}); });
    Reject([&] { robot.StartMapping(1); });
    robot.Start();
    // No task is required for odometry, local submaps or live occupancy.
    robot.PushOdometry({0,{}}); robot.PushScan(Scan(0)); Drain(robot);
    Check(robot.mapping_status().phase==MappingPhase::idle);
    Check(robot.data_flow_status().local_updates==1 && robot.live_map());
    robot.StartMapping(1);
    robot.PushScan(Scan(50)); Check(robot.pending()==1);
    robot.PushOdometry({100,{2,0,0}}); Drain(robot);
    robot.FinishMapping(); Drain(robot);
    const auto frozen=robot.map();
    Check(frozen && frozen->cells[5*20+8]==0);
    robot.SaveMap("first"); Reject([&] { robot.SaveMap("first"); });
    robot.PushOdometry({200,{3,0,0}}); robot.PushScan(Scan(150)); Drain(robot);
    Check(robot.map()==frozen && robot.live_map()->revision>frozen->revision);
    Check(robot.mapping_status().integrated==1);
    Reject([&] { robot.LoadMap("first"); });
    robot.StartMapping(2);
    robot.PushScan(Scan(250)); // Cancel does not discard a frame waiting for odometry.
    robot.CancelMapping();
    const auto generation=robot.runtime_status().generation;
    robot.PushOdometry({300,{4,0,0}}); Drain(robot);
    Check(robot.data_flow_status().local_updates==4 && robot.transforms().valid);
    Check(robot.runtime_status().generation==generation);
    robot.StartMapping(3);
    robot.PushScan(Scan(350));
    robot.FinishMapping(); robot.Poll(); // Task detaches unmatched scan, continuous route keeps it.
    Check(robot.mapping_status().phase==MappingPhase::idle);
    Check(robot.mapping_status().drop_sync==1 && robot.pending()==1);
    robot.PushOdometry({400,{4,0,0}}); Drain(robot);
    Check(robot.data_flow_status().local_updates==5);
    Check(robot.map()==frozen);
    robot.Stop(); Check(!robot.transforms().valid);
    robot.LoadMap("first"); Check(robot.map()->cells==frozen->cells);
    Reject([&] { robot.LoadMap("missing"); });
    robot.Reset(); Check(!robot.live_map() && robot.map()->cells==frozen->cells);
    robot.Start(); robot.PushOdometry({0,{}}); robot.PushScan(Scan(0)); Drain(robot);
    Check(robot.runtime_status().generation>generation && robot.live_map()->revision==1);
    robot.Shutdown(); robot.Shutdown(); Reject([&] { robot.Start(); });
    std::remove((std::string(directory)+"/first.rfgrid").c_str()); Check(::rmdir(directory)==0);
}
