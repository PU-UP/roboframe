#include "products/demo_robot/demo_robot_integration.hpp"
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace roboframe;
using namespace demo_robot;
void Check(bool value) { if (!value) throw std::runtime_error("async localization check failed"); }
template<class F> void Until(F f) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(4);
    while (!f()) { Check(std::chrono::steady_clock::now()<deadline);std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
int main() {
    char directory[]="/tmp/roboframe-localization-XXXXXX";Check(::mkdtemp(directory)!=nullptr);
    {
        DemoRobotIntegration robot(directory);
        DemoRobotOptions options{{30,20,1,-5,-5,GridFrame::map},100,20000000};
        options.localization_test_delay_ms=200;
        robot.Configure(options);robot.Start();robot.StartMapping(1);
        robot.PushOdometry({0,{}});robot.PushMapReference({0,{3,0,0}});
        robot.PushScan(std::make_shared<const Scan2D>(Scan2D{0,{{2,0}}}));
        Until([&] { return robot.data_flow_status().localization_active==1; });
        const auto before=std::chrono::steady_clock::now();
        robot.PushOdometry({1,{1,0,0}});
        robot.FinishMapping();robot.Poll();
        Check(std::chrono::steady_clock::now()-before<std::chrono::milliseconds(100));
        Check(robot.mapping_status().phase==MappingPhase::finishing && !robot.map());
        robot.PushScan(std::make_shared<const Scan2D>(Scan2D{1,{{2,0}}}));
        Until([&] { robot.Poll();return robot.mapping_status().phase==MappingPhase::ready; });
        Check(robot.map()->revision==1);
        const auto tf=robot.transforms();
        Check(tf.valid && tf.transforms.odom_from_base.stamp_us==1 && MapFromBase(tf.transforms).x==4);
        Until([&] { robot.Poll();return robot.pending()==0; });
        Check(robot.live_map()->revision==2 && robot.map()->revision==1);
        robot.PushOdometry({2,{2,0,0}});
        robot.PushScan(std::make_shared<const Scan2D>(Scan2D{2,{{2,0}}}));
        Until([&] { return robot.data_flow_status().localization_active==1; });
        robot.Stop();robot.Reset();robot.Start();robot.Poll();
        Check(!robot.transforms().valid && !robot.live_map() && robot.runtime_status().stale_results>=1);
        robot.Shutdown();
    }
    Check(::rmdir(directory)==0);
}
