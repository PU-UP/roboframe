#include "products/demo_robot/demo_robot_integration.hpp"
#include <map_server/file_map_component.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <cstdio>
#include <unistd.h>
using namespace roboframe;
using namespace demo_robot;
void Require(bool ok,int line) { if (!ok) throw std::runtime_error("navigation check line "+std::to_string(line)); }
#define Check(ok) Require((ok),__LINE__)
template<class F> void Until(F done) {
    auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(5);
    while (!done()) { Check(std::chrono::steady_clock::now()<deadline);std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
int main() {
    char directory[]="/tmp/roboframe-nav-XXXXXX";Check(::mkdtemp(directory)!=nullptr);
    auto map=std::make_shared<Grid>();map->spec={30,30,0.2,10,20,GridFrame::map};map->revision=7;map->stamp_us=0;map->cells=GridCells(900,254);
    map_server::FileMapComponent store(directory);store.Activate();store.Save("test",map);
    {
        DemoRobotIntegration robot(directory);robot.Configure({map->spec,100000,20000000});robot.LoadMap("test");robot.Start();
        std::vector<VelocityCommand> commands;
        std::vector<NavigationStatus> events;
        NavigationPath published;
        std::mutex commands_mutex;
        std::atomic<bool> worker_output{false};
        const auto owner=std::this_thread::get_id();
        DemoRobotOutputs output;
        output.velocity=[&](const VelocityCommand& v) { std::lock_guard<std::mutex> lock(commands_mutex);if (std::this_thread::get_id()!=owner) worker_output=true;commands.push_back(v); };
        output.navigation=[&](const NavigationStatus& s) { events.push_back(s); };
        output.path=[&](const NavigationPath& p) { published=p; };
        robot.SetOutputs(output);
        bool rejected=false;try { robot.Navigate({1,{14.1,22.1,0}}); } catch (const std::logic_error&) { rejected=true; }Check(rejected);
        std::int64_t stamp=0;
        auto locate=[&](Pose2D pose) {
            stamp+=10000;robot.PushOdometry({stamp,{1,2,0}});robot.PushMapReference({stamp,pose});
            robot.PushScan(std::make_shared<const Scan2D>(Scan2D{stamp,{{2,0}}}));
            Until([&] { robot.Poll();return robot.transforms().valid && robot.transforms().transforms.map_from_odom.stamp_us==stamp; });
        };
        auto command_snapshot=[&] { std::lock_guard<std::mutex> lock(commands_mutex);return commands; };
        auto stopped=[&] { std::lock_guard<std::mutex> lock(commands_mutex);return !commands.empty() && commands.back().velocity.linear==0 && commands.back().velocity.angular==0; };
        locate({12.1,22.1,0});robot.Navigate({1,{14.1,22.1,0}});robot.CancelNavigation();
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::canceled; });Check(stopped());
        robot.Navigate({2,{14.1,22.1,0}});
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::following; });
        Check(published.session==2 && published.path==robot.navigation_path().path);
        const auto until=std::chrono::steady_clock::now()+std::chrono::milliseconds(160);
        while (std::chrono::steady_clock::now()<until) { robot.Poll();std::this_thread::sleep_for(std::chrono::milliseconds(2)); }
        Check(robot.navigation_status().phase==NavigationPhase::following); // Unconsumed commands do not move the robot.
        bool moving=false;for (const auto& c:command_snapshot()) moving|=c.session==2 && c.velocity.linear>0;Check(moving);
        rejected=false;try { robot.SaveMap("blocked"); } catch (const std::logic_error&) { rejected=true; }Check(rejected);
        // Independent pose fixtures test completion; these are not generated from cmd_vel.
        const auto path=published.path;
        for (const auto& pose:path->poses) {
            locate(pose);
            const auto next=std::chrono::steady_clock::now()+std::chrono::milliseconds(60);
            while (std::chrono::steady_clock::now()<next) { robot.Poll();std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
        }
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::succeeded; });Check(stopped());
        robot.Navigate({3,{100,100,0}});
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::failed; });Check(stopped());
        locate({12.1,22.1,0});robot.Navigate({4,{14.1,22.1,0}});
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::following; });
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::failed; });
        Check(robot.navigation_status().reason=="tf_unavailable" && stopped());
        locate({12.1,22.1,0});robot.Navigate({5,{14.1,22.1,0}});robot.Stop();
        Check(stopped() && robot.navigation_status().phase==NavigationPhase::canceled);
        const auto generation=robot.runtime_status().generation;robot.Reset();robot.Start();robot.Poll();
        Check(robot.runtime_status().generation>generation && robot.navigation_status().phase==NavigationPhase::canceled);
        // Reset starts a fresh sensor clock; old navigation results cannot install a path.
        stamp=0;locate({12.1,22.1,0});robot.Navigate({6,{14.1,22.1,0}});
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::following; });
        robot.CancelNavigation();
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::canceled; });Check(stopped());
        const auto count=command_snapshot().size();robot.Poll();Check(command_snapshot().size()==count && worker_output);
        robot.SetOutputs({});robot.Shutdown();
    }
    Check(std::remove((std::string(directory)+"/test.rfgrid").c_str())==0);Check(::rmdir(directory)==0);
}
