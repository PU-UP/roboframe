#include "products/demo_robot/demo_robot_integration.hpp"
#include "products/demo_robot/interface/demo_stdio_interface.hpp"
#include <map_server/file_map_component.hpp>
#include <chrono>
#include <cstdio>
#include <fcntl.h>
#include <sstream>
#include <thread>
#include <unistd.h>
using namespace roboframe;
using namespace demo_robot;
void Require(bool ok,int line) {
    if (!ok) throw std::runtime_error("control output check line "+std::to_string(line));
}
#define Check(x) Require((x),__LINE__)
template<class F> void Until(F f) {
    auto end=std::chrono::steady_clock::now()+std::chrono::seconds(3);
    while (!f()) { Check(std::chrono::steady_clock::now()<end);std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
struct Pipe {
    int fd[2];
    Pipe() { Check(::pipe(fd)==0); }
    ~Pipe() { ::close(fd[0]);::close(fd[1]); }
};
struct Command { std::uint64_t event,session,sequence; double linear,angular; };
std::vector<Command> ReadCommands(int fd,std::string& pending) {
    char bytes[4096];ssize_t count;
    while ((count=::read(fd,bytes,sizeof(bytes)))>0) pending.append(bytes,count);
    Check(count<0 && (errno==EAGAIN || errno==EWOULDBLOCK));
    std::vector<Command> out;
    for (auto end=pending.find('\n');end!=std::string::npos;end=pending.find('\n')) {
        std::istringstream line(pending.substr(0,end));pending.erase(0,end+1);
        std::string kind,type;Command c{};std::uint64_t generation;std::int64_t stamp;
        if (line>>kind>>c.event>>type && kind=="EVENT" && type=="CMD_VEL") {
            Check(bool(line>>c.session>>generation>>c.sequence>>stamp>>c.linear>>c.angular));out.push_back(c);
        }
    }
    return out;
}
int main() {
    char directory[]="/tmp/roboframe-output-XXXXXX";Check(::mkdtemp(directory)!=nullptr);
    auto map=std::make_shared<Grid>();map->spec={30,30,0.2,10,20,GridFrame::map};
    map->revision=7;map->stamp_us=0;map->cells=GridCells(900,254);
    map_server::FileMapComponent store(directory);store.Activate();store.Save("map",map);
    {
        Pipe input,output;Check(::fcntl(output.fd[0],F_SETFL,O_NONBLOCK)==0);
        DemoStdioInterface interface(input.fd[0],output.fd[1],{});
        DemoRobotIntegration robot(directory);
        DemoRobotOptions options{map->spec,100000,20000000};options.control_period_ms=20;options.tf_timeout_ms=450;
        robot.Configure(options);robot.LoadMap("map");robot.Start();
        robot.SetOutputs(interface.Outputs());
        const std::string subscribe="REQ 1 SUBSCRIBE ON\n";
        Check(::write(input.fd[1],subscribe.data(),subscribe.size())==ssize_t(subscribe.size()));
        interface.ProcessNext([] {});
        std::int64_t stamp=0;
        auto locate=[&] {
            stamp+=10000;robot.PushOdometry({stamp,{1,2,0}});
            robot.PushMapReference({stamp,{12.1,22.1,0}});
            robot.PushScan(std::make_shared<const Scan2D>(Scan2D{stamp,{{2,0}}}));
            Until([&] { robot.Poll();return robot.transforms().valid &&
                robot.transforms().transforms.map_from_odom.stamp_us==stamp; });
        };
        std::string pending;
        auto start=[&](std::uint64_t session) {
            locate();robot.Navigate({session,{14.1,22.1,0}});
            Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::following; });
            bool moving=false;
            Until([&] {
                for (const auto& c:ReadCommands(output.fd[0],pending)) moving|=c.linear>0;
                return moving;
            });
        };
        start(1);
        // No helper calls Poll or sends output while the owner is blocked.
        std::this_thread::sleep_for(std::chrono::milliseconds(180));
        const auto moving=ReadCommands(output.fd[0],pending);
        Check(moving.size()>=4);
        for (std::size_t i=0;i<moving.size();++i) {
            Check(moving[i].session==1 && moving[i].linear>0);
            if (i) Check(moving[i].event>moving[i-1].event && moving[i].sequence>moving[i-1].sequence);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(450));
        const auto expired=ReadCommands(output.fd[0],pending);
        Check(!expired.empty() && expired.back().linear==0 && expired.back().angular==0);
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::failed; });
        start(2);robot.CancelNavigation();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        const auto canceled=ReadCommands(output.fd[0],pending);
        Check(!canceled.empty() && canceled.back().linear==0 && canceled.back().angular==0);
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        Check(ReadCommands(output.fd[0],pending).empty());
        Until([&] { robot.Poll();return robot.navigation_status().phase==NavigationPhase::canceled; });
        start(3);robot.Stop();interface.Finish();
        const auto stopped=ReadCommands(output.fd[0],pending);
        Check(!stopped.empty() && stopped.back().linear==0 && stopped.back().angular==0);
        robot.SetOutputs({});robot.Shutdown();
    }
    Check(std::remove((std::string(directory)+"/map.rfgrid").c_str())==0);Check(::rmdir(directory)==0);
}
