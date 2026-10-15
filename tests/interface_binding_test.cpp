#include "products/demo_robot/interface/demo_stdio_interface.hpp"
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace roboframe;
using namespace demo_robot;
void Check(bool ok) { if (!ok) throw std::runtime_error("interface binding check failed"); }
struct Pipe {
    int fd[2];
    Pipe() { Check(::pipe(fd) == 0); }
    ~Pipe() { for (int f : fd) if (f >= 0) ::close(f); }
    void CloseWriter() { ::close(fd[1]); fd[1] = -1; }
};
std::string Exchange(DemoRobotInputs inputs, const std::string& requests,
                     const std::function<void()>& service,
                     const std::function<void(DemoRobotOutputs)>& stop = {},
                     DemoStdioInterface::ReceiveResult* reason = nullptr) {
    Pipe incoming, outgoing;
    Check(::write(incoming.fd[1], requests.data(), requests.size()) == static_cast<ssize_t>(requests.size()));
    incoming.CloseWriter();
    {
        DemoStdioInterface transport(incoming.fd[0], outgoing.fd[1], std::move(inputs));
        DemoStdioInterface::ReceiveResult result;
        do { result = transport.ProcessNext(service); } while (result == DemoStdioInterface::ReceiveResult::received);
        if (reason) *reason = result;
        if (stop) stop(transport.Outputs()); // Application decides what ending the connection means.
        transport.Finish();
    }
    outgoing.CloseWriter();
    std::string result;
    char buffer[4096];
    ssize_t count;
    while ((count = ::read(outgoing.fd[0], buffer, sizeof(buffer))) > 0) result.append(buffer, count);
    Check(count == 0);
    return result;
}
int main() {
    const auto owner = std::this_thread::get_id();
    int ticks = 0, scans = 0, stops = 0;
    std::uint64_t started = 0;
    DemoRobotInputs mapping_product;
    mapping_product.mapping.start = [&](std::uint64_t id) { Check(std::this_thread::get_id() == owner); started = id; };
    mapping_product.sensors.scan = [&](Scan2DPtr scan) { Check(scan->stamp_us == 7 && scan->hits.size() == 1); ++scans; };
    DemoStdioInterface::ReceiveResult reason;
    auto output = Exchange(mapping_product,
        "REQ 1 SUBSCRIBE ON\nREQ 2 BEGIN 3\nDATA SCAN 7 1 2 0\nREQ 3 FINISH\nREQ 4 QUIT\n",
        [&] { Check(std::this_thread::get_id() == owner && stops == 0); ++ticks; },
        [&](DemoRobotOutputs publish) { ++stops; publish.mapping({MappingEventKind::stopped, {}}); }, &reason);
    Check(reason == DemoStdioInterface::ReceiveResult::quit);
    Check(started == 3 && scans == 1 && ticks > 0 && stops == 1);
    Check(output.find("RES 2 OK BEGIN") != std::string::npos);
    Check(output.find("RES 3 ERROR unsupported: FINISH") != std::string::npos);
    Check(output.find("MAPPING stopped") < output.find("RES 4 BYE"));
    Check(output.find("MAPPING stopped 4") != std::string::npos);

    DemoRobotInputs navigation_product;
    int goals=0,cancels=0;
    navigation_product.navigation.start=[&](NavigationGoal goal) {
        Check(goal.session==11 && goal.pose.x==2 && goal.pose.y==3 && goal.pose.yaw==0.5);++goals;
    };
    navigation_product.navigation.cancel=[&] { ++cancels; };
    navigation_product.navigation.status=[] { return NavigationStatus{NavigationPhase::following,11,2,{}}; };
    auto path=std::make_shared<Path2D>();path->map_revision=4;path->poses={{1,2,0},{2,3,0.5}};
    navigation_product.navigation.path=[&] { return NavigationPath{11,2,path}; };
    output=Exchange(navigation_product,
        "REQ 1 SUBSCRIBE ON\nREQ 2 NAVIGATE 11 2 3 0.5\nREQ 3 NAV_STATUS\nREQ 4 GET_PATH\n"
        "REQ 5 NAVIGATE 12 nan 3 0\nREQ 6 NAV_CANCEL\nREQ 7 QUIT\n",[] {},
        [](DemoRobotOutputs publish) {
            publish.velocity({11,2,5,100,{}});
            publish.navigation({NavigationPhase::canceled,11,2,"requested"});
        });
    Check(goals==1 && cancels==1 && output.find("RES 5 ERROR")!=std::string::npos);
    Check(output.find("RES 3 NAV following 11 2 -")!=std::string::npos);
    Check(output.find("RES 4 PATH 11 2 4 map 2 1 2 0 2 3 0.5")!=std::string::npos);
    Check(output.find("CMD_VEL 11 2 5 100 0 0")!=std::string::npos);
    Check(output.find("NAV canceled 11 2 requested")!=std::string::npos);

    // The same interface works with a sensor-only product, with no integration linked.
    DemoRobotInputs sensor_product;
    int odoms = 0;
    sensor_product.sensors.odometry = [&](const TimedPose2D& pose) { Check(pose.stamp_us == 8); ++odoms; };
    output = Exchange(sensor_product, "REQ 1 BEGIN 9\nDATA ODOM 8 0 0 0\n", [] {}, {}, &reason);
    Check(reason == DemoStdioInterface::ReceiveResult::end_of_input);
    Check(odoms == 1 && output.find("RES 1 ERROR unsupported: BEGIN") != std::string::npos);
    Check(output.find("BYE") == std::string::npos); // EOF is not a QUIT request.
    output = Exchange({}, "REQ 1 BEGIN", [] {}, {}, &reason);
    Check(reason == DemoStdioInterface::ReceiveResult::invalid_input);
    Check(output.find("RES 0 ERROR incomplete line") != std::string::npos);
    bool propagated = false;
    try { Exchange({}, "", [] { throw std::invalid_argument("service failure"); }); }
    catch (const std::invalid_argument& error) { propagated = std::string(error.what()) == "service failure"; }
    Check(propagated); // A product failure must not be reported as malformed IPC input.
}
