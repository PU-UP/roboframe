#include "integration/robot_integration.hpp"
#include <atomic>
#include <thread>
#include <stdexcept>
#include <unistd.h>
using namespace roboframe;
using Clock = std::chrono::steady_clock;
void Check(bool ok) { if (!ok) throw std::runtime_error("concurrent ingress failed"); }
template<class F> void Until(F done) {
    const auto deadline = Clock::now() + std::chrono::seconds(3);
    while (!done()) { Check(Clock::now() < deadline); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
int main() {
    char directory[] = "/tmp/roboframe-ingress-XXXXXX";
    Check(::mkdtemp(directory) != nullptr);
    RobotIntegration robot(directory, std::chrono::milliseconds(5000));
    robot.Configure({{20, 20, 1, -5, -5}, 100});
    const auto owner = std::this_thread::get_id();
    std::int64_t latest_odom = -1;
    ProductOutputs output;
    output.odometry = [&](const TimedPose2D& pose) { Check(std::this_thread::get_id() == owner); latest_odom = pose.stamp_us; };
    output.mapping = [&](const MappingEvent&) { Check(std::this_thread::get_id() == owner); };
    robot.SetOutputs(output); robot.StartMapping(1);
    robot.PushOdometry({0, {}}); robot.PushScan(std::make_shared<const Scan2D>(Scan2D{0, {{2, 0}}}));
    Until([&] { return robot.data_flow_status().compute_active == 1; });
    std::atomic<int> odoms{0}, scans{0}, closed{0};
    std::atomic<bool> go{false}, stop{false};
    auto odom_input = std::thread([&] {
        while (!go) std::this_thread::yield();
        for (int i = 1; !stop; ++i) {
            try { robot.PushOdometry({i, {}}); ++odoms; }
            catch (const std::logic_error&) { ++closed; break; }
        }
    });
    auto scan_input = std::thread([&] {
        while (!go) std::this_thread::yield();
        for (int i = 1; !stop; ++i) {
            try { robot.PushScan(std::make_shared<const Scan2D>(Scan2D{i, {{2, 0}}})); ++scans; }
            catch (const std::logic_error&) { ++closed; break; }
        }
    });
    go = true;
    Until([&] { robot.Poll(); return scans >= 500 && odoms >= 500; });
    Check(latest_odom >= 0 && robot.data_flow_status().compute_peak_retained <= 8);
    auto before = Clock::now();
    for (std::uint64_t session = 2; session <= 20; ++session) {
        robot.CancelMapping(); robot.StartMapping(session); robot.Poll();
        Check(robot.mapping_status().session == session && robot.mapping_status().integrated == 0);
    }
    Check(Clock::now() - before < std::chrono::seconds(1));
    robot.CancelMapping();
    before = Clock::now(); robot.Shutdown();
    Until([&] { return closed == 2; }); // Producers continue calling until the closed ingress rejects them.
    stop = true; odom_input.join(); scan_input.join();
    Check(Clock::now() - before < std::chrono::seconds(1));
    Check(robot.data_flow_status().compute_retained == 0);
    Check(::rmdir(directory) == 0);
}
