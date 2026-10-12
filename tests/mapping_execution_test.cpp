#include "integration/robot_integration.hpp"
#include <thread>
#include <vector>
#include <stdexcept>
#include <unistd.h>
using namespace roboframe;
using Clock = std::chrono::steady_clock;
void Check(bool ok) { if (!ok) throw std::runtime_error("execution check failed"); }
template<class F> void Until(F done) {
    const auto deadline = Clock::now() + std::chrono::seconds(5);
    while (!done()) { Check(Clock::now() < deadline); std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
}
Scan2DPtr Scan(std::int64_t stamp) { return std::make_shared<const Scan2D>(Scan2D{stamp, {{2, 0}}}); }
int main() {
    char directory[] = "/tmp/roboframe-execution-XXXXXX";
    Check(::mkdtemp(directory) != nullptr);
    {
        RobotIntegration robot(directory, std::chrono::milliseconds(10));
        const auto owner = std::this_thread::get_id();
        std::size_t maps = 0, odoms = 0, failures = 0;
        ProductOutputs outputs;
        outputs.odometry = [&](const TimedPose2D&) { Check(std::this_thread::get_id() == owner); ++odoms; };
        outputs.map = [&](const MapView&) { Check(std::this_thread::get_id() == owner); ++maps; };
        outputs.mapping = [&](const MappingEvent& event) {
            Check(std::this_thread::get_id() == owner);
            if (event.kind == MappingEventKind::failed) ++failures;
        };
        outputs.transforms = [&](const TransformStatus2D&) { Check(std::this_thread::get_id() == owner); };
        robot.SetOutputs(outputs);
        robot.Configure({{20, 20, 1, -5, -5}, 100});
        robot.StartMapping(1);
        std::vector<std::weak_ptr<const Scan2D>> inputs;
        for (int i = 0; i < 1000; ++i) {
            auto scan = Scan(i); inputs.push_back(scan);
            robot.PushScan(scan); // Matching odometry triggers admission, never computation.
            robot.PushOdometry({i, {}});
        }
        Until([&] { return robot.data_flow_status().compute_results == 8; });
        const auto pressure = robot.data_flow_status();
        Check(pressure.compute_retained == 8 && pressure.compute_peak_retained == 8);
        Check(pressure.compute_rejected == 992 && odoms == 1000 && maps == 0);
        std::size_t retained = 0;
        for (const auto& input : inputs) if (!input.expired()) ++retained;
        Check(retained == 8); // Rejected payloads are actually released, not just omitted from counters.
        Check(robot.mapping_status().integrated == 0); // Completion is not owner-side publication.
        robot.CancelMapping();
        Check(robot.mapping_status().drop_cancel == 8);
        robot.StartMapping(2);
        robot.Poll(); // Already-completed old results must not reach the new session.
        Check(robot.data_flow_status().stale_results == 8 && maps == 0);
        for (const auto& input : inputs) Check(input.expired());
        Check(robot.mapping_status().integrated == 0 && robot.pending() == 0);
        robot.PushOdometry({1000, {3, 0, 0}}); robot.PushScan(Scan(1000));
        robot.FinishMapping();
        Check(robot.mapping_status().phase == MappingPhase::finishing);
        Until([&] { robot.Poll(); return robot.mapping_status().phase == MappingPhase::ready; });
        Check(robot.map()->revision == 1 && robot.map()->cells[5 * 20 + 10] == 0);
        Check(robot.map()->cells[5 * 20 + 7] == 205); // Previous session's hit never leaked.
        Check(robot.mapping_status().integrated == 1);
        robot.StartMapping(3);
        robot.PushOdometry({1001, {100, 0, 0}}); robot.PushScan(Scan(1001));
        robot.FinishMapping();
        Until([&] { robot.Poll(); return robot.mapping_status().phase == MappingPhase::idle; });
        Check(failures == 1 && !robot.map() && robot.mapping_status().drop_bounds == 1);
        robot.Shutdown(); robot.Shutdown();
    }
    {
        RobotIntegration robot(directory, std::chrono::milliseconds(5000));
        robot.Configure({{20, 20, 1, -5, -5}, 100}); robot.StartMapping(1);
        robot.PushScan(Scan(0));
        auto start = Clock::now(); robot.PushOdometry({0, {}});
        Check(Clock::now() - start < std::chrono::milliseconds(500));
        Until([&] { return robot.data_flow_status().compute_active == 1; });
        start = Clock::now(); robot.CancelMapping(); robot.StartMapping(2); robot.Poll();
        Check(Clock::now() - start < std::chrono::milliseconds(500));
        robot.PushOdometry({1, {}}); robot.PushScan(Scan(1));
        Until([&] { return robot.data_flow_status().compute_active == 1; });
        start = Clock::now(); robot.Shutdown();
        Check(Clock::now() - start < std::chrono::milliseconds(500));
        Check(robot.data_flow_status().compute_retained == 0);
    }
    Check(::rmdir(directory) == 0);
}
