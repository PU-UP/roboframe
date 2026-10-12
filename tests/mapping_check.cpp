#include "integration/robot_integration.hpp"
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>
void Check(bool ok) { if (!ok) throw std::runtime_error("integration check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
roboframe::Scan2DPtr Scan(std::int64_t t) {
    return std::make_shared<const roboframe::Scan2D>(roboframe::Scan2D{t, {{2, 0}}});
}
void Drain(roboframe::RobotIntegration& integration) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    do {
        integration.Poll();
        if (!integration.data_flow_status().compute_retained) { integration.Poll(); return; }
        Check(std::chrono::steady_clock::now() < until);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    } while (true);
}
int main() {
    char directory[] = "/tmp/roboframe-mapping-XXXXXX";
    Check(::mkdtemp(directory) != nullptr);
    const auto late_directory = std::string(directory) + "/late";
    {
        roboframe::RobotIntegration retry(late_directory);
        Reject([&] { retry.Configure({{20, 20, 1, -5, -5}, 100}); });
        Check(::mkdir(late_directory.c_str(), 0700) == 0);
        retry.Configure({{20, 20, 1, -5, -5}, 100});
        retry.StartMapping(1);
    } // Product ownership stops components even without explicit Shutdown.
    Check(::rmdir(late_directory.c_str()) == 0);
    roboframe::RobotIntegration integration(directory);
    Reject([&] { integration.StartMapping(1); });
    integration.Shutdown(); // No components exist before configuration.
    Reject([&] { integration.Configure({{0, 20, 1, -5, -5}, 100}); });
    std::size_t emitted_odom = 0, emitted_maps = 0, emitted_events = 0;
    roboframe::GridSnapshot received_map;
    roboframe::ProductOutputs outputs;
    outputs.odometry = [&](const roboframe::TimedPose2D&) { ++emitted_odom; };
    outputs.map = [&](const roboframe::MapView& value) {
        Check(value.generation > 0); received_map = value.grid; ++emitted_maps;
    };
    outputs.mapping = [&](const roboframe::MappingEvent&) { ++emitted_events; };
    integration.SetOutputs(std::move(outputs));
    integration.Configure({{20, 20, 1, -5, -5}, 100});
    Reject([&] { integration.Configure({{20, 20, 1, -5, -5}, 100}); });
    integration.PushScan(Scan(0)); // Data outside a session is ignored, not retroactively mapped.
    Check(integration.mapping_status().ignored == 1);
    integration.PushOdometry({0, {}});
    integration.Poll();
    Check(emitted_odom == 1);
    integration.StartMapping(1);
    Reject([&] { integration.StartMapping(2); });
    Reject([&] { integration.SaveMap("before_finish"); });
    integration.PushScan(Scan(50));
    Check(integration.pending() == 1 && integration.mapping_status().integrated == 0);
    integration.PushOdometry({100, {2, 0, 0}});
    Drain(integration);
    Check(integration.pending() == 0 && integration.mapping_status().integrated == 1);
    integration.FinishMapping();
    Drain(integration);
    const auto frozen = integration.map();
    Check(received_map == integration.live_map() && emitted_maps > 0 && emitted_events > 0);
    Check(frozen->cells[5 * 20 + 8] == 0); // pose at t50 x=1, hit x=3; not latest x=2.
    integration.SaveMap("first");
    Reject([&] { integration.SaveMap("first"); });
    integration.PushOdometry({200, {3, 0, 0}});
    integration.PushScan(Scan(150));
    Drain(integration);
    Check(integration.map() == frozen && integration.mapping_status().ignored == 0);
    for (;;) {
        try { integration.WaitPlanner(); break; }
        catch (const std::logic_error&) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
    }
    Check(integration.live_map()->revision > frozen->revision);
    Check(integration.data_flow_status().live_updates == 1);
    Check(!integration.data_flow_status().planner_inconsistent);
    integration.PushOdometry({300, {100, 0, 0}});
    integration.PushScan(Scan(300));
    Drain(integration);
    Check(integration.data_flow_status().live_dropped == 1);
    Check(integration.mapping_status().dropped == 0);
    const auto task_received = integration.mapping_status().received;
    integration.PushScan(Scan(450));
    integration.PushOdometry({600, {}}); // Timestamp bracket exceeds 100 us: a live input drop.
    Check(integration.data_flow_status().live_dropped == 2);
    Check(integration.mapping_status().dropped == 0 && integration.mapping_status().received == task_received);
    integration.SaveMap("second");
    integration.LoadMap("first");
    Check(integration.map()->cells == frozen->cells);
    Reject([&] { integration.LoadMap("missing"); });
    Check(integration.map()->cells == frozen->cells);
    integration.StartMapping(2);
    Reject([&] { integration.FinishMapping(); });
    Check(integration.mapping_status().received == 0 && integration.mapping_status().dropped == 0);
    Check(integration.data_flow_status().live_dropped == 2); // Preserved across data context resets.
    integration.PushScan(Scan(650));
    integration.PushOdometry({900, {4, 0, 0}});
    Check(integration.mapping_status().dropped == 1);
    for (int i = 0; i < 65; ++i) integration.PushScan(Scan(1000 + i));
    Check(integration.pending() == 64 && integration.mapping_status().dropped == 2);
    integration.CancelMapping();
    Check(integration.pending() == 0 && integration.mapping_status().dropped == 66);
    integration.StartMapping(3);
    integration.PushOdometry({1100, {4, 0, 0}});
    integration.PushScan(Scan(1100));
    integration.PushScan(Scan(1200));
    integration.FinishMapping();
    Drain(integration);
    Check(integration.mapping_status().integrated == 1 && integration.mapping_status().dropped == 1);
    Check(frozen->revision == 1); // Old snapshot survives new sessions.
    integration.SetOutputs({});
    integration.Shutdown();
    integration.Shutdown();
    Reject([&] { integration.StartMapping(4); });
    for (auto name : {"first", "second"}) std::remove((std::string(directory) + "/" + name + ".rfgrid").c_str());
    ::rmdir(directory);
}
