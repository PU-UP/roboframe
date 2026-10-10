#include "integration/robot_integration.hpp"
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>
void Check(bool ok) { if (!ok) throw std::runtime_error("integration check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
roboframe::Scan2DPtr Scan(std::int64_t t) {
    return std::make_shared<const roboframe::Scan2D>(roboframe::Scan2D{t, {{2, 0}}});
}
int main() {
    char directory[] = "/tmp/roboframe-mapping-XXXXXX";
    Check(::mkdtemp(directory) != nullptr);
    roboframe::RobotIntegration integration({{20, 20, 1, -5, -5}, 100}, directory);
    integration.PushScan(Scan(0)); // Data outside a session is ignored, not retroactively mapped.
    Check(integration.mapping_status().ignored == 1);
    integration.PushOdometry({0, {}});
    integration.StartMapping(1);
    Reject([&] { integration.StartMapping(2); });
    Reject([&] { integration.SaveMap("before_finish"); });
    integration.PushScan(Scan(50));
    Check(integration.pending() == 1 && integration.mapping_status().integrated == 0);
    integration.PushOdometry({100, {2, 0, 0}});
    Check(integration.pending() == 0 && integration.mapping_status().integrated == 1);
    integration.FinishMapping();
    const auto frozen = integration.map();
    Check(frozen->cells[5 * 20 + 8] == 0); // pose at t50 x=1, hit x=3; not latest x=2.
    integration.SaveMap("first");
    Reject([&] { integration.SaveMap("first"); });
    integration.PushOdometry({200, {3, 0, 0}});
    integration.PushScan(Scan(150));
    Check(integration.map() == frozen && integration.mapping_status().ignored == 1);
    integration.SaveMap("second");
    integration.LoadMap("first");
    Check(integration.map()->cells == frozen->cells);
    Reject([&] { integration.LoadMap("missing"); });
    Check(integration.map()->cells == frozen->cells);
    integration.StartMapping(2);
    Reject([&] { integration.FinishMapping(); });
    integration.PushScan(Scan(250));
    integration.PushOdometry({400, {4, 0, 0}});
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
    Check(integration.mapping_status().integrated == 1 && integration.mapping_status().dropped == 1);
    Check(frozen->revision == 1); // Old snapshot survives new sessions.
    integration.Shutdown();
    integration.Shutdown();
    Reject([&] { integration.StartMapping(4); });
    for (auto name : {"first", "second"}) std::remove((std::string(directory) + "/" + name + ".rfgrid").c_str());
    ::rmdir(directory);
}
