#include "integration/mapping.hpp"
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <unistd.h>
void Check(bool ok) { if (!ok) throw std::runtime_error("integration check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
occupancy_grid::ScanPtr Scan(std::int64_t t) {
    return std::make_shared<const occupancy_grid::Scan>(occupancy_grid::Scan{t, {{2, 0}}});
}
int main() {
    char directory[] = "/tmp/roboframe-mapping-XXXXXX";
    Check(::mkdtemp(directory) != nullptr);
    roboframe::MappingIntegration integration({20, 20, 1, -5, -5}, directory, {512, 100});
    integration.PushScan(Scan(0)); // Data outside a session is ignored, not retroactively mapped.
    Check(integration.status().ignored == 1);
    integration.PushOdometry({0, {}});
    integration.Begin(1);
    Reject([&] { integration.Begin(2); });
    Reject([&] { integration.Save("before_finish"); });
    integration.PushScan(Scan(50));
    Check(integration.pending() == 1 && integration.status().integrated == 0);
    integration.PushOdometry({100, {2, 0, 0}});
    Check(integration.pending() == 0 && integration.status().integrated == 1);
    integration.Finish();
    const auto frozen = integration.map();
    Check(frozen->cells[5 * 20 + 8] == 0); // pose at t50 x=1, hit x=3; not latest x=2.
    integration.Save("first");
    Reject([&] { integration.Save("first"); });
    integration.PushOdometry({200, {3, 0, 0}});
    integration.PushScan(Scan(150));
    Check(integration.map() == frozen && integration.status().ignored == 1);
    integration.Save("second");
    integration.Load("first");
    Check(integration.map()->cells == frozen->cells);
    Reject([&] { integration.Load("missing"); });
    Check(integration.map()->cells == frozen->cells);
    integration.Begin(2);
    Reject([&] { integration.Finish(); });
    integration.PushScan(Scan(250));
    integration.PushOdometry({400, {4, 0, 0}});
    Check(integration.status().dropped == 1);
    for (int i = 0; i < 65; ++i) integration.PushScan(Scan(1000 + i));
    Check(integration.pending() == 64 && integration.status().dropped == 2);
    integration.Cancel();
    Check(integration.pending() == 0 && integration.status().dropped == 66);
    integration.Begin(3);
    integration.PushOdometry({1100, {4, 0, 0}});
    integration.PushScan(Scan(1100));
    integration.PushScan(Scan(1200));
    integration.Finish();
    Check(integration.status().integrated == 1 && integration.status().dropped == 1);
    Check(frozen->revision == 1); // Old snapshot survives new sessions.
    integration.Shutdown();
    integration.Shutdown();
    Reject([&] { integration.Begin(4); });
    for (auto name : {"first", "second"}) std::remove((std::string(directory) + "/" + name + ".rfgrid").c_str());
    ::rmdir(directory);
}
