#include "integration/data_flow/sensor_flow.hpp"
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("sensor flow check failed"); }
template<class F> void Reject(F f) {
    bool rejected = false;
    try { f(); } catch (const std::logic_error&) { rejected = true; }
    Check(rejected);
}
Scan2DPtr Scan(std::int64_t t) { return std::make_shared<const Scan2D>(Scan2D{t, {{2, 0}}}); }
int main() {
    Scan2DPtr delivered;
    std::uint64_t generation = 0;
    SensorFlow flow({GridFrame::odom, 100, 1000},
        [&](std::uint64_t id, Scan2DPtr scan, ScanTransforms2D tf) {
            Check(tf.odom_from_base.stamp_us == scan->stamp_us);
            generation = id; delivered = std::move(scan); return Admission::accepted;
        });
    Reject([&] { flow.Resume(); });
    Reject([&] { flow.Open(0); });
    flow.PushOdometry({0, {}});
    flow.PushScan(Scan(0));
    Check(flow.status().counts.ignored == 1);
    auto opened = flow.Open(7);
    Check(opened.counts.ignored == 1 && opened.submitted == 0);
    const auto scan = Scan(50);
    flow.PushScan(scan);
    Check(!delivered && flow.status().pending == 1);
    flow.PushOdometry({100, {2, 0, 0}});
    Check(delivered == scan && generation == 7);
    flow.PushScan(Scan(200));
    const auto paused = flow.Pause();
    Check(paused.pending == 0 && paused.counts.drop_sync == 1 && paused.submitted == 1);
    flow.PushScan(Scan(201));
    flow.PushOdometry({300, {}});
    Check(flow.TakeEvents().odom.stamp_us == 300); // Pausing scan delivery never stops odometry.
    Check(flow.status().counts.ignored == 2 && flow.status().submitted == 1);
    flow.Resume();
    flow.PushScan(Scan(300));
    Check(flow.status().submitted == 2 && generation == 7);
    flow.Close();
    Check(!flow.status().transforms.valid);
    Reject([&] { flow.Resume(); });
    opened = flow.Open(8);
    Check(opened.counts.received == 3 && opened.submitted == 0);
    flow.PushOdometry({400, {}}); flow.PushScan(Scan(400));
    Check(generation == 8 && flow.status().counts.received == 4);
    flow.Shutdown();
    Reject([&] { flow.Open(9); });
    Reject([&] { flow.PushOdometry({500, {}}); });
}
