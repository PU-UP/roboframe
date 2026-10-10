#include <odom/passthrough_odom_component.hpp>
#include <limits>
#include <stdexcept>
void Check(bool ok) { if (!ok) throw std::runtime_error("odometry check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
int main() {
    int count = 0;
    roboframe::TimedPose2D received;
    odom::PassthroughOdomComponent odometry([&](const roboframe::TimedPose2D& pose) { received = pose; ++count; });
    Reject([&] { odometry.PushOdometry({10, {}}); });
    odometry.Activate();
    odometry.PushOdometry({10, {1, 2, 0.3}});
    Check(count == 1 && received.stamp_us == 10 && received.pose.x == 1 && received.pose.yaw == 0.3);
    Reject([&] { odometry.PushOdometry({10, {}}); });
    Reject([&] { odometry.PushOdometry({11, {std::numeric_limits<double>::quiet_NaN(), 0, 0}}); });
    Check(count == 1);
    odometry.Deactivate();
    Reject([&] { odometry.PushOdometry({12, {}}); });
    odometry.Activate();
    odometry.PushOdometry({0, {}});
    Check(count == 2);
    Reject([] { odom::PassthroughOdomComponent missing({}); });
}
