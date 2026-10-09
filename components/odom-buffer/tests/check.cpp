#include <odom_buffer/component.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
void Check(bool ok) { if (!ok) throw std::runtime_error("odom check failed"); }
template<class F> void Reject(F f) {
    bool caught = false; try { f(); } catch (const std::exception&) { caught = true; } Check(caught);
}
int main() {
    odom_buffer::Component odometry({2, 100});
    roboframe::TimedPose2D result;
    Reject([&] { odometry.PushOdometry({0, {}}); });
    odometry.Activate();
    Check(odometry.At(0, result) == odom_buffer::Match::waiting);
    odometry.PushOdometry({10, {0, 0, 3.1}});
    odometry.PushOdometry({110, {10, 2, -3.1}});
    Check(odometry.At(60, result) == odom_buffer::Match::ready);
    Check(result.stamp_us == 60 && result.pose.x == 5 && result.pose.y == 1);
    Check(std::abs(std::abs(result.pose.yaw) - 3.141592653589793) < 1e-9);
    Check(odometry.At(110, result) == odom_buffer::Match::ready && result.pose.x == 10);
    Check(odometry.At(111, result) == odom_buffer::Match::waiting);
    Check(odometry.At(9, result) == odom_buffer::Match::unavailable);
    Reject([&] { odometry.PushOdometry({110, {}}); });
    Reject([&] { odometry.PushOdometry({111, {std::numeric_limits<double>::quiet_NaN(), 0, 0}}); });
    Check(odometry.size() == 2);
    odometry.PushOdometry({300, {20, 2, 0}});
    Check(odometry.At(200, result) == odom_buffer::Match::unavailable);
    Check(odometry.At(10, result) == odom_buffer::Match::unavailable);
    odometry.Deactivate();
    Check(odometry.size() == 0);
    Reject([&] { odometry.At(100, result); });
}
