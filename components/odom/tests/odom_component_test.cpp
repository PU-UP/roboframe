#include <odom/odom_component.hpp>
#include <limits>
#include <stdexcept>
void Check(bool ok) { if (!ok) throw std::runtime_error("odometry check failed"); }
template<class F> void Reject(F call) {
    bool caught = false;
    try { call(); } catch (const std::exception&) { caught = true; }
    Check(caught);
}
int main() {
    const auto info = odom::Info();
    Check(info.name && info.name[0] && info.source_id && info.source_id[0]);
    Reject([] { odom::OdomComponent missing({}); });

    int calls = 0;
    roboframe::TimedPose2D received{};
    odom::OdomComponent odometry([&](const roboframe::TimedPose2D& pose) {
        received = pose;
        if (++calls == 1) throw std::runtime_error("sink failed");
    });
    Check(odometry.lifecycle() == roboframe::Lifecycle::inactive && odometry.accepted() == 0);
    Reject([&] { odometry.Push({10, {1, 2, 0.3}}); });
    Reject([&] { odometry.Activate(); odometry.Activate(); });

    roboframe::TimedPose2D latest{};
    Reject([&] { odometry.Push({10, {1, 2, 0.3}}); });
    Check(calls == 1 && odometry.accepted() == 1 && odometry.Latest(latest));
    Check(latest.stamp_us == 10 && latest.pose.x == 1 && latest.pose.y == 2 && latest.pose.yaw == 0.3);
    Check(received.stamp_us == 10 && received.pose.x == 1);

    odometry.Push({10, {4, 5, 0.6}});
    Check(calls == 2 && odometry.accepted() == 2 && odometry.Latest(latest) && latest.pose.x == 4);
    odometry.Push({9, {-1, 0, 0}});
    Check(odometry.accepted() == 3 && odometry.Latest(latest) && latest.stamp_us == 9 && latest.pose.x == -1);

    Reject([&] { odometry.Push({12, {std::numeric_limits<double>::quiet_NaN(), 0, 0}}); });
    Reject([&] { odometry.Push({-1, {}}); });
    Reject([&] { odometry.Push({13, {1e6 + 1, 0, 0}}); });
    Check(calls == 3 && odometry.accepted() == 3 && odometry.Latest(latest) && latest.stamp_us == 9);

    odometry.Deactivate();
    Check(odometry.lifecycle() == roboframe::Lifecycle::inactive && !odometry.Latest(latest));
    Check(odometry.accepted() == 3);
    odometry.Deactivate();
    Reject([&] { odometry.Push({14, {}}); });

    odometry.Activate();
    Check(odometry.accepted() == 0 && !odometry.Latest(latest));
    odometry.Push({0, {}});
    Check(calls == 4 && odometry.accepted() == 1 && odometry.Latest(latest) && latest.stamp_us == 0);
    odometry.Deactivate();
}
