#include <odom_session/session_integration.hpp>
#include <limits>
#include <stdexcept>

void Check(bool ok) { if (!ok) throw std::runtime_error("session check failed"); }
template<class F> void Reject(F call) {
    bool caught = false;
    try { call(); } catch (const std::exception&) { caught = true; }
    Check(caught);
}

int main() {
    odom_session::SessionIntegration session;
    roboframe::TimedPose2D kept{};
    Check(session.runtime_status().state == roboframe::RuntimeState::idle);
    Check(session.view().valid_samples == 0 && !session.view().has_pose);
    Reject([&] { session.Push({10, {1, 2, 0.3}}); });
    session.Stop();
    Check(!session.view().has_pose);

    session.Start();
    Check(session.runtime_status().state == roboframe::RuntimeState::running);
    Reject([&] { session.Start(); });
    Reject([&] { session.Push({12, {std::numeric_limits<double>::quiet_NaN(), 0, 0}}); });
    Check(session.view().valid_samples == 0 && !session.view().has_pose);

    session.Push({10, {1, 2, 0.3}});
    session.Push({9, {1, 2, 0.3}});
    session.Push({11, {4, 6, -0.4}});
    const auto live = session.view();
    Check(live.valid_samples == 3 && live.has_pose && live.distance_m == 5);
    Check(live.latest.stamp_us == 11 && live.latest.pose.x == 4 && live.latest.pose.y == 6);
    Check(live.latest.pose.yaw == -0.4);
    Reject([&] { session.Push({13, {1e6 + 1, 0, 0}}); });
    Check(session.view().valid_samples == 3 && session.view().latest.stamp_us == 11);

    session.Stop();
    Check(session.runtime_status().state == roboframe::RuntimeState::idle);
    const auto stopped = session.view();
    Check(stopped.valid_samples == 3 && stopped.has_pose && stopped.latest.pose.x == 4);
    Check(stopped.distance_m == 5);
    Reject([&] { session.Push({14, {8, 8, 0}}); });
    Check(session.view().valid_samples == 3 && session.view().latest.pose.x == 4);
    Check(session.view().distance_m == 5);
    session.Stop();
    Check(session.view().valid_samples == 3 && session.view().latest.stamp_us == 11);

    session.Reset();
    Check(session.view().valid_samples == 0 && !session.view().has_pose && session.view().distance_m == 0);
    session.Start();
    Check(session.view().valid_samples == 0 && !session.view().has_pose && session.view().distance_m == 0);
    session.Push({0, {}});
    Check(session.view().valid_samples == 1 && session.view().has_pose);
    Check(session.view().latest.stamp_us == 0 && session.view().latest.pose.x == 0);
    kept = session.view().latest;
    session.Stop();
    Check(session.view().valid_samples == 1 && session.view().latest.stamp_us == kept.stamp_us);

    session.Start();
    Check(session.view().valid_samples == 0 && !session.view().has_pose && session.view().distance_m == 0);
    session.Push({4, {-2, 0.5, 1}});
    Check(session.view().valid_samples == 1 && session.view().latest.pose.x == -2);
    Check(session.view().distance_m == 0);
    session.Stop();
    Check(session.view().valid_samples == 1 && session.view().latest.pose.y == 0.5);
    Check(session.view().distance_m == 0);

    session.Start();
    session.Push({20, {0, 0, 0}});
    session.Push({21, {0, 0, 1.2}});
    session.Push({19, {3, 4, 0.7}});
    Check(session.view().valid_samples == 3 && session.view().distance_m == 5);
    Check(session.view().latest.stamp_us == 19 && session.view().latest.pose.yaw == 0.7);
    session.Stop();
    Check(session.view().distance_m == 5 && session.view().latest.pose.x == 3);
    Reject([&] { session.Push({22, {100, 100, 0}}); });
    Check(session.view().distance_m == 5 && session.view().valid_samples == 3);
    session.Start();
    Check(session.view().valid_samples == 0 && session.view().distance_m == 0 && !session.view().has_pose);
    session.Push({30, {0, 0, 0}});
    Check(session.view().valid_samples == 1 && session.view().distance_m == 0);
    session.Push({31, {0, 1, 9}});
    Check(session.view().valid_samples == 2 && session.view().latest.pose.y == 1);
    Check(session.view().distance_m == 1 && session.view().latest.pose.yaw == 9);
    session.Stop();
    Check(session.view().valid_samples == 2 && session.view().distance_m == 1);
}
