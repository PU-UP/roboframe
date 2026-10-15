#include <odom_session/session_integration.hpp>
#include <cmath>
#include <stdexcept>

namespace odom_session {
SessionIntegration::SessionIntegration()
    : odom_([this](const roboframe::TimedPose2D& pose) { Retain(pose); }) {}
SessionIntegration::~SessionIntegration() { Shutdown(); }

void SessionIntegration::Push(const roboframe::TimedPose2D& pose) {
    RequireRunning();
    if (!inputs_open_) throw std::logic_error("session input closed");
    odom_.Push(pose);
}
SessionView SessionIntegration::view() const {
    if (runtime_status().state == roboframe::RuntimeState::running) {
        SessionView live;
        live.valid_samples = odom_.accepted();
        live.has_pose = has_pose_;
        live.latest = latest_;
        live.distance_m = distance_m_;
        return live;
    }
    return frozen_;
}
void SessionIntegration::OnStart(std::uint64_t) {
    has_pose_ = false;
    has_segment_ = false;
    latest_ = {};
    segment_ = {};
    distance_m_ = 0;
    frozen_ = {};
    odom_.Activate();
    inputs_open_ = true;
}
void SessionIntegration::CloseInputs() noexcept { inputs_open_ = false; }
void SessionIntegration::OnStop() noexcept {
    frozen_.valid_samples = odom_.accepted();
    frozen_.has_pose = has_pose_;
    frozen_.latest = latest_;
    frozen_.distance_m = distance_m_;
    inputs_open_ = false;
    odom_.Deactivate();
}
void SessionIntegration::OnReset() {
    has_pose_ = false;
    has_segment_ = false;
    latest_ = {};
    segment_ = {};
    distance_m_ = 0;
    frozen_ = {};
    inputs_open_ = false;
}
void SessionIntegration::OnPoll() {}
void SessionIntegration::Retain(const roboframe::TimedPose2D& pose) {
    if (has_segment_) {
        const double dx = pose.pose.x - segment_.pose.x;
        const double dy = pose.pose.y - segment_.pose.y;
        distance_m_ += std::hypot(dx, dy);
    }
    segment_ = pose;
    has_segment_ = true;
    latest_ = pose;
    has_pose_ = true;
}
} // namespace odom_session
