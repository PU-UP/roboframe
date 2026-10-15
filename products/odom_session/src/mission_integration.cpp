#include <odom_session/mission_integration.hpp>
#include <cmath>
#include <stdexcept>
namespace odom_session {
MissionIntegration::MissionIntegration(distance_goal::DistanceGoalComponent::Output output)
    : goal_(std::move(output)),
      odom_([this](const roboframe::TimedPose2D& pose) {
          if (distance_.Push(pose) != roboframe::Admission::accepted)
              throw std::logic_error("distance admission changed");
      }),
      first_action_(goal_.server(), [this] { return first_; },
          [this](const Execution::Result& r) { if (r.state == roboframe::TaskState::succeeded) ++completed_; }),
      second_action_(goal_.server(), [this] { return second_; },
          [this](const Execution::Result& r) { if (r.state == roboframe::TaskState::succeeded) ++completed_; }),
      tree_({{"first distance", [this] { return first_action_.Tick(); }, [this] { first_action_.Halt(); }},
             {"second distance", [this] { return second_action_.Tick(); }, [this] { second_action_.Halt(); }}}) {}
MissionIntegration::~MissionIntegration() { Shutdown(); }
roboframe::Admission MissionIntegration::Push(roboframe::TimedPose2D pose) {
    RequireRunning();
    if (!inputs_open_) return roboframe::Admission::closed;
    if (!distance_.CanAccept()) return roboframe::Admission::full;
    odom_.Push(pose); // Validate before submitting. A full queue never increments odom's count.
    return roboframe::Admission::accepted;
}
bool MissionIntegration::Drained() {
    bool done = true;
    for (auto* action : {&first_action_, &second_action_}) {
        const auto handle = action->handle();
        if (!handle) continue;
        const auto status = goal_.server().GetStatus(handle).state;
        if (roboframe::ServerBusy(status)) { done = false; continue; }
        // Drained discards the Result, including finish errors. Preserve failure before consuming it.
        if (status == roboframe::TaskState::failed) state_ = MissionState::failed;
        action->Drained();
    }
    return done;
}
void MissionIntegration::Begin(double first, double second, std::chrono::milliseconds timeout) {
    RequireRunning();
    if (state_ == MissionState::running || !Drained()) throw std::logic_error("mission busy");
    if (state_ == MissionState::failed) {
        Fail();
        throw std::logic_error("failed mission needs reset");
    }
    if (!std::isfinite(first) || !std::isfinite(second) || first <= 0 || second <= first || timeout.count() <= 0)
        throw std::invalid_argument("mission needs 0 < first < second and positive timeout");
    tree_.Halt();
    first_action_.Reset();
    second_action_.Reset();
    first_ = {first, timeout};
    second_ = {second, timeout};
    completed_ = 0;
    state_ = MissionState::running;
}
void MissionIntegration::Cancel() noexcept {
    tree_.Halt();
    first_action_.Halt();
    second_action_.Halt();
    if (state_ == MissionState::running) state_ = MissionState::canceling;
}
void MissionIntegration::OnStart(std::uint64_t) {
    view_ = {};
    completed_ = 0;
    state_ = MissionState::idle;
    distance_.Activate();
    goal_.Activate();
    odom_.Activate();
    inputs_open_ = true;
}
void MissionIntegration::CloseInputs() noexcept {
    inputs_open_ = false;
    Cancel();
    distance_.RequestDeactivate();
    goal_.RequestDeactivate();
}
void MissionIntegration::OnStop() noexcept {
    distance_.Deactivate();
    goal_.Deactivate();
    odom_.Deactivate();
    Drained();
    if (state_ == MissionState::canceling) state_ = MissionState::canceled;
}
void MissionIntegration::OnReset() { view_ = {}; state_ = MissionState::idle; completed_ = 0; }
void MissionIntegration::OnPoll() {
    if (runtime_status().state != roboframe::RuntimeState::running) return;
    try {
        path_distance::PathDistanceComponent::Execution::Result result;
        for (unsigned i = 0; i < 32 && distance_.Take(result); ++i) {
            if (result.error) std::rethrow_exception(result.error);
            if (result.canceled) throw std::runtime_error("unexpected canceled distance sample");
            view_ = result.output;
            goal_.UpdateDistance(view_.meters);
        }
        if (state_ == MissionState::running) {
            const auto status = tree_.Tick();
            if (status == roboframe::bt::Status::success) state_ = MissionState::succeeded;
            if (status == roboframe::bt::Status::failure) { state_ = MissionState::failed; Fail(); }
        } else if (state_ == MissionState::canceling && Drained()) {
            if (state_ == MissionState::failed) Fail();
            else state_ = MissionState::canceled;
        }
    } catch (...) {
        state_ = MissionState::failed;
        Fail();
        throw;
    }
}
}
