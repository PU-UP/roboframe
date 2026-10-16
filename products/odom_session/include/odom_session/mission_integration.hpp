#pragma once
#include <odom/odom_component.hpp>
#include <path_distance/path_distance_component.hpp>
#include <distance_goal/distance_goal_component.hpp>
#include <roboframe/core/integration.hpp>
#include <roboframe/orchestration/server_action.hpp>
namespace odom_session {
enum class MissionState { idle, running, canceling, canceled, succeeded, failed };
// Control owner: lifecycle, Push/Poll, Begin/Cancel, view/state.
// Progress callback is on the goal worker; it must not call these control-owner methods.
class MissionIntegration final : public roboframe::Integration {
public:
    using Execution = distance_goal::DistanceGoalComponent::Execution;
    explicit MissionIntegration(distance_goal::DistanceGoalComponent::Output output = {});
    ~MissionIntegration() override;
    roboframe::Admission Push(roboframe::TimedPose2D pose);
    void Begin(double first, double second, std::chrono::milliseconds timeout = std::chrono::seconds(2));
    void Cancel() noexcept;
    MissionState state() const { return state_; }
    std::exception_ptr error() const { return error_; }
    const path_distance::Sample& view() const { return view_; }
    std::size_t pending() const { return distance_.retained(); }
    unsigned completed_steps() const { return completed_; }
private:
    void OnStart(std::uint64_t) override;
    void CloseInputs() noexcept override;
    void OnStop() noexcept override;
    void OnReset() override;
    void OnPoll() override;
    bool Drained();
    void OnResult(const Execution::Result& result) noexcept;
    MissionState state_ = MissionState::idle;
    std::exception_ptr error_;
    bool inputs_open_ = false;
    unsigned completed_ = 0;
    path_distance::Sample view_{};
    distance_goal::Goal first_{}, second_{};
    path_distance::PathDistanceComponent distance_;
    distance_goal::DistanceGoalComponent goal_;
    odom::OdomComponent odom_;
    roboframe::bt::ServerAction<Execution> first_action_, second_action_;
    roboframe::bt::Sequence tree_;
};
}
