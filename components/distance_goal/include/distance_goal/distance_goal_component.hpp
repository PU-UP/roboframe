#pragma once
#include <distance_goal/version.hpp>
#include <roboframe/core/component.hpp>
#include <roboframe/core/server.hpp>
#include <functional>
#include <mutex>
namespace distance_goal {
struct Goal {
    double meters = 0; // Absolute distance since Activate, not relative to goal submission.
    std::chrono::milliseconds timeout{2000};
};
struct Progress { bool active = false; double meters = 0; double target = 0; };
// Lifecycle has one control owner; UpdateDistance and server goal ports are thread-safe.
// Output runs on the Server worker, must be short, and must not reenter lifecycle.
// finish always attempts an inactive/zero output, including cancellation before start.
class DistanceGoalComponent final : public roboframe::Component {
public:
    using Execution = roboframe::Server<Goal, Progress, double>;
    using Output = std::function<void(Progress)>;
    explicit DistanceGoalComponent(Output output);
    ~DistanceGoalComponent() override;
    void Activate() override;
    void RequestDeactivate();
    void Deactivate() override;
    roboframe::Lifecycle lifecycle() const override;
    void UpdateDistance(double meters);
    Execution& server() { return execution_; }
private:
    void Start(const Goal&);
    Execution::Step Step(const Goal&, const Execution::Context&);
    const Output output_;
    mutable std::mutex mutex_;
    double meters_ = 0;
    bool active_ = false;
    Execution::Clock::time_point deadline_{};
    Execution execution_;
};
}
