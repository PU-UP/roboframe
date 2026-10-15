#include <distance_goal/distance_goal_component.hpp>
#include <condition_variable>
#include <stdexcept>
#include <thread>
#include <vector>
using distance_goal::DistanceGoalComponent;
void Check(bool ok) { if (!ok) throw std::runtime_error("distance goal check failed"); }
DistanceGoalComponent::Execution::Result Take(DistanceGoalComponent& component, roboframe::ServerHandle handle) {
    DistanceGoalComponent::Execution::Result result;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!component.server().TakeResult(handle, result)) {
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("goal result timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return result;
}
int main() {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<distance_goal::Progress> output;
    bool throw_on_active = false;
    std::thread::id worker;
    DistanceGoalComponent component([&](distance_goal::Progress progress) {
        std::lock_guard<std::mutex> lock(mutex);
        worker = std::this_thread::get_id();
        if (progress.active && throw_on_active) throw std::runtime_error("output failed");
        output.push_back(progress);
        changed.notify_all();
    });
    component.Activate();
    auto handle = component.server().Submit({5, std::chrono::seconds(1)});
    { // No Integration/Poll: a blocked caller still receives periodic worker output.
        std::unique_lock<std::mutex> lock(mutex);
        Check(changed.wait_for(lock, std::chrono::seconds(1), [&] { return output.size() >= 3; }));
        Check(worker != std::this_thread::get_id());
    }
    component.UpdateDistance(5);
    auto result = Take(component, handle);
    Check(result.state == roboframe::TaskState::succeeded && result.value == 5 && !result.error);
    { std::lock_guard<std::mutex> lock(mutex); Check(!output.back().active && output.back().meters == 0); }
    handle = component.server().Submit({10, std::chrono::seconds(1)});
    component.server().RequestCancel(handle);
    result = Take(component, handle);
    Check(result.state == roboframe::TaskState::canceled);
    handle = component.server().Submit({-1, std::chrono::seconds(1)});
    result = Take(component, handle);
    Check(result.state == roboframe::TaskState::failed && bool(result.error)); // start failed; finish still runs.
    handle = component.server().Submit({10, std::chrono::milliseconds(30)});
    result = Take(component, handle);
    Check(result.state == roboframe::TaskState::failed && bool(result.error));
    {
        std::lock_guard<std::mutex> lock(mutex);
        Check(!output.back().active);
        throw_on_active = true;
    }
    handle = component.server().Submit({10, std::chrono::seconds(1)});
    result = Take(component, handle);
    Check(result.state == roboframe::TaskState::failed && bool(result.error));
    {
        std::lock_guard<std::mutex> lock(mutex);
        Check(!output.back().active);
        throw_on_active = false;
    }
    bool rejected = false;
    try { component.UpdateDistance(4); } catch (const std::invalid_argument&) { rejected = true; }
    Check(rejected);
    component.Deactivate();
    component.Activate();
    handle = component.server().Submit({1, std::chrono::seconds(1)});
    component.UpdateDistance(1); // Reactivation cleared the previous cumulative distance.
    Check(Take(component, handle).state == roboframe::TaskState::succeeded);
    handle = component.server().Submit({100, std::chrono::seconds(1)});
    component.RequestDeactivate();
    component.Deactivate();
    Check(Take(component, handle).state == roboframe::TaskState::canceled);
    { std::lock_guard<std::mutex> lock(mutex); Check(!output.back().active); }
}
