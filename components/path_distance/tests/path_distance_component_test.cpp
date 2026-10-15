#include <path_distance/path_distance_component.hpp>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <thread>
using path_distance::PathDistanceComponent;
void Check(bool ok) { if (!ok) throw std::runtime_error("path distance check failed"); }
PathDistanceComponent::Execution::Result Take(PathDistanceComponent& component) {
    PathDistanceComponent::Execution::Result result;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!component.Take(result)) {
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("distance result timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return result;
}
int main() {
    PathDistanceComponent component(2);
    Check(component.Push({}) == roboframe::Admission::closed);
    component.Activate();
    Check(component.Push({10, {0, 0, 0}}) == roboframe::Admission::accepted);
    Check(component.Push({9, {3, 4, 9}}) == roboframe::Admission::accepted);
    Check(!component.CanAccept());
    Check(component.Push({11, {6, 8, 0}}) == roboframe::Admission::full);
    auto a = Take(component), b = Take(component);
    Check(!a.error && !b.error && !a.canceled && a.output.count == 1 && a.output.meters == 0);
    Check(b.output.count == 2 && b.output.meters == 5 && b.output.latest.stamp_us == 9);
    Check(component.Push({11, {6, 8, 0}}) == roboframe::Admission::accepted);
    Check(Take(component).output.meters == 10); // Retry did not lose or duplicate a segment.
    component.Push({12, {std::numeric_limits<double>::quiet_NaN(), 0, 0}});
    Check(bool(Take(component).error));
    component.Push({13, {6, 8, 0}});
    b = Take(component);
    Check(!b.error && b.output.count == 4 && b.output.meters == 10);
    component.Push({14, {9, 12, 0}});
    component.RequestDeactivate();
    Check(component.Push({}) == roboframe::Admission::closed);
    component.Deactivate();
    Check(component.retained() == 0);
    Check(!component.Take(a));
    component.Activate();
    component.Push({0, {100, 100, 0}});
    a = Take(component);
    Check(a.output.count == 1 && a.output.meters == 0 && a.output.latest.pose.x == 100);
    component.Deactivate();
    component.Deactivate();
}
