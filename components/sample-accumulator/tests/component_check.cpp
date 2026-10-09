#include "sample_accumulator/component.hpp"
#include <iostream>
#include <roboframe/log.hpp>
#include <limits>
#include <stdexcept>
#include <vector>

using namespace sample_accumulator;
using roboframe::TaskState;
void check(bool ok) {
    if (!ok) throw std::runtime_error("component check failed");
}
template<class F> void rejects(F fn) {
    bool rejected = false;
    try { fn(); } catch (const std::logic_error&) { rejected = true; }
    check(rejected);
}
int main() {
    const auto info = GetComponentInfo();
    roboframe::CheckCompatibility(info, contract_version);
    check(std::string(info.name) == "sample-accumulator");
    check(std::string(info.implementation_version).size() > 0);
    int component_logs = 0;
    roboframe::SetLogSink([&](roboframe::LogLevel, const std::string& module, const std::string&) {
        if (module == "component") ++component_logs;
    });
    Component component;
    std::vector<Feedback> feedback;
    std::vector<Result> results;
    component.SetCallbacks([&](const Feedback& f) { feedback.push_back(f); },
                           [&](const Result& r) { results.push_back(r); });
    rejects([&] { component.Submit({1, 2}); });
    roboframe::Component& lifecycle = component;
    lifecycle.Activate();
    check(component_logs == 1 && lifecycle.lifecycle() == roboframe::Lifecycle::active);
    rejects([&] { component.Submit({1, 0}); });
    rejects([&] { component.Submit({0, 2}); });
    component.Submit({1, 2});
    rejects([&] { component.Submit({2, 2}); });
    rejects([&] { component.Deactivate(); });
    rejects([&] { component.Cancel(2); });
    component.PushSample({3});
    check(results.empty() && feedback.back().count == 1);
    component.PushSample({5});
    check(results.size() == 1 && results.back().state == TaskState::succeeded);
    check(results.back().sum == 8 && results.back().goal_id == 1);
    rejects([&] { component.PushSample({9}); });
    rejects([&] { component.Cancel(1); });
    rejects([&] { component.Submit({1, 2}); });
    check(results.size() == 1 && feedback.size() == 2);

    component.Submit({2, 3});
    component.PushSample({-2});
    component.Cancel(2);
    check(results.size() == 2 && results.back().state == TaskState::canceled);
    check(results.back().count == 1 && results.back().sum == -2);
    component.Deactivate();
    rejects([&] { component.PushSample({3}); });
    component.Activate();
    rejects([&] { component.Submit({2, 1}); });

    roboframe::GoalId id = 3;
    for (auto edge : {std::numeric_limits<std::int64_t>::max(),
                      std::numeric_limits<std::int64_t>::min()}) {
        component.Submit({id++, 2});
        component.PushSample({edge});
        const auto before = feedback.size();
        component.PushSample({edge > 0 ? 1 : -1});
        check(results.back().state == TaskState::failed && results.back().error == Error::overflow);
        check(results.back().count == 1 && results.back().sum == edge);
        check(feedback.size() == before && component.lifecycle() == roboframe::Lifecycle::active);
    }
    component.Submit({id, 1});
    component.PushSample({7});
    check(results.back().sum == 7 && results.back().state == TaskState::succeeded);
    check(results.size() == 5);
    component.Deactivate();

    roboframe::Task task;
    rejects([&] { task.Finish(TaskState::succeeded); });
    task.Begin(1);
    rejects([&] { task.Finish(TaskState::canceled); });
    task.RequestCancel(1);
    rejects([&] { task.Finish(TaskState::succeeded); });
    task.Finish(TaskState::canceled);
    rejects([&] { task.Finish(TaskState::canceled); });
    roboframe::SetLogSink({});
    std::cout << "PASS: lifecycle, completion, cancellation, busy/stale rejection, overflow/reuse, task transitions\n";
}
