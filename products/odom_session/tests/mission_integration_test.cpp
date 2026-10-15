#include <odom_session/mission_integration.hpp>
#include "interface/stdio_channel.hpp"
#include <condition_variable>
#include <algorithm>
#include <poll.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>
using odom_session::MissionIntegration;
using odom_session::MissionState;
void Check(bool ok) { if (!ok) throw std::runtime_error("mission check failed"); }
template<class Predicate> void Until(MissionIntegration& mission, Predicate done) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!done()) {
        mission.Poll();
        if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("mission timeout");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}
void Send(MissionIntegration& mission, roboframe::TimedPose2D pose) {
    Until(mission, [&] { return mission.Push(pose) == roboframe::Admission::accepted; });
}
int main() {
    MissionIntegration mission;
    mission.Start();
    mission.Begin(5, 6);
    Send(mission, {1, {0, 0, 0}});
    Send(mission, {2, {3, 4, 0}});
    Until(mission, [&] { return mission.completed_steps() == 1; });
    Check(mission.state() == MissionState::running && mission.view().meters == 5);
    Send(mission, {3, {3, 5, 0}});
    Until(mission, [&] { return mission.state() == MissionState::succeeded; });
    Check(mission.completed_steps() == 2 && mission.view().count == 3 && mission.view().meters == 6);
    mission.Stop();
    mission.Start();
    Check(mission.view().count == 0);
    unsigned sent = 0;
    while (mission.Push({sent, {double(sent), 0, 0}}) == roboframe::Admission::accepted) ++sent;
    Check(sent == 32); // Completed-but-unread samples also hold admission credits.
    Until(mission, [&] { return mission.pending() == 0; });
    Check(mission.view().count == sent && mission.view().meters == sent - 1);
    Send(mission, {sent, {double(sent), 0, 0}});
    Until(mission, [&] { return mission.pending() == 0; });
    Check(mission.view().count == sent + 1 && mission.view().meters == sent);
    mission.Push({99, {1000, 0, 0}});
    mission.Stop(); // Abandon a queued/completed sample, then ensure it cannot leak into a new session.
    mission.Start();
    Send(mission, {0, {100, 100, 0}});
    Until(mission, [&] { return mission.pending() == 0; });
    Check(mission.view().count == 1 && mission.view().meters == 0);
    mission.Begin(1, 2, std::chrono::milliseconds(30));
    Until(mission, [&] { return mission.state() == MissionState::failed; });
    Check(mission.completed_steps() == 0 && mission.runtime_status().state == roboframe::RuntimeState::error);
    mission.Reset();
    mission.Start();
    mission.Begin(1, 2);
    mission.Poll();
    mission.Cancel();
    Until(mission, [&] { return mission.state() == MissionState::canceled; });
    mission.Begin(1, 2);
    Send(mission, {0, {0, 0, 0}});
    Send(mission, {1, {2, 0, 0}});
    Until(mission, [&] { return mission.state() == MissionState::succeeded; });
    mission.Stop();

    // Cancellation is a request: a new task must wait for a blocked output callback to finish.
    std::mutex mutex;
    std::condition_variable changed;
    bool entered = false, release = false;
    MissionIntegration held([&](distance_goal::Progress progress) {
        if (!progress.active) return;
        std::unique_lock<std::mutex> lock(mutex);
        entered = true;
        changed.notify_all();
        changed.wait(lock, [&] { return release; });
    });
    held.Start(); held.Begin(1, 2); held.Poll();
    {
        std::unique_lock<std::mutex> lock(mutex);
        Check(changed.wait_for(lock, std::chrono::seconds(1), [&] { return entered; }));
    }
    held.Cancel();
    bool rejected = false;
    try { held.Begin(1, 2); } catch (const std::logic_error&) { rejected = true; }
    { std::lock_guard<std::mutex> lock(mutex); release = true; changed.notify_all(); }
    Check(rejected);
    Until(held, [&] { return held.state() == MissionState::canceled; });
    held.Stop();

    MissionIntegration broken_finish([](distance_goal::Progress progress) {
        if (!progress.active) throw std::runtime_error("finish output failed");
    });
    broken_finish.Start(); broken_finish.Begin(1, 2); broken_finish.Poll();
    broken_finish.Cancel();
    Until(broken_finish, [&] { return broken_finish.state() != MissionState::canceling; });
    if (broken_finish.state() != MissionState::failed)
        throw std::runtime_error("finish failure reported as clean cancellation");
    Check(broken_finish.runtime_status().state == roboframe::RuntimeState::error);
    broken_finish.Stop();

    // Observe real pipe output while this control thread does NOT call Poll.
    int input[2], output[2];
    Check(::pipe(input) == 0 && ::pipe(output) == 0);
    {
        roboframe::StdioChannel channel(input[0], output[1]);
        MissionIntegration independent([&](distance_goal::Progress progress) {
            channel.Queue(progress.active ? "active" : "inactive");
        });
        independent.Start(); independent.Begin(100, 200); independent.Poll();
        Send(independent, {0, {0, 0, 0}});
        Until(independent, [&] { return independent.pending() == 0; });
        Send(independent, {1, {3, 4, 0}});
        std::string lines;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        auto read_line = [&] {
            pollfd fd{output[0], POLLIN, 0};
            Check(::poll(&fd, 1, 50) >= 0);
            if (fd.revents & POLLIN) {
                char buffer[1024];
                const auto count = ::read(output[0], buffer, sizeof(buffer));
                Check(count > 0);
                lines.append(buffer, std::size_t(count));
            }
            Check(std::chrono::steady_clock::now() < deadline);
        };
        while (std::count(lines.begin(), lines.end(), '\n') < 4) read_line();
        Check(independent.view().meters == 0 && independent.pending() == 1);
        independent.Poll();
        Check(independent.view().meters == 5);
        independent.Cancel();
        Until(independent, [&] { return independent.state() == MissionState::canceled; });
        independent.Stop();
        channel.Drain();
        while (lines.find("inactive\n") == std::string::npos) read_line();
        Check(lines.size() >= 9 && lines.substr(lines.size() - 9) == "inactive\n");
    }
    for (int fd : {input[0], input[1], output[0], output[1]}) ::close(fd);
}
