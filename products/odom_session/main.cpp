#include <odom_session/session_integration.hpp>
#include <odom_session/mission_integration.hpp>
#include <chrono>
#include <thread>
#include "interface/stdio_channel.hpp"
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>

namespace {
roboframe::TimedPose2D ParseOdom(const std::string& line) {
    double x = 0, y = 0, yaw = 0, tv = 0, rv = 0, accel = 0, seconds = 0, logged = 0;
    char host[128];
    host[0] = '\0';
    const int fields = std::sscanf(line.c_str(), "ODOM %lf %lf %lf %lf %lf %lf %lf %127s %lf",
        &x, &y, &yaw, &tv, &rv, &accel, &seconds, host, &logged);
    if (fields != 9 || host[0] == '\0') throw std::runtime_error("malformed ODOM record");
    (void)tv; (void)rv; (void)accel; (void)logged;
    if (!std::isfinite(seconds) || seconds < 0 || seconds >= 9223372036854.0)
        throw std::runtime_error("ODOM time is outside the microsecond range");
    roboframe::TimedPose2D pose;
    pose.stamp_us = std::llround(seconds * 1e6);
    pose.pose.x = x;
    pose.pose.y = y;
    pose.pose.yaw = yaw;
    return pose;
}

std::string Format(const odom_session::SessionView& view) {
    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(6);
    summary << "valid=" << view.valid_samples;
    if (view.has_pose) {
        summary << " latest_us=" << view.latest.stamp_us
                << " x=" << view.latest.pose.x
                << " y=" << view.latest.pose.y
                << " yaw=" << view.latest.pose.yaw;
    }
    summary << " distance=" << view.distance_m;
    return summary.str();
}

void Run() {
    roboframe::StdioChannel channel(STDIN_FILENO, STDOUT_FILENO);
    odom_session::SessionIntegration session;
    bool open = false;
    auto finish = [&] {
        if (!open) return;
        session.Stop();
        channel.Queue(Format(session.view()));
        open = false;
    };
    auto begin = [&] {
        if (open) throw std::runtime_error("session already started");
        session.Reset();
        session.Start();
        open = true;
    };
    std::string line;
    while (channel.NextLine(line, [&] { session.Poll(); })) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.compare(0, 5, "ODOM ") == 0) {
            if (!open) {
                if (session.runtime_status().generation != 0)
                    throw std::runtime_error("session is not running");
                begin();
            }
            session.Push(ParseOdom(line));
        } else if (line == "SESSION start") {
            begin();
        } else if (line == "SESSION stop") {
            if (!open) throw std::runtime_error("session is not running");
            finish();
        } else if (line == "SESSION restart") {
            finish();
            begin();
        } else if (line.compare(0, 8, "SESSION ") == 0) {
            throw std::runtime_error("unknown session command");
        }
    }
    finish();
    channel.Drain();
}

int RunMission(double first, double second) {
    roboframe::StdioChannel channel(STDIN_FILENO, STDOUT_FILENO);
    odom_session::MissionIntegration mission([&](distance_goal::Progress progress) {
        std::ostringstream line;
        line << "PROGRESS active=" << progress.active << " meters=" << progress.meters
             << " target=" << progress.target;
        channel.Queue(line.str(), "mission-progress");
    });
    try {
        mission.Start();
        mission.Begin(first, second);
        auto wait = [&](const std::function<bool()>& done) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
            while (!done()) {
                mission.Poll();
                if (mission.state() == odom_session::MissionState::failed)
                    throw std::runtime_error("mission failed");
                if (std::chrono::steady_clock::now() >= deadline)
                    throw std::runtime_error("mission drain timeout");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        };
        std::string line;
        while (channel.NextLine(line, [&] {
            mission.Poll();
            if (mission.state() == odom_session::MissionState::failed)
                throw std::runtime_error("mission failed");
        })) {
            if (!line.empty() && line.back() == '\r') line.pop_back();
            if (line.compare(0, 5, "ODOM ") == 0) {
                const auto pose = ParseOdom(line);
                wait([&] { return mission.Push(pose) == roboframe::Admission::accepted; });
            } else if (line == "MISSION cancel") {
                mission.Cancel();
            } else if (line.compare(0, 8, "MISSION ") == 0 || line.compare(0, 8, "SESSION ") == 0) {
                throw std::runtime_error("mission mode accepts only MISSION cancel");
            }
        }
        wait([&] { return mission.pending() == 0; });
        if (mission.state() == odom_session::MissionState::running && mission.view().meters < second)
            mission.Cancel(); // EOF cannot provide the remaining distance.
        wait([&] {
            return mission.state() != odom_session::MissionState::running &&
                   mission.state() != odom_session::MissionState::canceling;
        });
        const bool success = mission.state() == odom_session::MissionState::succeeded;
        mission.Stop(); // Wait for final inactive output before closing the writer.
        const auto& sample = mission.view();
        odom_session::SessionView summary;
        summary.valid_samples = sample.count;
        summary.has_pose = sample.count != 0;
        summary.latest = sample.latest;
        summary.distance_m = sample.meters;
        channel.Queue(std::string("MISSION state=") + (success ? "succeeded" : "canceled") +
                      " steps=" + std::to_string(mission.completed_steps()) + " " + Format(summary));
        channel.Drain();
        return success ? 0 : 2;
    } catch (...) {
        mission.Shutdown();
        channel.Drain(); // Final inactive output must be flushed on input/algorithm errors too.
        throw;
    }
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGPIPE, SIG_IGN);
    try {
        if (argc == 4 && std::string(argv[1]) == "--mission") {
            auto number = [](const char* text) {
                std::size_t used = 0;
                const double value = std::stod(text, &used);
                if (used != std::string(text).size()) throw std::invalid_argument("invalid mission distance");
                return value;
            };
            return RunMission(number(argv[2]), number(argv[3]));
        }
        if (argc != 1) throw std::invalid_argument("usage: odom_session [--mission first_m second_m]");
        Run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
