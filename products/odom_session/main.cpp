#include <odom_session/session_integration.hpp>
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
} // namespace

int main() {
    std::signal(SIGPIPE, SIG_IGN);
    try {
        Run();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
