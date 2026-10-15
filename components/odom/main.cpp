#include <odom/odom_component.hpp>
#include "interface/stdio_channel.hpp"
#include <cmath>
#include <cstdint>
#include <csignal>
#include <exception>
#include <iostream>
#include <cstdio>
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

void Receive() {
    roboframe::StdioChannel channel(STDIN_FILENO, STDOUT_FILENO);
    roboframe::TimedPose2D first{}, last{};
    bool any = false;
    odom::OdomComponent odometry([&](const roboframe::TimedPose2D& pose) {
        if (!any) first = pose;
        last = pose;
        any = true;
    });
    odometry.Activate();
    std::string line;
    while (channel.NextLine(line, [] {})) {
        if (line.compare(0, 5, "ODOM ") != 0) continue;
        odometry.Push(ParseOdom(line));
    }
    if (!any || !odometry.Latest(last)) throw std::runtime_error("no ODOM records");
    std::ostringstream summary;
    summary.setf(std::ios::fixed);
    summary.precision(6);
    summary << "accepted=" << odometry.accepted()
              << " first_us=" << first.stamp_us
              << " x=" << first.pose.x << " y=" << first.pose.y << " yaw=" << first.pose.yaw
              << " last_us=" << last.stamp_us
              << " x=" << last.pose.x << " y=" << last.pose.y << " yaw=" << last.pose.yaw;
    odometry.Deactivate();
    channel.Queue(summary.str());
    channel.Drain();
}
} // namespace

int main() {
    std::signal(SIGPIPE, SIG_IGN);
    try {
        Receive();
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
