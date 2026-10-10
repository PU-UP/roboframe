#include "interface/stdio_interface.hpp"
#include <roboframe/mapping.hpp>
#include <roboframe/scan2d.hpp>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace roboframe {
namespace {
std::uint64_t Unsigned(std::istream& input, std::uint64_t maximum = std::numeric_limits<std::int64_t>::max()) {
    std::string token;
    if (!(input >> token) || token.empty() || token.size() > 20 ||
        token.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("expected unsigned integer");
    const auto value = std::stoull(token);
    if (value > maximum) throw std::invalid_argument("integer exceeds limit");
    return value;
}
double Number(std::istream& input) {
    double value;
    if (!(input >> value) || !std::isfinite(value)) throw std::invalid_argument("expected finite number");
    return value;
}
void End(std::istream& input) {
    std::string extra;
    if (input >> extra) throw std::invalid_argument("unexpected argument");
}
const char* MappingPhaseName(MappingPhase phase) {
    switch (phase) {
        case MappingPhase::idle: return "idle";
        case MappingPhase::mapping: return "mapping";
        case MappingPhase::ready: return "ready";
    }
    return "unknown";
}
}
StdioInterface::StdioInterface(std::istream& input, std::ostream& output, std::string directory)
    : input_(input), output_(output), map_directory_(std::move(directory)) {}
void StdioInterface::Send(const std::string& message) {
    output_ << message << std::endl;
    if (!output_) throw std::runtime_error("output channel closed");
}
std::string StdioInterface::Status() const {
    const auto& s = integration_->mapping_status();
    const auto map = integration_->map();
    std::ostringstream out;
    out << "STATUS " << MappingPhaseName(s.phase) << ' ' << s.session << ' ' << s.received << ' '
        << s.integrated << ' ' << s.ignored << ' ' << s.dropped << ' ' << integration_->pending()
        << ' ' << s.used_rays << ' ' << s.outside_rays << ' ' << (map ? map->revision : 0)
        << ' ' << s.drop_sync << ' ' << s.drop_queue << ' ' << s.drop_bounds << ' ' << s.drop_cancel;
    return out.str();
}
bool StdioInterface::Dispatch(const std::string& line) {
    std::istringstream input(line);
    std::string command;
    if (!(input >> command)) throw std::invalid_argument("empty command");
    if (command == "QUIT") {
        End(input);
        if (integration_) integration_->Shutdown();
        Send("BYE");
        return false;
    }
    if (command == "CONFIG") {
        if (integration_) throw std::logic_error("already configured");
        MappingOptions options;
        auto& spec = options.grid;
        spec.width = static_cast<std::uint32_t>(Unsigned(input, 4096));
        spec.height = static_cast<std::uint32_t>(Unsigned(input, 4096));
        spec.resolution = Number(input);
        spec.origin_x = Number(input);
        spec.origin_y = Number(input);
        options.max_pose_gap_us = static_cast<std::int64_t>(Unsigned(input, 10000000));
        End(input);
        integration_ = std::make_unique<RobotIntegration>(options, map_directory_);
    } else {
        if (!integration_) throw std::logic_error("CONFIG required");
        if (command == "BEGIN") {
            const auto id = Unsigned(input);
            End(input);
            integration_->StartMapping(id);
        } else if (command == "ODOM") {
            TimedPose2D pose;
            pose.stamp_us = static_cast<std::int64_t>(Unsigned(input));
            pose.pose.x = Number(input);
            pose.pose.y = Number(input);
            pose.pose.yaw = Number(input);
            End(input);
            integration_->PushOdometry(pose);
        } else if (command == "SCAN") {
            auto scan = std::make_shared<roboframe::Scan2D>();
            scan->stamp_us = static_cast<std::int64_t>(Unsigned(input));
            const auto count = Unsigned(input, 2048);
            scan->hits.reserve(count);
            for (std::size_t i = 0; i < count; ++i) {
                const double x = Number(input), y = Number(input);
                scan->hits.push_back({x, y});
            }
            End(input);
            integration_->PushScan(std::move(scan));
        } else if (command == "SAVE" || command == "SNAPSHOT" || command == "LOAD") {
            std::string name;
            if (!(input >> name)) throw std::invalid_argument("expected map name");
            End(input);
            if (command == "SAVE") integration_->SaveMap(name);
            else if (command == "LOAD") integration_->LoadMap(name);
            else integration_->SnapshotMap(name);
        } else {
            End(input);
            if (command == "FINISH") integration_->FinishMapping();
            else if (command == "CANCEL") integration_->CancelMapping();
            else if (command == "STATUS") { Send(Status()); return true; }
            else throw std::invalid_argument("unknown command");
        }
    }
    Send("OK " + command);
    return true;
}
void StdioInterface::Run() {
    Send("READY roboframe-v1");
    std::string line;
    bool oversized = false;
    char ch;
    while (input_.get(ch)) {
        if (ch != '\n') {
            if (line.size() < 65536) line.push_back(ch);
            else oversized = true;
            continue;
        }
        bool running = true;
        try {
            if (oversized) throw std::invalid_argument("line exceeds 65536 bytes");
            running = Dispatch(line);
        } catch (const std::bad_alloc&) {
            throw; // Memory exhaustion is not a recoverable malformed request.
        } catch (const std::exception& error) {
            if (!output_) throw; // Broken IPC cannot be reported back on the same channel.
            Send(std::string("ERROR ") + error.what());
        }
        line.clear();
        oversized = false;
        if (!running) return;
    }
    if (integration_) integration_->Shutdown();
    if (!line.empty() || oversized) Send("ERROR incomplete line");
    if (input_.bad()) throw std::runtime_error("input channel failed");
}
} // namespace roboframe
