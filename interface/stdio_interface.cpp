#include "interface/stdio_interface.hpp"
#include "integration/robot_integration.hpp"
#include <roboframe/mapping.hpp>
#include <roboframe/scan2d.hpp>
#include <iostream>
#include <iomanip>
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
        case MappingPhase::finishing: return "finishing";
    }
    return "unknown";
}
}
StdioInterface::StdioInterface(int input, int output, RobotIntegration& integration)
    : channel_(input, output), integration_(integration) {}
void StdioInterface::Send(const std::string& message) {
    if (request_id_) channel_.Queue("RES " + std::to_string(request_id_) + " " + message);
}
void StdioInterface::Event(const std::string& message, const std::string& key) {
    if (subscribed_) channel_.Queue("EVENT " + std::to_string(++event_id_) + " " + message, key);
}
namespace {
std::string PoseText(const Pose2D& pose) {
    std::ostringstream out; out << std::setprecision(17) << pose.x << ' ' << pose.y << ' ' << pose.yaw;
    return out.str();
}
std::string MapText(const MapView& view) {
    const auto& g = *view.grid;
    std::ostringstream out;
    out << std::setprecision(17) << view.generation << ' ' << g.revision << ' ' << g.stamp_us << ' '
        << (g.spec.frame == GridFrame::map ? "map" : "odom") << ' ' << g.spec.width << ' ' << g.spec.height
        << ' ' << g.spec.resolution << ' ' << g.spec.origin_x << ' ' << g.spec.origin_y;
    return out.str();
}
}
ProductOutputs StdioInterface::Outputs() {
    ProductOutputs output;
    output.odometry = [this](const TimedPose2D& value) {
        Event("ODOM " + std::to_string(value.stamp_us) + " " + PoseText(value.pose), "odom");
    };
    output.correction = [this](const MapCorrection2D& value) {
        Event("CORRECTION " + std::to_string(value.stamp_us) + " " + PoseText(value.pose), "correction");
    };
    output.transforms = [this](const TransformStatus2D& value) {
        Event(value.valid ? "TF valid " + std::to_string(value.transforms.odom_from_base.stamp_us) + " " +
            std::to_string(value.transforms.map_from_odom.stamp_us) + " " + PoseText(MapFromBase(value.transforms)) :
            "TF unavailable", "tf");
    };
    output.map = [this](const MapView& value) { Event("MAP " + MapText(value), "map"); };
    output.mapping = [this](const MappingEvent& value) {
        const char* name = "unknown";
        switch (value.kind) {
            case MappingEventKind::started: name = "started"; break;
            case MappingEventKind::progress: name = "progress"; break;
            case MappingEventKind::finished: name = "finished"; break;
            case MappingEventKind::canceled: name = "canceled"; break;
            case MappingEventKind::saved: name = "saved"; break;
            case MappingEventKind::loaded: name = "loaded"; break;
            case MappingEventKind::stopped: name = "stopped"; break;
            case MappingEventKind::finishing: name = "finishing"; break;
            case MappingEventKind::failed: name = "failed"; break;
        }
        Event("MAPPING " + std::string(name) + " " + std::to_string(request_id_) + " " +
            std::to_string(value.status.session) + " " + std::to_string(value.generation) + " " +
            std::to_string(value.status.integrated) + " " + std::to_string(value.status.dropped) + " " +
            (value.name.empty() ? "-" : value.name), value.kind == MappingEventKind::progress ? "progress" : "");
    };
    return output;
}
std::string StdioInterface::Status() const {
    const auto& s = integration_.mapping_status();
    const auto map = integration_.map();
    std::ostringstream out;
    out << "STATUS " << MappingPhaseName(s.phase) << ' ' << s.session << ' ' << s.received << ' '
        << s.integrated << ' ' << s.ignored << ' ' << s.dropped << ' ' << integration_.pending()
        << ' ' << s.used_rays << ' ' << s.outside_rays << ' ' << (map ? map->revision : 0)
        << ' ' << s.drop_sync << ' ' << s.drop_queue << ' ' << s.drop_bounds << ' ' << s.drop_cancel;
    return out.str();
}
bool StdioInterface::Dispatch(const std::string& line) {
    std::istringstream input(line);
    std::string command;
    if (!(input >> command)) throw std::invalid_argument("empty command");
    if (!request_id_ && command != "ODOM" && command != "SCAN" && command != "MAP_POSE")
        throw std::invalid_argument("DATA accepts only sensor inputs");
    if (command == "SUBSCRIBE") {
        std::string value; input >> value; End(input);
        if (value != "ON" && value != "OFF") throw std::invalid_argument("expected ON or OFF");
        subscribed_ = value == "ON";
        if (!subscribed_) channel_.DropTelemetry();
        Send("OK SUBSCRIBE"); return true;
    }
    if (command == "IPC_STATUS") { End(input); Send(channel_.Status()); return true; }
    if (command == "GET_MAP") {
        std::string role; input >> role; End(input);
        if (role != "live" && role != "baseline") throw std::invalid_argument("expected live or baseline");
        const auto view = integration_.map_view(role == "baseline");
        if (!view.grid) throw std::logic_error("map unavailable");
        std::string response = "MAP " + role + " " + MapText(view) + " ";
        response.reserve(response.size() + 2 * view.grid->cells.size());
        // shortcut: bounded full-map encoding is synchronous; use chunks for frequent large-map export.
        const char* hex = "0123456789abcdef";
        for (std::size_t i = 0; i < view.grid->cells.size(); ++i) {
            const auto cell = view.grid->cells[i]; response += hex[cell >> 4]; response += hex[cell & 15];
        }
        Send(response); return true;
    }
    if (command == "QUIT") {
        End(input);
        integration_.Shutdown();
        Send("BYE");
        return false;
    }
    if (command == "TF") {
        End(input);
        const auto state = integration_.transforms();
        if (!state.valid) { Send("TF unavailable"); return true; }
        const auto& tf = state.transforms;
        const auto global = MapFromBase(tf);
        std::ostringstream out;
        out << std::setprecision(17) << "TF " << tf.odom_from_base.stamp_us << ' ' << tf.map_from_odom.stamp_us;
        for (const auto& p : {tf.odom_from_base.pose, tf.map_from_odom.pose, global})
            out << ' ' << p.x << ' ' << p.y << ' ' << p.yaw;
        Send(out.str());
        return true;
    }
    if (command == "RESOURCES") {
        End(input);
        const auto s = integration_.data_flow_status();
        std::ostringstream out;
        out << "RESOURCES";
        out << " compute_pending=" << s.compute_pending << " compute_active=" << s.compute_active
            << " compute_results=" << s.compute_results << " compute_retained=" << s.compute_retained
            << " compute_peak_retained=" << s.compute_peak_retained << " compute_rejected=" << s.compute_rejected
            << " stale_results=" << s.stale_results;
        out << " lidar_scans=" << s.lidar_scans << " correction_outputs=" << s.correction_outputs
            << " missed_references=" << s.missed_references << " tf_dropped=" << s.tf_dropped;
        out << " shared_scans=" << s.shared_scans;
        out << " live_updates=" << s.live_updates;
        out << " live_dropped=" << s.live_dropped;
        out << " publications=" << s.publications;
        out << " copied_bytes=" << s.copied_bytes;
        out << " map_bytes=" << s.map_bytes;
        out << " table_bytes=" << s.table_bytes;
        out << " planner_received=" << s.planner_received;
        out << " planner_completed=" << s.planner_completed;
        out << " planner_replaced=" << s.planner_replaced;
        out << " planner_pending=" << s.planner_pending;
        out << " planner_active=" << s.planner_active;
        out << " planner_peak_retained=" << s.planner_peak_retained;
        out << " planner_inconsistent=" << s.planner_inconsistent;
        out << " planner_revision=" << s.planner_revision;
        Send(out.str());
        return true;
    }
    if (command == "CONFIG") {
        MappingOptions options;
        auto& spec = options.grid;
        spec.width = static_cast<std::uint32_t>(Unsigned(input, 4096));
        spec.height = static_cast<std::uint32_t>(Unsigned(input, 4096));
        spec.resolution = Number(input);
        spec.origin_x = Number(input);
        spec.origin_y = Number(input);
        options.max_pose_gap_us = static_cast<std::int64_t>(Unsigned(input, 10000000));
        std::string mode;
        if (input >> mode) {
            if (mode != "MAP_TF") throw std::invalid_argument("expected MAP_TF");
            options.grid.frame = GridFrame::map;
            options.max_correction_age_us = static_cast<std::int64_t>(Unsigned(input, 60000000));
        }
        End(input);
        integration_.Configure(options);
    } else {
        if (command == "BEGIN") {
            const auto id = Unsigned(input);
            End(input);
            integration_.StartMapping(id);
        } else if (command == "ODOM") {
            TimedPose2D pose;
            pose.stamp_us = static_cast<std::int64_t>(Unsigned(input));
            pose.pose.x = Number(input);
            pose.pose.y = Number(input);
            pose.pose.yaw = Number(input);
            End(input);
            integration_.PushOdometry(pose);
        } else if (command == "MAP_POSE") {
            MapPose2D pose;
            pose.stamp_us = static_cast<std::int64_t>(Unsigned(input));
            pose.pose.x = Number(input); pose.pose.y = Number(input); pose.pose.yaw = Number(input);
            End(input);
            integration_.PushMapReference(pose);
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
            integration_.PushScan(std::move(scan));
        } else if (command == "SAVE" || command == "SNAPSHOT" || command == "LOAD") {
            std::string name;
            if (!(input >> name)) throw std::invalid_argument("expected map name");
            End(input);
            if (command == "SAVE") integration_.SaveMap(name);
            else if (command == "LOAD") integration_.LoadMap(name);
            else integration_.SnapshotMap(name);
        } else {
            End(input);
            if (command == "FINISH") { integration_.FinishMapping(); Send("ACCEPTED FINISH"); return true; }
            else if (command == "CANCEL") integration_.CancelMapping();
            else if (command == "STATUS") { Send(Status()); return true; }
            else if (command == "WAIT_PLANNER") integration_.WaitPlanner();
            else throw std::invalid_argument("unknown command");
        }
    }
    Send("OK " + command);
    return true;
}
void StdioInterface::Run() {
    channel_.Queue("READY roboframe-v3");
    try {
        std::string line;
        while (channel_.NextLine(line, [this] { request_id_ = 0; integration_.Poll(); })) {
            request_id_ = 0;
            try {
                if (line.size() > 65536) throw std::invalid_argument("line exceeds 65536 bytes");
                std::istringstream envelope(line);
                std::string kind; envelope >> kind;
                if (kind == "REQ") {
                    const auto id = Unsigned(envelope);
                    if (!id || id <= last_request_id_) throw std::invalid_argument("request id must increase");
                    request_id_ = last_request_id_ = id;
                } else if (kind != "DATA") throw std::invalid_argument("expected REQ id or DATA");
                std::string command; std::getline(envelope, command);
                if (!Dispatch(command)) break;
            } catch (const std::bad_alloc&) { throw; }
            catch (const std::exception& error) {
                if (request_id_) Send(std::string("ERROR ") + error.what());
                else channel_.Queue(std::string("RES 0 ERROR ") + error.what());
            }
        }
        request_id_ = 0;
        integration_.Shutdown();
        channel_.Drain();
    } catch (const std::invalid_argument& error) {
        request_id_ = 0;
        channel_.Queue(std::string("RES 0 ERROR ") + error.what());
        integration_.Shutdown(); channel_.Drain();
    } catch (...) {
        integration_.SetOutputs({});
        integration_.Shutdown();
        throw;
    }
}
} // namespace roboframe
