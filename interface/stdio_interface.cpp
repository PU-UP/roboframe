#include <roboframe/log.hpp>
#include "interface/stdio_interface.hpp"
#include <iostream>
#include <sstream>
#include <stdexcept>

namespace roboframe {
namespace {
const char* StateName(TaskState state) {
    switch (state) {
    case TaskState::idle: return "idle";
    case TaskState::running: return "running";
    case TaskState::canceling: return "canceling";
    case TaskState::succeeded: return "succeeded";
    case TaskState::failed: return "failed";
    case TaskState::canceled: return "canceled";
    }
    throw std::logic_error("unknown task state");
}
std::uint64_t Unsigned(std::istringstream& input) {
    std::string token;
    if (!(input >> token) || token.find_first_not_of("0123456789") != std::string::npos)
        throw std::invalid_argument("expected unsigned integer");
    return std::stoull(token);
}
void End(std::istringstream& input) {
    std::string extra;
    if (input >> extra) throw std::invalid_argument("unexpected argument");
}
}

StdioInterface::StdioInterface(std::istream& input, std::ostream& output)
    : input_(input), output_(output) {
    integration_.SetCallbacks(
        [this](const sample_accumulator::Feedback& f) {
            Send("FEEDBACK " + std::to_string(f.goal_id) + " " +
                 std::to_string(f.count) + " " + std::to_string(f.sum));
        },
        [this](const sample_accumulator::Result& r) {
            Send("RESULT " + std::to_string(r.goal_id) + " " + StateName(r.state) + " " +
                 std::to_string(r.count) + " " + std::to_string(r.sum) + " " +
                 (r.error == sample_accumulator::Error::none ? "none" : "overflow"));
        });
}

void StdioInterface::Send(const std::string& line) {
    output_ << line << std::endl;
    if (!output_) throw std::runtime_error("output channel closed");
}

bool StdioInterface::Dispatch(const std::string& line) {
    ROBO_LOG_INFO("interface") << "received: " << line;
    std::istringstream input(line);
    std::string op;
    input >> op;
    if (op == "GOAL") {
        const auto id = Unsigned(input);
        const auto count = Unsigned(input);
        End(input);
        integration_.Submit({id, count});
        Send("ACCEPTED " + std::to_string(id));
    } else if (op == "SAMPLE") {
        std::int64_t value = 0;
        if (!(input >> value)) throw std::invalid_argument("expected int64 sample");
        End(input);
        integration_.PushSample({value});
    } else if (op == "CANCEL") {
        const auto id = Unsigned(input);
        End(input);
        integration_.Cancel(id);
    } else {
        End(input);
        if (op == "ACTIVATE") {
            integration_.Activate();
            Send("OK ACTIVATE");
        } else if (op == "DEACTIVATE") {
            integration_.Deactivate();
            Send("OK DEACTIVATE");
        } else if (op == "STATUS") {
            const auto task = integration_.status();
            Send(std::string("STATUS ") +
                 (integration_.lifecycle() == Lifecycle::active ? "active " : "inactive ") +
                 std::to_string(task.id) + " " + StateName(task.state));
        } else if (op == "QUIT") {
            return false;
        } else {
            throw std::invalid_argument("unknown command");
        }
    }
    return true;
}

void StdioInterface::Run() {
    Send("READY");
    std::string line;
    bool oversized = false;
    char ch;
    while (input_.get(ch)) {
        if (ch != '\n') {
            if (line.size() < 128) line.push_back(ch);
            else oversized = true;
            continue;
        }
        bool keep_running = true;
        try {
            if (oversized) throw std::invalid_argument("line exceeds 128 bytes");
            keep_running = Dispatch(line);
        } catch (const std::logic_error& error) {
            Send(std::string("ERROR ") + error.what());
        }
        line.clear();
        oversized = false;
        if (!keep_running) {
            integration_.Shutdown();
            Send("BYE");
            return;
        }
    }
    // A pipe closed mid-command must not execute a partial request.
    if (!line.empty() || oversized) Send("ERROR incomplete line");
    integration_.Shutdown();
    if (input_.bad()) throw std::runtime_error("input channel failed");
}
} // namespace roboframe
