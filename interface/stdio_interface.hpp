#pragma once
#include "interface/stdio_channel.hpp"
#include <roboframe/outputs.hpp>
#include <string>
namespace roboframe {
class RobotIntegration;
class StdioInterface {
public:
    StdioInterface(int input, int output, RobotIntegration& integration);
    ProductOutputs Outputs(); // main connects these while this interface is alive.
    void Run();
private:
    bool Dispatch(const std::string& line);
    void Send(const std::string& message);
    void Event(const std::string& message, const std::string& key = {});
    std::string Status() const;
    StdioChannel channel_;
    RobotIntegration& integration_;
    std::uint64_t request_id_ = 0, last_request_id_ = 0, event_id_ = 0;
    bool subscribed_ = false;
};
} // namespace roboframe
