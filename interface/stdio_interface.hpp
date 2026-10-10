#pragma once
#include <iosfwd>
#include <string>

namespace roboframe {
class RobotIntegration;
// Line-oriented IPC; decoding and bounded input only, no mapping algorithm or product state.
class StdioInterface {
public:
    StdioInterface(std::istream& input, std::ostream& output, RobotIntegration& integration);
    void Run();
private:
    bool Dispatch(const std::string& line);
    void Send(const std::string& message);
    std::string Status() const;
    std::istream& input_;
    std::ostream& output_;
    RobotIntegration& integration_; // Borrowed; main owns the integration lifetime.
};
} // namespace roboframe
