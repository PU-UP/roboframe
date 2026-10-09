#pragma once
#include <iosfwd>
#include <string>
#include "integration/demo_integration.hpp"

namespace roboframe {
class StdioInterface {
public:
    StdioInterface(std::istream& input, std::ostream& output);
    StdioInterface(const StdioInterface&) = delete;
    StdioInterface& operator=(const StdioInterface&) = delete;
    void Run();
private:
    bool Dispatch(const std::string& line);
    void Send(const std::string& line);
    std::istream& input_;
    std::ostream& output_;
    DemoIntegration integration_;
};
} // namespace roboframe
