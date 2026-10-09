#pragma once
#include "integration/mapping.hpp"
#include <iosfwd>
#include <memory>
#include <string>

namespace roboframe {
// Line-oriented IPC; decoding and bounded input only, no mapping algorithm or product state.
class MappingStdio {
public:
    MappingStdio(std::istream& input, std::ostream& output, std::string map_directory);
    void Run();
private:
    bool Dispatch(const std::string& line);
    void Send(const std::string& message);
    std::string Status() const;
    std::istream& input_;
    std::ostream& output_;
    std::string map_directory_;
    std::unique_ptr<MappingIntegration> integration_;
};
} // namespace roboframe
