#pragma once
#include "interface/stdio_channel.hpp"
#include <demo_robot/inputs.hpp>
#include <string>
namespace demo_robot {
// Demo protocol adapter: depends on its public contract, never integration implementation.
class DemoStdioInterface {
public:
    enum class ReceiveResult { received, quit, end_of_input, invalid_input };
    DemoStdioInterface(int input, int output, DemoRobotInputs inputs);
    DemoRobotOutputs Outputs(); // main connects these while this interface is alive.
    // One request per call; service runs between reads on this same owner thread.
    // End reasons are explicit; exceptions report transport or service failure.
    ReceiveResult ProcessNext(const std::function<void()>& service);
    void Finish(); // Product stops publishers first; then send BYE (QUIT only) and drain.
private:
    bool Dispatch(const std::string& line);
    void Send(const std::string& message);
    void Event(const std::string& message, const std::string& key = {});
    std::string Status() const;
    roboframe::StdioChannel channel_;
    const DemoRobotInputs inputs_;
    bool quit_ = false;
    std::uint64_t request_id_ = 0, last_request_id_ = 0, event_id_ = 0;
    bool subscribed_ = false;
};
} // namespace demo_robot
