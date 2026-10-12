#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <string>
#include <functional>
#include <stdexcept>
namespace roboframe {
// POSIX pipe transport only. All methods run on the interface owner thread.
class StdioChannel {
public:
    struct InputError : std::invalid_argument { using std::invalid_argument::invalid_argument; };
    StdioChannel(int input, int output);
    ~StdioChannel();
    StdioChannel(const StdioChannel&) = delete;
    StdioChannel& operator=(const StdioChannel&) = delete;
    bool NextLine(std::string& line, const std::function<void()>& service);
    void Queue(std::string line, std::string replace_key = {});
    void DropTelemetry();
    void Drain();
    std::string Status() const;
private:
    using Clock = std::chrono::steady_clock;
    struct Message { std::string text, key; };
    void Flush();
    void Check() const;
    int input_, output_, input_flags_, output_flags_;
    std::deque<Message> pending_;
    std::size_t offset_ = 0, bytes_ = 0, peak_bytes_ = 0, reliable_ = 0;
    std::uint64_t replaced_ = 0;
    std::string failure_, input_buffer_, partial_;
    bool oversized_ = false, eof_ = false;
    Clock::time_point progress_ = Clock::now();
};
} // namespace roboframe
