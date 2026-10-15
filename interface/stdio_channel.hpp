#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <string>
#include <functional>
#include <stdexcept>
#include <mutex>
#include <thread>
#include <condition_variable>
namespace roboframe {
// POSIX transport: input has one owner; bounded output enqueue is concurrent-safe.
// A dedicated writer alone flushes the descriptor, independently of the input owner.
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
    void Check() const; // Output mutex held.
    void RunWriter() noexcept;
    int input_, output_, input_flags_, output_flags_;
    std::deque<Message> pending_;
    std::size_t offset_ = 0, bytes_ = 0, peak_bytes_ = 0, reliable_ = 0;
    std::uint64_t replaced_ = 0;
    mutable std::mutex output_mutex_;
    std::condition_variable output_changed_;
    std::thread writer_;
    bool stopping_=false;
    std::string failure_, input_buffer_, partial_;
    bool oversized_ = false, eof_ = false;
    Clock::time_point progress_ = Clock::now();
};
} // namespace roboframe
