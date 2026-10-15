#include "interface/stdio_channel.hpp"
#include <algorithm>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <unistd.h>
namespace roboframe {
namespace {
constexpr std::size_t max_bytes = 20 * 1024 * 1024;
constexpr std::size_t max_reliable = 256;
constexpr auto timeout = std::chrono::seconds(2);
}
StdioChannel::StdioChannel(int input, int output) : input_(input), output_(output),
    input_flags_(::fcntl(input, F_GETFL)), output_flags_(::fcntl(output, F_GETFL)) {
    if (input_flags_ < 0 || output_flags_ < 0) throw std::runtime_error("invalid IPC descriptors");
    if (::fcntl(input_, F_SETFL, input_flags_ | O_NONBLOCK) < 0)
        throw std::runtime_error("cannot configure IPC input");
    if (::fcntl(output_, F_SETFL, output_flags_ | O_NONBLOCK) < 0) {
        ::fcntl(input_, F_SETFL, input_flags_);
        throw std::runtime_error("cannot configure IPC output");
    }
    try { writer_=std::thread([this] { RunWriter(); }); }
    catch (...) {
        ::fcntl(input_, F_SETFL, input_flags_);
        ::fcntl(output_, F_SETFL, output_flags_);
        throw;
    }
}
StdioChannel::~StdioChannel() {
    {
        std::lock_guard<std::mutex> lock(output_mutex_);
        stopping_=true;output_changed_.notify_all();
    }
    if (writer_.joinable()) writer_.join();
    ::fcntl(input_, F_SETFL, input_flags_);
    ::fcntl(output_, F_SETFL, output_flags_);
}
void StdioChannel::Queue(std::string line, std::string key) {
    std::lock_guard<std::mutex> lock(output_mutex_);
    if (!failure_.empty()) return;
    if (!key.empty()) {
        for (auto it = pending_.begin(); it != pending_.end(); ++it) {
            if (it->key == key && (it != pending_.begin() || offset_ == 0)) {
                bytes_ -= it->text.size(); pending_.erase(it); ++replaced_; break;
            }
        }
    }
    line += '\n';
    if (bytes_ + line.size() > max_bytes || (key.empty() && reliable_ == max_reliable)) {
        failure_ = "IPC output queue exhausted"; return;
    }
    if (pending_.empty()) progress_ = Clock::now();
    bytes_ += line.size(); peak_bytes_ = std::max(peak_bytes_, bytes_);
    if (key.empty()) ++reliable_;
    pending_.push_back({std::move(line), std::move(key)});
    output_changed_.notify_all();
}
void StdioChannel::DropTelemetry() {
    std::lock_guard<std::mutex> lock(output_mutex_);
    for (auto it = pending_.begin(); it != pending_.end();) {
        if (!it->key.empty() && (it != pending_.begin() || offset_ == 0)) {
            bytes_ -= it->text.size(); it = pending_.erase(it);
        } else ++it;
    }
}
void StdioChannel::Check() const {
    if (!failure_.empty()) throw std::runtime_error(failure_);
    if (!pending_.empty() && Clock::now() - progress_ >= timeout)
        throw std::runtime_error("IPC reader stalled");
}
void StdioChannel::Flush() {
    Check();
    std::size_t budget = 65536; // Bound lock hold time for concurrent producers.
    while (!pending_.empty() && budget) {
        const auto& text = pending_.front().text;
        const auto n = ::write(output_, text.data() + offset_, std::min(budget, text.size() - offset_));
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
            throw std::runtime_error("IPC output disconnected");
        }
        if (!n) throw std::runtime_error("IPC output made no progress");
        offset_ += n; budget -= n; progress_ = Clock::now();
        if (offset_ == text.size()) {
            bytes_ -= text.size();
            if (pending_.front().key.empty()) --reliable_;
            pending_.pop_front(); offset_ = 0;
        }
    }
}
bool StdioChannel::NextLine(std::string& line, const std::function<void()>& service) {
    for (;;) {
        service();
        {
            std::lock_guard<std::mutex> lock(output_mutex_);Check();
        }
        const auto end = input_buffer_.find('\n');
        const auto count = end == std::string::npos ? input_buffer_.size() : end;
        const auto keep = std::min(count, 65537 - partial_.size());
        partial_.append(input_buffer_, 0, keep);
        oversized_ = oversized_ || partial_.size() > 65536 || keep < count;
        input_buffer_.erase(0, count + (end != std::string::npos));
        if (end != std::string::npos) {
            line = std::move(partial_); partial_.clear();
            if (oversized_) line.assign(65537, 'X');
            oversized_ = false;
            return true;
        }
        if (eof_) {
            if (!partial_.empty() || oversized_) throw InputError("incomplete line");
            return false;
        }
        pollfd fds[] = {{input_, POLLIN, 0}, {output_, 0, 0}};
        const auto ready = ::poll(fds, 2, 5);
        if (ready < 0) { if (errno == EINTR) continue; throw std::runtime_error("IPC poll failed"); }
        if (fds[1].revents & (POLLERR | POLLHUP | POLLNVAL))
            throw std::runtime_error("IPC output disconnected");
        if (fds[0].revents & (POLLERR | POLLNVAL)) throw std::runtime_error("IPC input failed");
        if (fds[0].revents & (POLLIN | POLLHUP)) {
            char buffer[4096];
            const auto n = ::read(input_, buffer, sizeof(buffer));
            if (n > 0) input_buffer_.append(buffer, n);
            else if (!n) eof_ = true;
            else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK)
                throw std::runtime_error("IPC input failed");
        }
    }
}
void StdioChannel::RunWriter() noexcept {
    std::unique_lock<std::mutex> lock(output_mutex_);
    try {
        while (!stopping_) {
            Flush();
            output_changed_.notify_all();
            const bool pending=!pending_.empty();
            lock.unlock();
            pollfd fd{output_,short(pending ? POLLOUT : 0),0};
            const auto ready=::poll(&fd,1,pending ? 5 : 0);
            lock.lock();
            if (ready<0 && errno!=EINTR) throw std::runtime_error("IPC poll failed");
            if (fd.revents & (POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("IPC output disconnected");
            // Idle wait is notified by enqueue; a blocked pipe is polled without holding the lock.
            if (!pending) output_changed_.wait_for(lock,std::chrono::milliseconds(5));
        }
    } catch (const std::exception& error) {
        failure_=error.what();
    } catch (...) {
        failure_="IPC writer failed";
    }
    output_changed_.notify_all();
}
void StdioChannel::Drain() {
    std::unique_lock<std::mutex> lock(output_mutex_);
    const auto deadline=Clock::now()+timeout;
    while (!pending_.empty()) {
        Check();
        if (output_changed_.wait_until(lock,deadline)==std::cv_status::timeout && !pending_.empty())
            throw std::runtime_error("IPC close drain timed out");
    }
    Check();
}
std::string StdioChannel::Status() const {
    std::lock_guard<std::mutex> lock(output_mutex_);
    return "IPC_STATUS bytes=" + std::to_string(bytes_) + " peak_bytes=" + std::to_string(peak_bytes_) +
        " pending=" + std::to_string(pending_.size()) + " replaced=" + std::to_string(replaced_);
}
} // namespace roboframe
