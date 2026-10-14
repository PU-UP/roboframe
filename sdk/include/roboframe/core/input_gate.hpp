#pragma once
#include <cstdint>
#include <mutex>
#include <stdexcept>

namespace roboframe {
// Short input/state access only: never wait for computation or reenter while holding Access.
// Close waits for admitted calls to leave. Owner must close before resetting execution groups.
class InputGate {
public:
    class Access {
    public:
        Access(Access&&) = default;
        bool open() const { return gate_.generation_ != 0; }
        void RequireOpen() const {
            if (!open()) throw std::logic_error("runtime input closed");
        }
        std::uint64_t generation() const { RequireOpen(); return gate_.generation_; }
    private:
        friend class InputGate;
        explicit Access(const InputGate& gate) : gate_(gate), lock_(gate.mutex_) {}
        const InputGate& gate_;
        std::unique_lock<std::mutex> lock_;
    };
    Access Lock() const { return Access(*this); }
    void Open(std::uint64_t generation) {
        auto access = Lock();
        if (!generation || access.open()) throw std::logic_error("invalid input context");
        generation_ = generation;
    }
    void Close() noexcept {
        auto access = Lock();
        generation_ = 0;
    }
private:
    mutable std::mutex mutex_;
    std::uint64_t generation_ = 0;
};
} // namespace roboframe
