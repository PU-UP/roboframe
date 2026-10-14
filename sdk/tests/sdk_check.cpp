#include <roboframe/core/component.hpp>
#include <roboframe/core/log.hpp>
#include <roboframe/core/task.hpp>
#include <roboframe/core/version.hpp>
#include <stdexcept>
#include <thread>

void Require(bool value) {
    if (!value) throw std::runtime_error("SDK check failed");
}
template<class Function> void Reject(Function function) {
    bool rejected = false;
    try { function(); } catch (const std::runtime_error&) { rejected = true; }
    Require(rejected);
}
int main() {
    using namespace roboframe;
    static_assert(Compatible({1, 2, 0}, {1, 1, 9}), "compatible minor");
    static_assert(!Compatible({2, 0, 0}, {1, 0, 0}), "incompatible major");
    static_assert(!Compatible({1, 0, 0}, {1, 0, 1}), "missing patch");
    ComponentInfo info{"test", "source-fingerprint", sdk_version};
    CheckCompatibility(info);
    info.built_with_sdk.major += 1;
    Reject([&] { CheckCompatibility(info); });
    info.built_with_sdk = sdk_version;
    info.built_with_sdk.minor += 1;
    Reject([&] { CheckCompatibility(info); });

    int count = 0;
    std::string last;
    SetLogSink([&](LogLevel, const std::string& module, const std::string& message) {
        ++count;
        last = module + ":" + message;
    });
    ROBO_LOG_DEBUG("test") << "hidden";
    ROBO_LOG_INFO("test") << "value=" << 3;
    Require(count == 1 && last == "test:value=3");
    SetLogLevel(LogLevel::off);
    ROBO_LOG_ERROR("test") << "hidden";
    Require(count == 1);
    SetLogLevel(LogLevel::info);
    auto log = [] { for (int i = 0; i < 100; ++i) ROBO_LOG_INFO("thread") << i; };
    std::thread first(log), second(log);
    first.join();
    second.join();
    Require(count == 201);
    SetLogSink([](LogLevel, const std::string&, const std::string&) {
        throw std::runtime_error("broken sink");
    });
    ROBO_LOG_INFO("test") << "must not escape";
    SetLogSink({});
}
