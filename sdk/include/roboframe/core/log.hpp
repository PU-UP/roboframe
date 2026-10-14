#pragma once
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

namespace roboframe {
enum class LogLevel { debug, info, warning, error, off };
using LogSink = std::function<void(LogLevel, const std::string&, const std::string&)>;

inline const char* LogLevelName(LogLevel level) {
    switch (level) {
        case LogLevel::debug: return "debug";
        case LogLevel::info: return "info";
        case LogLevel::warning: return "warning";
        case LogLevel::error: return "error";
        case LogLevel::off: return "off";
    }
    return "unknown";
}
namespace detail {
struct LogSettings {
    std::mutex mutex;
    LogLevel minimum = LogLevel::info;
    LogSink sink = [](LogLevel level, const std::string& module, const std::string& message) {
        std::clog << "[" << LogLevelName(level) << "] [" << module << "] " << message << '\n';
    };
};
inline LogSettings& Logger() {
    static LogSettings settings;
    return settings;
}
} // namespace detail

inline void SetLogLevel(LogLevel minimum) {
    auto& settings = detail::Logger();
    std::lock_guard<std::mutex> lock(settings.mutex);
    settings.minimum = minimum;
}
// Sinks run serially. They must not call logging/configuration functions recursively.
inline void SetLogSink(LogSink sink) {
    auto& settings = detail::Logger();
    std::lock_guard<std::mutex> lock(settings.mutex);
    settings.sink = std::move(sink);
}
inline void WriteLog(LogLevel level, const std::string& module, const std::string& message) noexcept {
    try {
        auto& settings = detail::Logger();
        std::lock_guard<std::mutex> lock(settings.mutex);
        if (level != LogLevel::off && level >= settings.minimum && settings.sink)
            settings.sink(level, module, message);
    } catch (...) {
        // A failed diagnostic sink must not change component task outcomes.
    }
}
class LogLine {
public:
    LogLine(LogLevel level, const char* module) : level_(level), module_(module) {}
    ~LogLine() noexcept {
        try { WriteLog(level_, module_, stream_.str()); } catch (...) {}
    }
    std::ostream& stream() { return stream_; }
private:
    LogLevel level_;
    const char* module_;
    std::ostringstream stream_;
};
} // namespace roboframe

#define ROBO_LOG_DEBUG(module) ::roboframe::LogLine(::roboframe::LogLevel::debug, module).stream()
#define ROBO_LOG_INFO(module) ::roboframe::LogLine(::roboframe::LogLevel::info, module).stream()
#define ROBO_LOG_WARN(module) ::roboframe::LogLine(::roboframe::LogLevel::warning, module).stream()
#define ROBO_LOG_ERROR(module) ::roboframe::LogLine(::roboframe::LogLevel::error, module).stream()
