#include "interface/stdio_interface.hpp"
#include "integration/robot_integration.hpp"
#include <roboframe/log.hpp>
#include <csignal>
#include <iostream>

int main(int argc, char** argv) {
    try {
        if (argc != 2 && argc != 3) throw std::invalid_argument("usage: robot_runtime <map-directory> [--map-delay-ms=N]");
        long delay = 0;
        if (argc == 3) {
            const std::string option(argv[2]), prefix("--map-delay-ms=");
            if (option.compare(0, prefix.size(), prefix) != 0) throw std::invalid_argument("unknown option");
            const auto value = option.substr(prefix.size());
            if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos)
                throw std::invalid_argument("invalid mapper test delay");
            delay = std::stol(value);
            if (delay > 5000) throw std::invalid_argument("mapper test delay exceeds 5000 ms");
        }
#ifdef SIGPIPE
        std::signal(SIGPIPE, SIG_IGN);
#endif
        roboframe::RobotIntegration integration(argv[1], std::chrono::milliseconds(delay));
        roboframe::StdioInterface interface(0, 1, integration);
        integration.SetOutputs(interface.Outputs());
        try { interface.Run(); }
        catch (...) { integration.SetOutputs({}); throw; }
        integration.SetOutputs({});
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("robot-runtime") << error.what();
        return 1;
    }
}
