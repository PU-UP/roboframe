#include "interface/stdio_interface.hpp"
#include "integration/robot_integration.hpp"
#include <roboframe/log.hpp>
#include <csignal>
#include <iostream>

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::invalid_argument("usage: robot_runtime <existing-map-directory>");
#ifdef SIGPIPE
        std::signal(SIGPIPE, SIG_IGN);
#endif
        roboframe::RobotIntegration integration(argv[1]);
        roboframe::StdioInterface interface(std::cin, std::cout, integration);
        interface.Run();
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("robot-runtime") << error.what();
        return 1;
    }
}
