#include <roboframe/log.hpp>
#include <csignal>
#include <exception>
#include <iostream>
#include "interface/stdio_interface.hpp"

int main() {
    try {
#ifdef SIGPIPE
        std::signal(SIGPIPE, SIG_IGN);
#endif
        roboframe::StdioInterface interface(std::cin, std::cout);
        interface.Run();
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("host") << error.what();
        return 1;
    }
}
