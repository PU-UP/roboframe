#include "interface/mapping_stdio.hpp"
#include <roboframe/log.hpp>
#include <csignal>
#include <iostream>

int main(int argc, char** argv) {
    try {
        if (argc != 2) throw std::invalid_argument("usage: mapping_runtime <existing-map-directory>");
#ifdef SIGPIPE
        std::signal(SIGPIPE, SIG_IGN);
#endif
        roboframe::MappingStdio interface(std::cin, std::cout, argv[1]);
        interface.Run();
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("mapping-host") << error.what();
        return 1;
    }
}
