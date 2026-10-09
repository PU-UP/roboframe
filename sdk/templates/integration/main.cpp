#include "integration.hpp"
#include <roboframe/log.hpp>
#include <stdexcept>

int main() {
    try {
        Integration integration({2.0});
        double result = 0;
        integration.SetOutputCallback([&](value_scale::Output output) {
            result = output.value;
            ROBO_LOG_INFO("integration") << "output=" << result;
        });
        integration.Start();
        integration.PushValue({3.0});
        integration.Shutdown();
        if (result != 6.0) throw std::runtime_error("integration self-check failed");
        ROBO_LOG_INFO("integration") << "PASS";
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("integration") << error.what();
        return 1;
    }
}
