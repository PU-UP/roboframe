#include "interface/stdio_interface.hpp"
#include "integration/robot_integration.hpp"
#include <roboframe/log.hpp>
#include <csignal>
#include <utility>

namespace {
roboframe::ProductInputs BindProduct(roboframe::RobotIntegration& product) {
    roboframe::ProductInputs inputs;
    inputs.sensors.odometry = [&](const roboframe::TimedPose2D& pose) { product.PushOdometry(pose); };
    inputs.sensors.scan = [&](roboframe::Scan2DPtr scan) { product.PushScan(std::move(scan)); };
    inputs.sensors.map_reference = [&](const roboframe::MapPose2D& pose) { product.PushMapReference(pose); };
    inputs.mapping.configure = [&](roboframe::MappingOptions options) { product.Configure(options); };
    inputs.mapping.start = [&](std::uint64_t id) { product.StartMapping(id); };
    inputs.mapping.finish = [&] { product.FinishMapping(); };
    inputs.mapping.cancel = [&] { product.CancelMapping(); };
    inputs.mapping.status = [&] {
        const auto map = product.map();
        return roboframe::MappingReport{product.mapping_status(), product.pending(), map ? map->revision : 0};
    };
    inputs.maps.save = [&](const std::string& name) { product.SaveMap(name); };
    inputs.maps.load = [&](const std::string& name) { product.LoadMap(name); };
    inputs.maps.snapshot = [&](const std::string& name) { product.SnapshotMap(name); };
    inputs.maps.view = [&](bool baseline) { return product.map_view(baseline); };
    inputs.diagnostics.transforms = [&] { return product.transforms(); };
    inputs.diagnostics.resources = [&] { return product.data_flow_status(); };
    inputs.diagnostics.check_readers_idle = [&] { product.WaitPlanner(); };
    return inputs;
}
}

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
        roboframe::StdioInterface interface(0, 1, BindProduct(integration));
        integration.SetOutputs(interface.Outputs());
        try {
            while (interface.ProcessNext([&] { integration.Poll(); }) ==
                   roboframe::StdioInterface::ReceiveResult::received) {}
            integration.Shutdown(); // This product stops on QUIT/EOF; transport only reports the event.
            interface.Finish();
        } catch (...) {
            integration.SetOutputs({});
            integration.Shutdown();
            throw;
        }
        integration.SetOutputs({});
    } catch (const std::exception& error) {
        ROBO_LOG_ERROR("robot-runtime") << error.what();
        return 1;
    }
}
