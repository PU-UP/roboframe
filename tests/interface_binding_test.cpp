#include "interface/stdio_interface.hpp"
#include <stdexcept>
#include <thread>
#include <unistd.h>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("interface binding check failed"); }
struct Pipe {
    int fd[2];
    Pipe() { Check(::pipe(fd) == 0); }
    ~Pipe() { for (int f : fd) if (f >= 0) ::close(f); }
    void CloseWriter() { ::close(fd[1]); fd[1] = -1; }
};
std::string Exchange(ProductInputs inputs, const std::string& requests,
                     const std::function<void()>& service,
                     const std::function<void(ProductOutputs)>& stop = {},
                     StdioInterface::ReceiveResult* reason = nullptr) {
    Pipe incoming, outgoing;
    Check(::write(incoming.fd[1], requests.data(), requests.size()) == static_cast<ssize_t>(requests.size()));
    incoming.CloseWriter();
    {
        StdioInterface transport(incoming.fd[0], outgoing.fd[1], std::move(inputs));
        StdioInterface::ReceiveResult result;
        do { result = transport.ProcessNext(service); } while (result == StdioInterface::ReceiveResult::received);
        if (reason) *reason = result;
        if (stop) stop(transport.Outputs()); // Application decides what ending the connection means.
        transport.Finish();
    }
    outgoing.CloseWriter();
    std::string result;
    char buffer[4096];
    ssize_t count;
    while ((count = ::read(outgoing.fd[0], buffer, sizeof(buffer))) > 0) result.append(buffer, count);
    Check(count == 0);
    return result;
}
int main() {
    const auto owner = std::this_thread::get_id();
    int ticks = 0, scans = 0, stops = 0;
    std::uint64_t started = 0;
    ProductInputs mapping_product;
    mapping_product.mapping.start = [&](std::uint64_t id) { Check(std::this_thread::get_id() == owner); started = id; };
    mapping_product.sensors.scan = [&](Scan2DPtr scan) { Check(scan->stamp_us == 7 && scan->hits.size() == 1); ++scans; };
    StdioInterface::ReceiveResult reason;
    auto output = Exchange(mapping_product,
        "REQ 1 SUBSCRIBE ON\nREQ 2 BEGIN 3\nDATA SCAN 7 1 2 0\nREQ 3 FINISH\nREQ 4 QUIT\n",
        [&] { Check(std::this_thread::get_id() == owner && stops == 0); ++ticks; },
        [&](ProductOutputs publish) { ++stops; publish.mapping({MappingEventKind::stopped, {}}); }, &reason);
    Check(reason == StdioInterface::ReceiveResult::quit);
    Check(started == 3 && scans == 1 && ticks > 0 && stops == 1);
    Check(output.find("RES 2 OK BEGIN") != std::string::npos);
    Check(output.find("RES 3 ERROR unsupported: FINISH") != std::string::npos);
    Check(output.find("MAPPING stopped") < output.find("RES 4 BYE"));
    Check(output.find("MAPPING stopped 4") != std::string::npos);

    // The same interface works with a sensor-only product, with no integration linked.
    ProductInputs sensor_product;
    int odoms = 0;
    sensor_product.sensors.odometry = [&](const TimedPose2D& pose) { Check(pose.stamp_us == 8); ++odoms; };
    output = Exchange(sensor_product, "REQ 1 BEGIN 9\nDATA ODOM 8 0 0 0\n", [] {}, {}, &reason);
    Check(reason == StdioInterface::ReceiveResult::end_of_input);
    Check(odoms == 1 && output.find("RES 1 ERROR unsupported: BEGIN") != std::string::npos);
    Check(output.find("BYE") == std::string::npos); // EOF is not a QUIT request.
    output = Exchange({}, "REQ 1 BEGIN", [] {}, {}, &reason);
    Check(reason == StdioInterface::ReceiveResult::invalid_input);
    Check(output.find("RES 0 ERROR incomplete line") != std::string::npos);
    bool propagated = false;
    try { Exchange({}, "", [] { throw std::invalid_argument("service failure"); }); }
    catch (const std::invalid_argument& error) { propagated = std::string(error.what()) == "service failure"; }
    Check(propagated); // A product failure must not be reported as malformed IPC input.
}
