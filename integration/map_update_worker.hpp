#pragma once
#include <map_updater/occupancy_map_component.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
namespace roboframe {
// One execution lane for the stateful mapper, not a general executor.
// Owner calls Submit/Reset/Take; only Run touches the algorithm instance.
class MapUpdateWorker {
public:
    struct Result {
        std::uint64_t generation = 0;
        Scan2DPtr scan;
        GridSnapshot map;
        map_updater::Update update;
        std::uint64_t copied_bytes = 0;
        enum class Outcome { success, outside, canceled, failed } outcome = Outcome::success;
        std::string error;
    };
    struct Status { std::size_t queued, active, completed, retained, peak; };
    explicit MapUpdateWorker(GridSpec spec, std::chrono::milliseconds delay);
    ~MapUpdateWorker();
    void Reset(std::uint64_t generation);
    bool Submit(std::uint64_t generation, Scan2DPtr scan, ScanTransforms2D tf);
    bool Take(Result& result);
    Status status() const;
    void Stop();
    static constexpr std::size_t capacity = 8; // queued + computing + completed, not eight of each.
private:
    struct Input { std::uint64_t generation; Scan2DPtr scan; ScanTransforms2D tf; };
    void Run();
    map_updater::OccupancyMapComponent mapper_;
    const std::chrono::milliseconds delay_;
    mutable std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<Input> inputs_;
    std::deque<Result> results_;
    std::atomic<std::uint64_t> generation_{0};
    std::atomic<bool> stopping_{false};
    std::size_t retained_ = 0, peak_ = 0;
    bool computing_ = false;
    std::thread thread_;
};
} // namespace roboframe
