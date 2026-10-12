#include "integration/map_update_worker.hpp"
#include <algorithm>
#include <utility>
namespace roboframe {
MapUpdateWorker::MapUpdateWorker(GridSpec spec, std::chrono::milliseconds delay)
    : mapper_(spec), delay_(delay) {
    if (delay.count() < 0 || delay.count() > 5000) throw std::invalid_argument("invalid mapper test delay");
    thread_ = std::thread(&MapUpdateWorker::Run, this);
}
MapUpdateWorker::~MapUpdateWorker() { Stop(); }
void MapUpdateWorker::Reset(std::uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);
    generation_ = generation;
    retained_ -= inputs_.size(); inputs_.clear();
    // Completed/active old results retain their original generation for owner-side rejection.
    changed_.notify_all();
}
bool MapUpdateWorker::Submit(std::uint64_t generation, Scan2DPtr scan, ScanTransforms2D tf) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_ || generation != generation_) throw std::logic_error("mapper lane inactive");
    if (retained_ == capacity) return false; // Drop newest; preserve order of already admitted scans.
    inputs_.push_back({generation, std::move(scan), tf});
    ++retained_; peak_ = std::max(peak_, retained_);
    changed_.notify_one(); return true;
}
bool MapUpdateWorker::Take(Result& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (results_.empty()) return false;
    result = std::move(results_.front()); results_.pop_front(); --retained_;
    return true;
}
MapUpdateWorker::Status MapUpdateWorker::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {inputs_.size(), computing_ ? 1u : 0u, results_.size(), retained_, peak_};
}
void MapUpdateWorker::Stop() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true; generation_ = 0;
        retained_ -= inputs_.size(); inputs_.clear();
        changed_.notify_all();
    }
    if (thread_.joinable()) thread_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    results_.clear(); retained_ = 0;
}
void MapUpdateWorker::Run() {
    mapper_.Activate();
    std::uint64_t mapped_generation = 0;
    for (;;) {
        Input input;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            changed_.wait(lock, [&] { return stopping_ || !inputs_.empty(); });
            if (stopping_) break;
            input = std::move(inputs_.front()); inputs_.pop_front(); computing_ = true;
            // Test delay is interruptible, just like the algorithm's cooperative cancellation points.
            changed_.wait_for(lock, delay_, [&] { return stopping_ || generation_ != input.generation; });
        }
        Result result; result.generation = input.generation; result.scan = std::move(input.scan);
        const auto canceled = [&] { return stopping_ || generation_ != input.generation; };
        try {
            if (canceled()) throw map_updater::UpdateCanceled();
            if (mapped_generation != input.generation) {
                mapper_.Reset(); mapped_generation = input.generation;
            }
            result.update = mapper_.PushScan(result.scan, input.tf, canceled);
            if (canceled()) throw map_updater::UpdateCanceled();
            result.map = mapper_.Snapshot(); result.copied_bytes = mapper_.copied_bytes();
        } catch (const map_updater::UpdateCanceled&) {
            result.outcome = Result::Outcome::canceled; mapped_generation = 0;
        } catch (const std::out_of_range&) {
            result.outcome = Result::Outcome::outside;
        } catch (const std::exception& error) {
            result.outcome = Result::Outcome::failed; result.error = error.what(); mapped_generation = 0;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            computing_ = false;
            if (stopping_) { --retained_; break; }
            results_.push_back(std::move(result)); // Reserved admission credit; worker never waits for owner.
        }
    }
    mapper_.Deactivate();
}
} // namespace roboframe
