#pragma once
#include <roboframe/datatype/geometry2d.hpp>
#include <cstddef>
#include <deque>

namespace demo_robot {
struct PoseBufferConfig { std::size_t capacity = 512; std::int64_t max_gap_us = 250000; };
enum class PoseMatch { ready, waiting, unavailable };
class PoseBuffer {
public:
    explicit PoseBuffer(PoseBufferConfig config = {});
    void Clear() { samples_.clear(); }
    void PushOdometry(const roboframe::TimedPose2D& sample);
    // No extrapolation. Pose is written only on ready; interpolation follows the shortest yaw arc.
    PoseMatch At(std::int64_t stamp_us, roboframe::TimedPose2D& pose) const;
    std::size_t size() const { return samples_.size(); }
private:
    PoseBufferConfig config_;
    std::deque<roboframe::TimedPose2D> samples_;
};
} // namespace demo_robot
