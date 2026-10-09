#pragma once
#include <roboframe/component.hpp>
#include <roboframe/geometry2d.hpp>
#include <roboframe/version.hpp>
#include <cstddef>
#include <deque>

namespace odom_buffer {
constexpr roboframe::Version contract_version{1, 0, 0};
roboframe::ComponentInfo GetComponentInfo();
struct Config { std::size_t capacity = 512; std::int64_t max_gap_us = 250000; };
enum class Match { ready, waiting, unavailable };
class Component final : public roboframe::Component {
public:
    explicit Component(Config config = {});
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { lifecycle_ = roboframe::Lifecycle::inactive; samples_.clear(); }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void PushOdometry(const roboframe::TimedPose2D& sample);
    // No extrapolation. Pose is written only on ready; interpolation follows the shortest yaw arc.
    Match At(std::int64_t stamp_us, roboframe::TimedPose2D& pose) const;
    std::size_t size() const { return samples_.size(); }
private:
    Config config_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    std::deque<roboframe::TimedPose2D> samples_;
};
} // namespace odom_buffer
