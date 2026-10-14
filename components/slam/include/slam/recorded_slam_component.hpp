#pragma once
#include <roboframe/core/component.hpp>
#include <roboframe/datatype/transforms2d.hpp>
#include <roboframe/datatype/scan2d.hpp>
#include <slam/version.hpp>
#include <deque>
#include <functional>

namespace slam {
roboframe::ComponentInfo GetComponentInfo();
// Recorded backend, not SLAM. Reference poses are injected before their matching scans.
// Calls/output are synchronous and serialized; callbacks must not reenter this component.
class RecordedSlamComponent final : public roboframe::Component {
public:
    using Output = std::function<void(const roboframe::MapPose2D&)>;
    explicit RecordedSlamComponent(Output output);
    void Activate() override { lifecycle_ = roboframe::Lifecycle::active; }
    void Deactivate() override { Reset(); lifecycle_ = roboframe::Lifecycle::inactive; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Reset();
    void PushReferencePose(const roboframe::MapPose2D& pose);
    void PushScan(const roboframe::Scan2DPtr& scan, const roboframe::TimedPose2D& odometry);
    std::uint64_t received() const { return received_; }
    std::uint64_t published() const { return published_; }
    std::uint64_t missed_references() const { return missed_; }
private:
    Output output_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    std::deque<roboframe::MapPose2D> references_;
    std::int64_t last_scan_us_ = -1, last_reference_us_ = -1;
    std::uint64_t received_ = 0, published_ = 0, missed_ = 0;
};
} // namespace slam
