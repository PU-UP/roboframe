#pragma once
#include <path_distance/version.hpp>
#include <roboframe/core/component.hpp>
#include <roboframe/core/execution.hpp>
#include <roboframe/datatype/geometry2d.hpp>
namespace path_distance {
struct Sample {
    std::uint64_t count = 0;
    roboframe::TimedPose2D latest{};
    double meters = 0;
};
// Control-owner ports; the owned worker computes in input order.
// Push copies input; full means retry the same sample, never drop distance segments.
// Take releases capacity (queued + active + completed). Deactivate discards old results.
class PathDistanceComponent final : public roboframe::Component {
public:
    using Execution = roboframe::ExecutionChannel<roboframe::TimedPose2D, Sample>;
    explicit PathDistanceComponent(std::size_t capacity = 32);
    ~PathDistanceComponent() override;
    void Activate() override;
    void RequestDeactivate();
    void Deactivate() override;
    roboframe::Lifecycle lifecycle() const override;
    roboframe::Admission Push(roboframe::TimedPose2D pose);
    bool Take(Execution::Result& result);
    std::size_t retained() const;
    bool CanAccept() const;
private:
    Sample Compute(const roboframe::TimedPose2D&, const roboframe::Cancellation&);
    const std::size_t capacity_;
    bool active_ = false;
    std::uint64_t epoch_ = 0;
    Sample sample_{};
    Execution execution_;
};
}
