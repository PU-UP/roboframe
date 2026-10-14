#pragma once
#include <roboframe/component.hpp>
#include <roboframe/submap2d.hpp>
#include <roboframe/grid.hpp>
#include <map_updater/version.hpp>
#include <functional>
#include <map>
namespace map_updater {
roboframe::ComponentInfo GetComponentInfo();
struct UpdateCanceled : std::runtime_error { UpdateCanceled() : std::runtime_error("map update canceled") {} };
struct Update { std::size_t used_rays = 0, outside_rays = 0; };
class OccupancyRaster;
// Synchronous single-owner atlas. Append immutable local scans, reposition whole submaps.
// Cancellation may leave private evidence partial: Reset before reuse; published maps stay immutable.
class OccupancyMapComponent final : public roboframe::Component {
public:
    explicit OccupancyMapComponent(roboframe::GridSpec spec);
    ~OccupancyMapComponent() override;
    void Activate() override;
    void Deactivate() override;
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Reset();
    Update PushSubmap(roboframe::Submap2DPtr submap, const roboframe::PoseNode2D& anchor,
                      const std::function<bool()>& canceled = {});
    void UpdateNode(const roboframe::PoseNode2D& node, const std::function<bool()>& canceled = {});
    bool Contains(const roboframe::Pose2D& pose) const;
    roboframe::GridSnapshot Snapshot() const;
    std::uint64_t revision() const { return revision_; }
    std::uint64_t copied_bytes() const;
    std::size_t submap_count() const { return submaps_.size(); }
    std::size_t retained_points() const { return points_; }
    static constexpr std::size_t max_submaps = 4096, max_points = 4000000;
private:
    struct Entry { roboframe::Submap2DPtr submap; roboframe::PoseNode2D anchor; };
    Update Integrate(const Entry& entry, std::size_t start, const std::function<bool()>& canceled);
    void Rebuild(const std::function<bool()>& canceled);
    roboframe::GridSpec spec_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
    std::unique_ptr<OccupancyRaster> raster_;
    std::map<std::uint64_t, Entry> submaps_;
    std::size_t points_ = 0;
    std::uint64_t revision_ = 0, rebuild_copied_bytes_ = 0;
    std::int64_t stamp_ = -1;
    mutable roboframe::GridSnapshot published_;
};
} // namespace map_updater
