#include <map_updater/occupancy_map_component.hpp>
#include "occupancy_raster.hpp"
namespace map_updater {
using namespace roboframe;
ComponentInfo GetComponentInfo() { return {component_name, source_id, sdk_version}; }
OccupancyMapComponent::OccupancyMapComponent(GridSpec spec)
    : spec_(spec), raster_(new OccupancyRaster(spec)) {}
OccupancyMapComponent::~OccupancyMapComponent() = default;
void OccupancyMapComponent::Activate() { raster_->Activate(); lifecycle_ = Lifecycle::active; }
void OccupancyMapComponent::Deactivate() { raster_->Deactivate(); lifecycle_ = Lifecycle::inactive; }
void OccupancyMapComponent::Reset() {
    raster_->Reset(); submaps_.clear(); points_ = 0; revision_ = 0; rebuild_copied_bytes_ = 0; stamp_ = -1; published_.reset();
}
bool OccupancyMapComponent::Contains(const Pose2D& pose) const {
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
           pose.x >= spec_.origin_x && pose.y >= spec_.origin_y &&
           pose.x < spec_.origin_x + spec_.width * spec_.resolution &&
           pose.y < spec_.origin_y + spec_.height * spec_.resolution;
}
Update OccupancyMapComponent::Integrate(const Entry& entry, std::size_t start, const std::function<bool()>& canceled) {
    Update result;
    for (std::size_t i = start; i < entry.submap->scans.size(); ++i) {
        const auto& frame = entry.submap->scans[i];
        const auto parent_from_base = Compose(entry.anchor.map_from_node, frame.local_from_base);
        // Raster is private: already placed geometry, so its transform correction is identity.
        const auto update = raster_->PushScan(frame.scan, {{frame.scan->stamp_us, parent_from_base},
                                              {frame.scan->stamp_us, {}}}, canceled);
        result.used_rays += update.used_rays; result.outside_rays += update.outside_rays;
    }
    return result;
}
Update OccupancyMapComponent::PushSubmap(Submap2DPtr submap, const PoseNode2D& anchor,
                                         const std::function<bool()>& canceled) {
    if (lifecycle_ != Lifecycle::active) throw std::logic_error("map updater inactive");
    ValidateNode(anchor);
    if (!submap || !submap->id || !submap->revision || submap->anchor_id != anchor.id ||
        submap->scans.empty() || submap->scans.size() > 64)
        throw std::invalid_argument("invalid submap identity, anchor or size");
    if (spec_.frame == GridFrame::odom &&
        (anchor.map_from_node.x != anchor.odom_from_node.pose.x ||
         anchor.map_from_node.y != anchor.odom_from_node.pose.y ||
         anchor.map_from_node.yaw != anchor.odom_from_node.pose.yaw))
        throw std::invalid_argument("global anchor supplied to odom-frame grid");
    const auto old = submaps_.find(submap->id);
    std::size_t start = 0;
    if (old != submaps_.end()) {
        const auto& before = *old->second.submap;
        if (submap->id != submaps_.rbegin()->first || submap->revision <= before.revision ||
            submap->anchor_id != before.anchor_id || submap->scans.size() <= before.scans.size() ||
            anchor.revision != old->second.anchor.revision ||
            anchor.map_from_node.x != old->second.anchor.map_from_node.x ||
            anchor.map_from_node.y != old->second.anchor.map_from_node.y ||
            anchor.map_from_node.yaw != old->second.anchor.map_from_node.yaw)
            throw std::invalid_argument("submap must append; use UpdateNode for pose revisions");
        start = before.scans.size();
        for (std::size_t i = 0; i < start; ++i) {
            const auto& a = submap->scans[i]; const auto& b = before.scans[i];
            if (a.scan != b.scan || a.local_from_base.x != b.local_from_base.x ||
                a.local_from_base.y != b.local_from_base.y || a.local_from_base.yaw != b.local_from_base.yaw)
                throw std::invalid_argument("immutable submap prefix changed");
        }
    } else {
        if (!submaps_.empty() && submap->id <= submaps_.rbegin()->first)
            throw std::invalid_argument("submap ids must increase");
        for (const auto& item : submaps_)
            if (item.second.anchor.id == anchor.id) throw std::invalid_argument("duplicate submap anchor");
        if (submaps_.size() == max_submaps) throw std::length_error("submap capacity reached");
    }
    std::size_t added = 0;
    auto last = stamp_;
    for (std::size_t i = 0; i < submap->scans.size(); ++i) {
        const auto& frame = submap->scans[i];
        if (!frame.scan) throw std::invalid_argument("null submap scan");
        ValidateScan(*frame.scan); ValidatePose({frame.scan->stamp_us, frame.local_from_base});
        if (i < start) continue;
        if (frame.scan->stamp_us <= last) throw std::invalid_argument("unordered submap scans");
        last = frame.scan->stamp_us; added += frame.scan->hits.size();
        if (!Contains(Compose(anchor.map_from_node, frame.local_from_base)))
            throw std::out_of_range("robot outside configured grid");
    }
    if (submap->scans.front().scan->stamp_us != anchor.odom_from_node.stamp_us)
        throw std::invalid_argument("submap anchor timestamp mismatch");
    if (added > max_points - points_) throw std::length_error("submap point budget reached");
    Entry entry{submap, anchor};
    const auto update = Integrate(entry, start, canceled);
    submaps_[submap->id] = std::move(entry);
    points_ += added; revision_ += submap->scans.size() - start; stamp_ = last;
    published_.reset(); return update;
}
void OccupancyMapComponent::Rebuild(const std::function<bool()>& canceled) {
    rebuild_copied_bytes_ += raster_->copied_bytes();
    raster_->Reset();
    for (const auto& item : submaps_) Integrate(item.second, 0, canceled);
}
void OccupancyMapComponent::UpdateNode(const PoseNode2D& node, const std::function<bool()>& canceled) {
    if (lifecycle_ != Lifecycle::active) throw std::logic_error("map updater inactive");
    ValidateNode(node);
    if (spec_.frame != GridFrame::map) throw std::logic_error("odom atlas cannot be globally relocated");
    auto found = submaps_.end();
    for (auto it = submaps_.begin(); it != submaps_.end(); ++it)
        if (it->second.anchor.id == node.id) { found = it; break; }
    if (found == submaps_.end()) throw std::invalid_argument("unknown submap anchor");
    const auto& old = found->second.anchor;
    if (node.revision <= old.revision || node.odom_from_node.stamp_us != old.odom_from_node.stamp_us ||
        node.odom_from_node.pose.x != old.odom_from_node.pose.x ||
        node.odom_from_node.pose.y != old.odom_from_node.pose.y ||
        node.odom_from_node.pose.yaw != old.odom_from_node.pose.yaw)
        throw std::invalid_argument("invalid anchor revision");
    for (const auto& frame : found->second.submap->scans)
        if (!Contains(Compose(node.map_from_node, frame.local_from_base)))
            throw std::out_of_range("revised submap outside grid");
    found->second.anchor = node;
    Rebuild(canceled); ++revision_; published_.reset();
}
GridSnapshot OccupancyMapComponent::Snapshot() const {
    if (!published_) {
        auto map = std::make_shared<Grid>(*raster_->Snapshot());
        map->revision = revision_; map->stamp_us = stamp_; published_ = std::move(map);
    }
    return published_;
}
std::uint64_t OccupancyMapComponent::copied_bytes() const { return rebuild_copied_bytes_ + raster_->copied_bytes(); }
} // namespace map_updater
