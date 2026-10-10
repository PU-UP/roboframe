#pragma once
#include <roboframe/scan2d.hpp>
#include <roboframe/mapping.hpp>
#include <memory>
#include <string>

namespace roboframe {
// Product entry point; component assembly is private to the implementation.
class RobotIntegration {
public:
    explicit RobotIntegration(std::string directory);
    ~RobotIntegration();
    RobotIntegration(const RobotIntegration&) = delete;
    RobotIntegration& operator=(const RobotIntegration&) = delete;
    // Commit only after successful activation; failed configuration may be retried.
    void Configure(MappingOptions options);
    void StartMapping(std::uint64_t id);
    void FinishMapping();
    void CancelMapping();
    void SaveMap(const std::string& name);
    void SnapshotMap(const std::string& name); // Inspection checkpoint; does not finish mapping.
    void LoadMap(const std::string& name);
    void PushOdometry(const TimedPose2D& pose);
    void PushScan(roboframe::Scan2DPtr scan);
    void Shutdown();
    const MappingStatus& mapping_status() const;
    std::size_t pending() const;
    GridSnapshot map() const;
private:
    struct Impl;
    Impl& Configured() const;
    std::string map_directory_;
    std::unique_ptr<Impl> impl_;
};
} // namespace roboframe
