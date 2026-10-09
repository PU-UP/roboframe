#pragma once
#include <roboframe/component.hpp>
#include <roboframe/grid.hpp>
#include <roboframe/version.hpp>
#include <string>

namespace map_store {
constexpr roboframe::Version contract_version{1, 0, 0};
roboframe::ComponentInfo GetComponentInfo();
// Linux storage implementation. Root must already exist. Names are basenames, never paths.
// Save publishes one complete file and never overwrites an existing map.
class Component final : public roboframe::Component {
public:
    explicit Component(std::string directory);
    void Activate() override;
    void Deactivate() override { lifecycle_ = roboframe::Lifecycle::inactive; }
    roboframe::Lifecycle lifecycle() const override { return lifecycle_; }
    void Save(const std::string& name, const roboframe::GridSnapshot& map) const;
    roboframe::GridSnapshot Load(const std::string& name) const;
private:
    std::string Path(const std::string& name) const;
    std::string directory_;
    roboframe::Lifecycle lifecycle_ = roboframe::Lifecycle::inactive;
};
} // namespace map_store
