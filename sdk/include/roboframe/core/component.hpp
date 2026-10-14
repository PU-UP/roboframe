#pragma once
#include <roboframe/core/lifecycle.hpp>

namespace roboframe {
// Data and task ports remain typed and belong to each component.
class Component {
public:
    virtual ~Component() = default;
    virtual void Activate() = 0;
    virtual void Deactivate() = 0;
    virtual Lifecycle lifecycle() const = 0;
};
} // namespace roboframe
