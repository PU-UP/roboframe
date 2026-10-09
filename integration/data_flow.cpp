#include <roboframe/log.hpp>
#include "integration/demo_integration.hpp"

namespace roboframe {
// Data routes are separate from task commands; no routing by topic strings or payload casts.
void DemoIntegration::PushSample(sample_accumulator::Sample sample) {
    ROBO_LOG_INFO("integration/data") << "sample -> accumulator.PushSample";
    components_.accumulator.PushSample(sample);
}
} // namespace roboframe
