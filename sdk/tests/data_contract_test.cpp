#include <roboframe/scan2d.hpp>
#include <roboframe/mapping.hpp>
#include <limits>
#include <stdexcept>
#include <type_traits>
void Require(bool ok) { if (!ok) throw std::runtime_error("data contract check failed"); }
template<class F> void Reject(F f) {
    bool caught = false;
    try { f(); } catch (const std::invalid_argument&) { caught = true; }
    Require(caught);
}
int main() {
    using namespace roboframe;
    static_assert(std::is_const<Scan2DPtr::element_type>::value, "published scans are read-only");
    const Scan2D scan{7, {{30, 0}}};
    ValidateScan2D(scan); // 20m is not an SDK-wide sensor limit.
    ValidateScan2D({8, std::vector<ScanPoint2D>(2049, {1, 0})}); // Nor is the mapper's capacity.
    Reject([] { ValidateScan2D({-1, {{1, 0}}}); });
    Reject([] { ValidateScan2D({0, {}}); });
    Reject([] { ValidateScan2D({0, {{std::numeric_limits<double>::quiet_NaN(), 0}}}); });
    Reject([] { ValidateScan2D({0, {{0, std::numeric_limits<double>::infinity()}}}); });
    MappingOptions options{{20, 20, 0.1, -1, -1}, 100};
    ValidateGridSpec(options.grid);
    MappingStatus status;
    Require(options.max_pose_gap_us == 100 && status.phase == MappingPhase::idle && status.session == 0);
}
