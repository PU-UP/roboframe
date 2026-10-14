#include <localmapbuilder/local_map_component.hpp>
#include <stdexcept>
using namespace roboframe;
void Check(bool ok) { if (!ok) throw std::runtime_error("submap check failed"); }
template<class F> void Reject(F f) { bool yes=false; try { f(); } catch (const std::exception&) { yes=true; } Check(yes); }
int main() {
    localmapbuilder::LocalMapComponent local(2);
    auto scan = std::make_shared<const Scan2D>(1, std::vector<ScanPoint2D>{{2,0}});
    Reject([&] { local.PushScan(scan,{1,{}},1); });
    local.Activate();
    auto first = local.PushScan(scan,{1,{10,0,1.5707963267948966}},2);
    auto next_scan = std::make_shared<const Scan2D>(2,std::vector<ScanPoint2D>{{2,0}});
    auto second = local.PushScan(next_scan,{2,{10,1,1.5707963267948966}},3);
    Check(first->scans.size()==1 && second->scans.size()==2 && second->anchor_id==2);
    Check(first->scans[0].scan == scan && second->scans[0].scan == scan);
    Check(std::abs(second->scans[1].local_from_base.x - 1)<1e-9);
    Reject([&] { local.PushScan(next_scan,{2,{}},3); });
    auto third=local.PushScan(std::make_shared<const Scan2D>(3,std::vector<ScanPoint2D>{{2,0}}),{3,{}},4);
    Check(third->id==2 && third->anchor_id==4 && third->scans.size()==1);
    local.Reset();
    auto reset=local.PushScan(scan,{1,{}},2);
    Check(reset->id==1 && second->scans.size()==2);
}
