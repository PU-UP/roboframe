#include <lidar_map/recorded_lidar_map_component.hpp>
#include <cmath>
#include <limits>
#include <stdexcept>
void Check(bool ok) { if (!ok) throw std::runtime_error("lidar-map check failed"); }
template<class F> void Reject(F f) { bool caught=false; try {f();} catch(const std::exception&) {caught=true;} Check(caught); }
void Near(const roboframe::Pose2D& a, const roboframe::Pose2D& b) {
    Check(std::hypot(a.x-b.x,a.y-b.y)<1e-9 && std::abs(roboframe::WrapAngle(a.yaw-b.yaw))<1e-9);
}
int main() {
    using namespace roboframe;
    MapCorrection2D output;
    int count=0;
    lidar_map::RecordedLidarMapComponent component([&](const MapCorrection2D& p){output=p;++count;});
    Reject([&]{component.PushReferencePose({10,{}});});
    component.Activate();
    auto scan=std::make_shared<const Scan2D>(10,std::vector<ScanPoint2D>{{2,0}});
    const TimedPose2D odom{10,{3,-4,1.3}};
    const MapPose2D global{10,{-2,5,-2.8}};
    component.PushReferencePose(global);
    Reject([&]{component.PushReferencePose(global);});
    Reject([&]{component.PushScan(scan,{9,{}});});
    component.PushScan(scan,odom);
    Near(Compose(output.pose,odom.pose),global.pose);
    Near(Compose(odom.pose,Inverse(odom.pose)),{});
    Check(count==1 && output.stamp_us==10 && component.received()==1);
    Reject([&]{component.PushReferencePose({10,{}});});
    auto next=std::make_shared<const Scan2D>(20,std::vector<ScanPoint2D>{{2,0}});
    component.PushReferencePose({30,{1,2,0.3}});
    component.PushScan(next,{20,{4,-4,1.3}});
    Check(count==1); // Never publish a future reference or republish the old one per scan.
    component.PushScan(std::make_shared<const Scan2D>(30,std::vector<ScanPoint2D>{{2,0}}),{30,{4,-3,0.2}});
    Check(count==2 && component.published()==2);
    Near(Compose(output.pose,{4,-3,0.2}),{1,2,0.3});
    Reject([&]{ValidateTransforms({{20,{}},{30,{}}});});
    Reject([&]{ValidateTransforms({{20,{}},{10,{std::numeric_limits<double>::quiet_NaN(),0,0}}});});
    Check(scan->hits[0].x==2); // Shared sensor payload remains unchanged.
    component.Reset();
    Check(component.received()==0 && component.published()==0);
    for(int i=0;i<64;++i) component.PushReferencePose({i,{}});
    Reject([&]{component.PushReferencePose({64,{}});});
    component.Deactivate(); component.Activate();
    component.PushScan(scan,odom);
    Check(component.received()==1 && component.published()==0);
}
