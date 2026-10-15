#include <planner/planner_component.hpp>
#include <algorithm>
#include <queue>
#include <limits>
namespace planner {
using namespace roboframe;
ComponentInfo GetComponentInfo() { return {component_name,source_id,sdk_version}; }
PlannerComponent::PlannerComponent(double radius)
    : radius_(radius),execution_(std::chrono::milliseconds(0),{
        {},
        [this](const PlanGoal& goal,const Execution::Context& context) {
            auto result=Plan(goal,context);
            const auto state=result.status==PlanStatus::success ? TaskState::succeeded : TaskState::failed;
            return Execution::Step{state,{},std::move(result)};
        },{}
    }) {
    if (!std::isfinite(radius) || radius<0 || radius>2) throw std::invalid_argument("invalid robot radius");
}
PlanResult PlannerComponent::Plan(const PlanGoal& input,const Execution::Context& context) const {
    const auto& map=input.map;
    const auto& tf=input.tf;
    const auto& goal=input.goal;
    if (!map || !tf.valid) throw std::invalid_argument("planner requires map and TF");
    ValidateGridSpec(map->spec); ValidatePose({0,goal});
    const auto start=MapFromBase(tf.transforms);
    ValidatePose({tf.transforms.odom_from_base.stamp_us,start});
    const auto& s=map->spec;
    const int w=s.width,h=s.height,n=w*h;
    if (s.frame!=GridFrame::map || map->cells.size()!=static_cast<std::size_t>(n) || n>1000000)
        throw std::invalid_argument("planner requires map frame and at most one million cells");
    auto cell=[&](const Pose2D& p) {
        const double x=std::floor((p.x-s.origin_x)/s.resolution),y=std::floor((p.y-s.origin_y)/s.resolution);
        return x>=0 && y>=0 && x<w && y<h ? static_cast<int>(y)*w+static_cast<int>(x) : -1;
    };
    const int begin=cell(start),end=cell(goal),margin=static_cast<int>(std::ceil(radius_/s.resolution))+1;
    if (begin<0 || end<0) return {};
    // Conservative square clearance plus one cell covers segment/end-point offsets. Unknown is blocked.
    std::vector<int> prefix((w+1)*(h+1),0);
    for (int y=0;y<h;++y) {
        if (context.canceled()) return {PlanStatus::canceled,{}};
        for (int x=0;x<w;++x) {
            const auto value=map->cells[y*w+x];
            if (value!=0 && value!=205 && value!=254) throw std::invalid_argument("invalid occupancy value");
            prefix[(y+1)*(w+1)+x+1]=(value!=254)+prefix[y*(w+1)+x+1]+
                prefix[(y+1)*(w+1)+x]-prefix[y*(w+1)+x];
        }
    }
    auto clear=[&](int i) {
        const int x=i%w,y=i/w;
        if (x<margin || y<margin || x>=w-margin || y>=h-margin) return false;
        const int x0=x-margin,y0=y-margin,x1=x+margin+1,y1=y+margin+1;
        return prefix[y1*(w+1)+x1]-prefix[y0*(w+1)+x1]-prefix[y1*(w+1)+x0]+prefix[y0*(w+1)+x0]==0;
    };
    if (!clear(begin) || !clear(end)) return {};
    auto heuristic=[&](int i) { return std::abs(i%w-end%w)+std::abs(i/w-end/w); };
    using Entry=std::pair<int,int>;
    std::priority_queue<Entry,std::vector<Entry>,std::greater<Entry>> open;
    std::vector<int> cost(n,std::numeric_limits<int>::max()),parent(n,-1);
    std::vector<bool> closed(n,false);
    cost[begin]=0;open.push({heuristic(begin),begin});
    while (!open.empty()) {
        if (context.canceled()) return {PlanStatus::canceled,{}};
        const int at=open.top().second;open.pop();
        if (closed[at]) continue;
        closed[at]=true;
        if (at==end) break;
        for (int next:{at-1,at+1,at-w,at+w}) {
            if (next<0 || next>=n || closed[next] || !clear(next) || cost[next]<=cost[at]+1) continue;
            cost[next]=cost[at]+1;parent[next]=at;open.push({cost[next]+heuristic(next),next});
        }
    }
    if (!closed[end]) return {};
    auto path=std::make_shared<Path2D>();path->map_revision=map->revision;
    path->start_stamp_us=tf.transforms.odom_from_base.stamp_us;path->robot_radius=radius_;
    for (int at=end;at!=-1;at=parent[at]) {
        if (context.canceled()) return {PlanStatus::canceled,{}};
        if (path->poses.size()>=16382) throw std::length_error("path exceeds 16384 poses");
        path->poses.push_back({s.origin_x+(at%w+0.5)*s.resolution,s.origin_y+(at/w+0.5)*s.resolution,0});
    }
    std::reverse(path->poses.begin(),path->poses.end());
    path->poses.insert(path->poses.begin(),start);path->poses.push_back(goal);
    return {PlanStatus::success,std::move(path)};
}
} // namespace planner
