# OccupancyGrid

C++14 / SDK >=1.1、相同 major；公开 target 为 OccupancyGrid::component。
只依赖 SDK。不依赖 OdomBuffer；调用者提供同一时刻的 TimedPose2D。

Scan 的输入是 base 坐标系二维激光命中点，时间与 odom 同源，单位为米。
共享指针为 shared_ptr<const Scan>；生产者交出后也不得通过别名修改该帧。
数据适配层负责激光外参和无效量程过滤。当前只支持 <=20m 的真实返回，
没有把无返回数据伪造成障碍或空闲射线。

构造 GridSpec 定义固定栅格；Activate → Reset → PushScan → Snapshot。
PushScan 明确要求 pose.stamp_us == scan.stamp_us，组件内不做隐式配对。
Bresenham 射线累积证据：空闲 -1、命中 +4，范围 [-30,30]。
快照阈值为 occupied >=3、free <0，其余 unknown；这是可替换的参考算法。
机器人在图外拒绝整帧；端点在图外跳过该射线并计数，不在边界造假墙。

Snapshot 生成一次独立只读地图；之后继续更新或 Reset 不改变既有快照。
没有每帧复制整张地图，没有业务 Begin/Finish/Save，也没有伪造的 Action。
地图坐标就是建图 odom；没有 SLAM 回环、坐标纠正、导航膨胀或重定位。

~~~sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/sdk
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /path/to/grid-package
~~~

tests/check.cpp 是不依赖 integration 的独立 main，验证射线更新、时间约束与快照隔离。
算法替换留在本目录，业务流程与同步策略不应塞进这里。
