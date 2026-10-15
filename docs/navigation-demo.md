# 地图 → Planner → Controller → 速度输出 demo

2026-10-15。已将 PlannerComponent / ControllerComponent 接入 SDK Server 骨架，运行于 demo_robot，保持 C++14。
本 demo 发布速度但没有驱动或运动仿真消费者；地图来自真实录制数据，导航时 TF 是明确提供的静止测试输入。

架构、六个线程和新产品分工见 [统一架构图](architecture.md)；
Server 接口见 [SDK](../sdk/README.md#server-与可选-bt)，命令格式见 [IPC](ipc.md#导航扩展)。

## 地图、路径与 TF 的约定

- 当前导航使用 FINISH/LOAD 得到的基准地图 `frozen_`；不直接用尚未完成的实时建图快照。没有基准地图时拒绝导航。
- 地图、目标、路径统一为 map 坐标，位姿使用 `T_map_base = T_map_odom × T_odom_base`。速度为 base 坐标的前向 m/s 和逆时针 rad/s。
- Planner 捕获同一次请求的地图快照、TF 和目标；路径记录 map_revision、起点采集时间、frame、机器人半径。Controller 安装路径时核对规划地图关联。
- 默认半径 0.2 m，仅是 demo 参数。未知格禁止通行；用 `ceil(radius/resolution)+1` 格的保守方形余量检查自由空间，不修改共享原图。
- 四邻域搜索避免斜穿障碍角；路径保留实际起点、格中心与目标姿态。最多 100 万地图格、16,384 路径点，超限明确失败。
- Controller 使用最新基准地图检查当前位置与下一目标点间的空间，地图几何变化、局部阻塞或离当前路径点超过 1 m 时停止并失败。这里没有实时障碍感知或局部 costmap。
- TF 同时检查 PoseGraph 的采集时钟有效性和 ControllerComponent 的接收新鲜度：默认 500 ms 没有成功接收新里程计即失败并清零速度反馈；输出回调和发送均不依赖主线程。不会把历史采集时间直接与系统墙钟相减，也不接受早于路径起点的控制 TF。

Controller 用当前朝向算目标方位，先原地转向，再前进；限制为 0.3 m/s、1 rad/s。
位置误差门限为 min(0.05 m, 地图分辨率/4)，最终朝向误差小于等于 0.05 rad 才返回 reached。
没有消费速度时，TF 不会自己移动，任务不会自行成功。

## 任务与业务规则

导航为 planning → following → succeeded / failed；取消可经过 canceling → canceled。
一次只接受一个目标，新 session 必须递增；新任务必须等上一个任务清理完成。
NAV_CANCEL 不改变运行 generation，不停止定位/建图流；规划失败只结束本次导航。
建图可并行，但导航只使用基准地图；新的基准使路径不可用时明确失败，不隐式重规划。

任务保存 Planner 返回的不可变路径，再交给 Controller。
Controller 默认 50 ms 周期，500 ms 未收到有效里程计则归零并失败。
取消、结束、TF 失效或异常均由控制线程输出零速度；BT 之后领取结果并更新任务状态。
主线程暂停不阻止控制与发送；共享管道慢读仍可延迟发送，断连不保证送达。

导航活动期间仍拒绝 SAVE/SNAPSHOT/GET_MAP 的同步文件 I/O 或完整编码；LOAD 要求 IDLE。

## 运行和验收

```sh
cmake -S . -B build-server -DROBOFRAME_PRODUCT=demo_robot -DCMAKE_BUILD_TYPE=Release
cmake --build build-server -j4
ctest --test-dir build-server --output-on-failure
python3 tools/mapping/replay.py --dataset corrected --scans 0 --finish-after 909 \
  --binary build-server/robot_runtime --output build-server/reference-map
python3 tools/navigation/demo.py --map build-server/reference-map/live.rfgrid \
  --binary build-server/robot_runtime --output build-server/navigation-demo
python3 tests/build_isolation_test.py
```

两个输出目录均须不存在。demo 复用现有离线连通性检查选择有余量的起终点，实际路径由 C++ Planner 重新规划。
它注入非单位 map/odom TF，保持机器人静止，仅观察速度，不用速度积分位置。
验证取消和停止输入后的零速度、限幅、路径全部处于保守自由空间，以及未运动时不会报告成功。

验证用例：[导航任务](../tests/navigation_test.cpp)、[独立控制输出](../tests/control_output_test.cpp)、
Planner/Controller 组件测试。主线程阻塞测试直接读取真实管道，不通过 Poll 代发。

统一组件后的验证：36 项回归、10 项 UBSan、八组件独立安装/混合构建及 36 个公开头独立编译通过。
记录分别见 build-server/ctest-component.log、build-server-ubsan/ctest-component.log 和 build-server/isolation-component.log。
build-server/navigation-component 在 910 帧录制地图上得到 468 点、46.5 m 路径，取消及 TF 超时归零。
算法与执行测试均通过统一组件的 server()，没有第二套算法 Component 或兼容包装。

## 已知限制

仅验证简单导航接线，没有真实运动闭环、在线 SLAM、动态障碍、自动重规划/恢复、
速度平滑或设备停止确认。TF 为明确注入的静止输入，持续提供它时任务可一直 following。
算法参数、周期和超时均为演示值；ARM、硬实时与实机控制未验证。
TSan 在当前 WSL 环境因 unexpected memory mapping 未能运行，不能声称并发检查通过。
后续工作统一见 [TODO](../TODO.md)。
