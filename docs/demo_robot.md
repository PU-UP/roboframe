# demo_robot 示例产品

demo_robot 是 C++14 的可运行示例产品，以持续感知、建图和导航验证框架。建图是该产品的一项流程，不代表整个 integration。Odom 透传原始里程计，SLAM 回放数据集修正位姿，PoseGraph 维护坐标关系与节点；LocalMapBuilder 构建短窗口子图，MapUpdater 生成全局占据栅格。MapServer 保留保存/加载能力。
建图任务复用 SDK 可选 BT，树为 Collect → Drain → Freeze，见 [架构与 BT 边界](architecture.md)。
导航任务已接入 Planner/Controller 并发布速度，见 [导航 demo](navigation-demo.md)。
不接 Brain，不运行真实 SLAM、图优化器或设备运动闭环。

完整组件关系和线程流统一见 [架构图](architecture.md)。下表列出本产品感知/建图组件的实现入口。

| 所有者 | 公开文件 / 当前实现 |
|---|---|
| 原始局部 pose | components/odom/include/odom/passthrough_odom_component.hpp；未来 DRComponent/VIOComponent 可以独立实现，不必塞入这个目录 |
| 修正位姿 | components/slam/include/slam/recorded_slam_component.hpp；同一 scan 与 odom 触发配对的录制参考输出 |
| 定位关系与节点 | components/posegraph/include/posegraph/pose_graph_component.hpp；当前位姿传播、T_map_odom、节点存储和显式历史修正 |
| 子图内容 | components/localmapbuilder/include/localmapbuilder/local_map_component.hpp；原始 scan 引用及局部相对 pose 的短窗口堆叠 |
| 全局栅格 | components/map-updater/include/map_updater/occupancy_map_component.hpp；消费子图与锚点，私有 occupancy_raster 负责光线更新 |
| 地图文件 | components/map-server/include/map_server/file_map_component.hpp；不位于每帧数据转发路径 |

## 数据与执行

1. Python 校验数据来源并把原始 scan、raw odom、稀疏修正位姿分别发送；数据集格式留在适配器。
2. SensorIngress 短锁处理 Odom、PoseBuffer 匹配与有界接纳；录制 SLAM/PoseGraph 在独立定位通道执行。扫描携带匹配时刻的 odom，不在延迟回调中重新查历史。
3. SLAM 输出 MapPose2D；PoseGraph 用同一时刻的 odom 推导 T_map_odom。每次原始 odom 可传播当前全局位姿，不等待地图计算。没有修正或修正过期时输出无效。
4. 有有效定位的扫描注册节点，所有配对成功的 scan 都可传入容量 8 的计算通道；缺少全局 TF 不阻止局部子图。LocalMapBuilder 和 MapUpdater 当前共用这个串行执行组；定位通道也有容量 8；下游满时保留定位结果，产品诊断 pipeline_pending 覆盖两个阶段。
5. LocalMapBuilder 默认每 16 帧开始新子图。integration 在定位修正时间发生变化时也要求新锚点，避免把一次新修正混入旧子图坐标；恢复有效全局定位或重新进入栅格范围也新开锚点；这个策略归当前产品，不写进 SDK。
6. 子图局部原点是首帧 base。每帧保留同一 scan 引用以及 T_local_base；锚点由 PoseGraph 提供。MapUpdater 接收子图快照，正常只积分新增帧，不复制 scan payload。
7. Poll 在业务 owner 上收集计算完成结果，发布地图与任务事件。输入接纳、算法完成、对外发布是三个阶段。

## 指令与生命周期

main 装配产品并绑定产品契约 DemoRobotInputs/DemoRobotOutputs。CONFIG 后是 IDLE；
RUN 启动持续能力，不要求存在建图任务。BEGIN 仅给后续输入附加任务标签，不清定位、图节点或地图。
FINISH 记录任务结果边界，之后输入继续计算；完成时冻结任务最后一张快照，SAVE 保存它。
CANCEL 不改变运行 generation，不关闭队列；旧任务结果仍可更新实时地图，但不回填后续任务。
因此两次 BEGIN 共用持续地图，想从空地图采集应显式 RESET → RUN。

STOP 关闭输入并等待协作取消，RESET 清工作 atlas/定位上下文，RUN 开始新 generation。
已完成的基准快照及磁盘文件保留。QUIT/EOF 最终关闭，不自动保存。
LOAD 要求 IDLE，只读取基准资产，不伪造实时地图、定位或子图历史；加载资产的运行 generation 标为 0。
运行状态变化由 RUNTIME 事件报告，只有正在执行的任务被停止时才有 MAPPING stopped。
完整规则见 [运行骨架](architecture.md)。

## 本产品资源上限

| 资源 | 当前上限 |
|---|---|
| 等待配对 scan / odom 历史 | 64 / 512 |
| 定位 / 建图通道 | 各 8，含完成未领取结果 |
| 单帧 scan | 2048 点 |
| 地图 atlas | 4096 子图 / 400 万点 |
| PoseGraph 节点 | 32768 |

这些是 demo 策略，不是 SDK 默认值；外部共享快照与 IPC 缓存另计，目前没有全进程字节预算。

## 历史修正

PoseGraph::ReviseNode 接收明确的新节点版本；MapUpdater::UpdateNode 重新放置对应子图并重建栅格，旧快照保持不变。
tests/submap_pipeline_test.cpp 直接连接五个组件，验证旧墙消失、新墙出现、旧快照不变、过期版本拒绝和取消。
当前日志没有在线历史优化事件，因此运行时回放只产生当前定位修正，不凭空修改过去节点；历史修正路径在组件组合测试中验证，尚未接外部历史约束流。
重建当前采用完整 atlas 重放；子图整体刚体变换不能修复子图内部形变。增量重建/子图重做在真实需求到来时再实现。

## 运行

```sh
cmake -S . -B build-product -DROBOFRAME_PRODUCT=demo_robot -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tools/mapping/replay.py --dataset tf --scans 0 --binary build-product/robot_runtime --output build-product/submap-result
python3 tests/build_isolation_test.py
```

输出目录必须是新目录。replay.html 可播放地图快照与轨迹，summary.json 记录节点、子图、共享点数、队列峰值和任务统计。
raw 验证原始里程计；corrected 验证 910 个配对关键帧；tf 验证全部 13,631 帧和 910 次稀疏定位修正。
完整数据最初 168 帧没有有效全局参考，仍生成局部子图，仅跳过全局栅格放置；不默认使用单位变换。
地图质量与通道吞吐是不同指标；离线回放按批次排空以避免人为过载。


## 坐标与数据来源

原始日志和修正轨迹的来源、校验值、配对方式见 [数据说明](../data/intel-lab/README.md)。
raw odom 不被修改；RecordedSlam 只按采集时刻提供录制的修正 pose，不声称运行在线 SLAM。

T_parent_child 把 child 坐标转换到 parent；单位为米/弧度，时间为同一采集时钟的微秒：

- Odom：T_odom_base；参考 SLAM：T_map_base。
- PoseGraph：T_map_odom = T_map_base × inverse(T_odom_base)，必须使用同一时刻的 odom。
- 当前全局 pose：有效的历史 T_map_odom × 当前 T_odom_base，不使用未来修正；过期则无效。
- 子图首帧定义 local 原点，内部存 T_local_base；全局积分使用 T_map_node × T_local_base。

节点/子图 ID 只在本次运行 generation 内有效；revision 表示更新版本。
子图是追加前缀，旧帧 scan/pose 不可悄悄修改；历史节点修正必须提升版本。
加载栅格不能恢复 SLAM/atlas，也不能当成已经重定位。
MapServer 的 RFGRID1 表示旧 odom 坐标地图；RFGRID2 显式记录 map 坐标标签。
两者都可读取，但不能把旧格式直接当作 map 坐标资产。

Scan2D 保存已经转换到 base 坐标系的有效二维命中点；Python 负责数据集角度、外参与配对。
当前光线原点为 base，无返回量测被省略；非零传感器原点、逐束时间、去畸变需要更丰富的契约。
当前 20 米过滤属于这个示例的输入限制。地图质量不是框架正确性的证明，也不能据录制轨迹验证控制闭环。

## 验收入口与已知限制

- ctest：36 项（包含 Server、BT、Planner、Controller、异步定位与导航 integration 测试），覆盖 SDK 双执行组、模板、共享快照、并发输入、任务边界、慢读、断连和关闭。
- mapping_execution_ipc：320 帧真实数据，50 ms 延迟下无任务/取消后继续运行；
  mapping_execution_test 验证 5 秒注入延迟可协作取消，停止时间要求小于 500 ms。
- tests/build_isolation_test.py：八组件独立构建安装、纯包/混合构建、产品选择隔离、
  协议适配无 integration 依赖、安装模板与公开头独立编译。
- 完整 tf 回放：13,631 scan/odom、910 次修正；最初 168 帧无全局定位但仍构建局部子图。
  FINISH 后继续更新，冻结地图、IPC 读取与保存/重载逐字节对比。
- 最近导航与输出验证记录集中在 [导航 demo](navigation-demo.md)；本页不保留逐轮构建目录日志。

验证环境为 WSL Ubuntu 22.04 / x86_64 / C++14。离线回放分批排空，不代表实时吞吐；
历史修正尚未接入运行时输入流，当前完整 atlas 重放也不解决子图内部形变。
ARM、真实 VIO/SLAM、ROS/DDS 和实机时限未验证。后续统一维护在 [TODO](../TODO.md)。
