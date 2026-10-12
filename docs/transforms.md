# 两级 TF 与 lidar-map 数据链路

这是固定的二维 base_link / odom / map 契约，不是通用 TF 树或 ROS 依赖。
`T_parent_child` 将 child 中的点变换到 parent。用户描述的 base_link→odom 对应 T_odom_base，
odom→map 对应 T_map_odom；组合为 `T_map_base = T_map_odom * T_odom_base`。
常见树画法 map→odom→base_link 表示父子层级，不要把它与点坐标变换方向混淆。

## 分工与文件

| 层/组件 | 负责内容 | 文件 |
|---|---|---|
| SDK | TimedPose2D 只表示 T_odom_base；MapPose2D 表示 T_map_base；MapCorrection2D 表示 T_map_odom；ScanTransforms2D 携带完整两级链 | sdk/include/roboframe/geometry2d.hpp、transforms2d.hpp |
| odom | 接收已有 raw odom，校验后同步原样发布 T_odom_base | components/odom/include/odom/passthrough_odom_component.hpp |
| lidar-map | 同时接收只读 scan 与同采集时刻的 odom；有录制参考 pose 的帧才发布 T_map_odom | components/lidar-map/include/lidar_map/recorded_lidar_map_component.hpp |
| integration | 缓存/插值 odom，分发同一 scan，保留最近已发布的有效校正，向 mapper 交付 ScanTransforms2D | integration/data_flow.cpp、pose_buffer.hpp |
| map-updater | 用完整 TF 投影 scan，更新导航占据图；不估计定位、不依赖 lidar-map 的私有类型 | components/map-updater/include/map_updater/occupancy_map_component.hpp |
| map-server / planner | 保存有坐标归属的快照 / 有界异步读取共享地图与 scan | 对应具名 component 头文件 |
| interface | SDK 输入的 IPC 适配，不做 TF 计算 | interface/stdio_interface.cpp |

~~~mermaid
flowchart LR
  raw[raw odom] --> I[interface]
  scan[scan] --> I
  ref[录制 map pose / 后端替身输入] --> I
  I --> G[integration]
  G --> O[odom]
  O --> P[按 scan 时间取 T_odom_base]
  P --> L[lidar-map]
  G -->|同一 scan 引用| L
  G -->|参考 pose| L
  L -->|低频 T_map_odom 回调| T[保留有效校正]
  P --> U[map-updater]
  T -->|完整 TF| U
  G -->|同一 scan 引用| U
  U --> M[不可变导航地图快照]
  M --> R[只读 planner / map-server]
~~~

RecordedLidarMapComponent 是独立 SOURCE/PACKAGE 组件，只依赖 SDK，可自己写 main 接入。
它不创建地图：参考位姿通过 PushReferencePose 注入，内部仅在对应 scan 到达时计算
`T_map_odom = T_map_base_reference * inverse(T_odom_base)` 并同步回调。
它不运行扫描匹配/回环，不能声称真实 SLAM。将来可替换为真正激光 SLAM，并自行保留 SLAM 内部地图；
导航占据图/costmap 仍由 map-updater 负责，不必等同于 SLAM 内部地图。

## 时序和所有权

- Python 按 180 束完整量程的唯一匹配关联 raw/corrected，不按文件行号或不同精度时间戳硬配。
  引用校正值时保留 raw 的采集时间；原始 odom 数值不变。仅地图范围预估与图上轨迹在 Python 计算，
  实际用于建图的校正来自 C++ lidar-map，回放通过 TF 查询独立验证。
- 新校正仅在同帧 scan/odom 配齐后生效。后续 scan 使用本帧 odom + 最近的过去校正，
  校正自身时间戳保留，不把它伪装成每帧新估计；不插值未来校正。
- MAP_TF 模式没有首个校正或校正过期就丢弃该帧，计入 tf_dropped，不回填或用单位变换冒充已定位。
  回放使用 20 秒有效期以覆盖该日志最长约 13.782 秒的校正间隔；这是离线验证配置，不是实机默认安全时限。
- 参考 pose 要先于对应 scan 输入；迟到引用明确拒绝。参考队列最多 64 个小位姿，scan/odom 等待队列仍有界。
  被丢弃 scan 对应的引用在后续扫描到达时清理，并记录 missed_references；不无限积压。
- odom 同步等待由 integration 负责；lidar-map 与 mapper 不各存一份大 scan。两者顺序读取同一个 Scan2DPtr，
  planner 在地图发布时获得同一对象；payload 不复制。TF 只按值复制几个标量，没必要给小结构套 shared_ptr。
- 新建图会话、取消、加载、关闭会清理对应校正上下文。加载 map 不会产生 TF 或冒充定位成功。
- component 的发布是同步 typed 回调；integration 的具名输出经 interface 变为主动事件，详见 [双向 IPC](ipc.md)。
  TF 查询/事件表示最近处理 scan 的变换，不代表当前墙钟时刻的定位健康状态。odom 独立随输入发布。

## 协议与持久化

以下是命令体，stdio v2 需加 REQ ID（传感器也可用 DATA），见 [线协议](ipc.md)。
CONFIG 无 MAP_TF 尾部时表示 odom-frame 本地堆叠：
~~~text
CONFIG width height resolution origin_x origin_y max_odom_gap_us MAP_TF max_correction_age_us
MAP_POSE stamp_us x y yaw
TF
~~~
MAP_POSE 是录制后端测试输入，不能作为未认证外部定位自动接管的产品策略。
TF 返回 scan_stamp correction_stamp，随后依次是 odom_from_base / map_from_odom / map_from_base 的 x y yaw；
没有有效链时返回 `TF unavailable`。RESOURCES 增加 lidar_scans、correction_outputs、missed_references、tf_dropped。

GridSpec.frame 显式区分 odom/map。map-frame 文件写 RFGRID2，校验覆盖坐标标签；
odom 文件继续写 RFGRID1，旧 RFGRID1 读取为 odom。产品拒绝加载不同坐标归属的地图，
不能把旧版未标坐标的 corrected 产物静默当成新 map 资产；请用本版重放生成。
尚无跨多个地图的身份与坐标变换注册，后续工作模式再明确地图 ID/修订号与任务绑定。
SDK 4.0 改变 GridSpec 和 mapper 输入契约，旧组件开发包必须重建；仍不需要开发者维护组件版本号。

## 运行与验收
~~~sh
cmake -S . -B build-product -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
python3 tools/mapping/replay.py --dataset tf --scans 0 --finish-after 12267 --output build-product/tf-mapping
~~~
输出目录需用新名称。`--dataset corrected` 现在也走同一 TF 链，但只回放匹配的 910 帧；
`--dataset tf` 则回放全部 13,631 帧 raw scan + sparse reference。两者都不把 corrected pose 塞进 odom 槽。

本次全量：910 次校正全部消费，首次定位前 168 帧跳过，融合 13,463 帧；结束建图后继续融合 1,364 帧。
1,038 次 TF 数值检查（包含全部 910 次参考校正），最大位置误差约 3.4e-14 米，这是计算一致性，不是定位精度。
planner 不一致读取为 0，最多保留两项；固定快照不变、保存/重载一致。回放页面与 tf-trace.json 保留证据。

12 项 CTest 覆盖旋转/平移组合、异步 odom、首次定位、过期、未来引用、重置、地图坐标落盘等；
ASan/UBSan 通过；五个组件独立 SDK-only 构建、安装包/混合构建和公开头文件隔离通过。

限制：校正间零阶保持可能留下局部重影。map-updater 不会在回环/大校正后自动重投影历史栅格；
未来按地图用途选择局部滑窗清理或重建全局导航图，不能把变换更新等同于历史地图已经校正。
非零雷达外参/逐束运动补偿仍需扩展 scan 契约；此次没有实现在线 SLAM、真实 planner/controller 或 BT。
