# 产品集成中的建图流程
当前产品只有一个 robot_runtime。RobotIntegration 持有组件；建图是其一个流程，
不是一种进程身份。旧实验的独立建图入口和数值 demo 已移除。

## 模块与数据路线
~~~mermaid
flowchart LR
    R[外部日志回放] -->|ODOM / SCAN / 指令 IPC| I[StdioInterface]
    I --> N[RobotIntegration]
    N --> O[PassthroughOdomComponent]
    O -->|TimedPose2D 同步输出| D[integration 数据流和 PoseBuffer]
    N -->|只读 Scan2D| D
    D -->|同时间戳 Scan2D + Pose| U[OccupancyMapComponent]
    N -->|快照和地图名| M[FileMapComponent]
    U -->|只读 GridSnapshot| N
    N -->|同一 scan + 地图快照| P[ReadOnlyPlannerComponent 独立线程]
~~~

| 文件 | 职责 |
|---|---|
| interface/stdio_interface.hpp | 传输对象；解析、校验、编码回执 |
| main.cpp | 创建并持有 integration，把引用交给 stdio |
| integration/robot_integration.hpp | 产品公开接口，仅依赖 SDK |
| integration/robot_integration_impl.hpp | 私有组件所有权、接线和建图会话状态 |
| integration/robot_integration.cpp | 构造、兼容性检查、启停 |
| integration/data_flow.cpp | 转发数据、等待同步、调用地图更新 |
| integration/pose_buffer.hpp | integration 私有缓存工具，没有组件生命周期 |
| integration/mapping_flow.cpp | StartMapping、FinishMapping、CancelMapping、地图操作 |
| components/odom/include/odom/passthrough_odom_component.hpp | 透传位姿操作契约与组件；数据类型在 SDK |
| components/map-updater/include/map_updater/occupancy_map_component.hpp | 更新结果、组件输入策略和占据图组件 |
| components/map-server/include/map_server/file_map_component.hpp | 地图保存和读取组件 |
| sdk/include/roboframe/geometry2d.hpp / grid.hpp / scan2d.hpp | 跨模块位姿、只读地图与扫描类型 |
| components/planner/include/planner/read_only_planner_component.hpp | 受限异步读取和一致性验证，不做路径规划 |
| sdk/include/roboframe/grid_cells.hpp | 分块写时复制，旧快照和未变化块共享 |
| sdk/include/roboframe/mapping.hpp | interface 使用的 MappingOptions、MappingStatus 和 MappingPhase |

数据入口与产品流程在一个 integration 中，但分开实现。
Continuous odom 不使用 Action 状态。建图的 idle/mapping/ready 属于产品会话；
组件只维护自己的 lifecycle。以后新产品可以提供自己的 integration，
而复用 SDK 和能力组件。其他业务不必重启 integration。

## 各阶段
1. main 先创建未配置的 integration 并注入 stdio；进程输出 READY roboframe-v1。
   CONFIG 调用 Configure，成功才提交组件装配；参数或资源初始化失败允许重试，成功后不允许重复配置。
2. BEGIN id 调用 StartMapping，清空本次占据图和待匹配 scan，保留持续 odom 数据流。
3. ODOM 经透传组件输出到 PoseBuffer；SCAN 在 integration 排队（最多 64）。
   精确时间戳优先；否则只在合法时间间隔内插值，绝不挑最新 pose 或外推。
4. 配对成功调用 OccupancyMapComponent::PushScan。scan 只读，组件不修改输入。
5. FINISH 处理剩余队列，未配对的计为丢弃；非空地图形成基准快照，状态 ready，实时更新继续。
6. SAVE name 保存基准快照；后续 scan 更新实时地图，不改变已交付的旧快照。
   integration 按数据时间最多 10 Hz 发布地图；planner 同时读取这份地图和同一个 scan。
7. LOAD name 可在未建图时加载地图并交给 planner；加载后的地图只读，不自动恢复 evidence 更新状态。
   BEGIN 新 id 可在同一进程再次建图。
8. CANCEL 结束本次会话；QUIT/EOF 调用 integration 关闭组件，不隐式结束并保存地图。
   main 持有 integration；异常退出或忘记显式关闭时，析构负责清理。

## IPC
每行不超过 65536 字节，stdout 只用于协议，stderr 为日志。
所有请求先完整解析再执行；非法参数返回 ERROR；输出通道断开时退出。
接口串行，尚无网络安全、并发或多客户端语义。

~~~text
CONFIG width height resolution origin_x origin_y max_pose_gap_us
BEGIN session_id
ODOM stamp_us x y yaw
SCAN stamp_us count x1 y1 ... xn yn
FINISH
CANCEL
SAVE name
SNAPSHOT name
LOAD name
STATUS
RESOURCES
WAIT_PLANNER
QUIT
~~~

BEGIN/FINISH/CANCEL 是当前建图命令，语义不等同组件启动/停止。
SNAPSHOT 用于保存当前更新中的地图，在 FINISH 后也可调用，不改变基准地图。SAVE/LOAD 名字只允许字母数字、下划线、连字符，
不接受路径，不覆盖已存在文件。STATUS 为建图会话查询：
phase / session / received / integrated / ignored / dropped / pending /
used_rays / outside_rays / map_revision / drop_sync / drop_queue / drop_bounds / drop_cancel。
map_revision 指基准地图。RESOURCES 另行报告实时更新、块复制量及 planner 的读取/替换计数，
避免后台更新改写已完成建图的成功/丢帧计数。WAIT_PLANNER 发布最新版本并等待读取结束，用于验收。

## 后续规划用的参考地图
新增 `--dataset corrected`，使用发布者配套校正日志，仍经过同一 IPC/component 链路。
来源、下载指纹、坐标语义和规划使用边界见 [Intel 数据说明](../data/intel-lab/README.md)。
原始模式及历史结果保留；下节描述的是 raw 模式，corrected 模式不是原始 odom 或实测真值。

## 原始数据与地图验证
使用 history/2026-10-09-full-prototype/data/intel-lab/intel.log.gz。
SHA256：449a9813e9709b6fd053ba0d006ea69c5f4485ba08dd021448e653e3af2e6ff9。

该日志有独立 ODOM 与 FLASER 自带的原始 odom。回放使用后者原值，
在相同时间戳分别发送 ODOM 与 SCAN。没有位姿修正、扫描匹配或生成轨迹。
组件仍是分开的 typed 输入，产品同步逻辑仍支持真实异步流，并有插值/丢帧测试。
这项调整消除了回放中额外插值和无必要丢帧，但没有改变原始坐标。

180 束角度沿用 [GMapping 旧 CARMEN 读取实现](https://github.com/ros-perception/openslam_gmapping/blob/master/log/carmenconfiguration.cpp)：
-90..-1,+1..+90 度。该旧日志没有完整传感器标定元数据，因此这不是重新标定。
过滤小于 0.01m 和大于等于 20m 的读数（无返回不当作障碍点）；本数据的 laser/base 位姿相同，
非零外参在适配层明确拒绝，不能直接拿到其他设备上使用。

20 米过滤只删除读数，不把远点裁剪成 20 米障碍，也不移动保留点；已有回归测试验证这一点。
它会减少观测覆盖，不能仅凭这个阈值解释坐标重叠。错误坐标系、重复变换或忽略非零外参
确实可能造成错图；当前数据的同帧 laser/base 位姿相同，单帧投影检查也通过，
所以目前没有证据把上述 Scan2D 表达方式认定为重叠主因。全局重叠仍未解决，按用户要求暂停。

修复的 occ 问题：原来每条射线立即更新，同一帧里的空闲射线可反复擦除已命中的格。
现在每帧每格只更新一次，占据优先，与射线排列顺序无关。
不是把冲突隐藏掉：不同帧仍分别累计证据，所以互相矛盾的输入观测仍会形成重叠。

验证分开报告：
- CTest 验证同步、业务流程、IPC、不可变快照、存储和生命周期。
- tests/intel_geometry_check.py 从完整日志选取 12 帧，用独立的极坐标公式计算
  世界坐标落点，与每帧实际 IPC 地图的全部占据格逐一相等比较。
- 全量回放 13,631 scan；第 12,267 帧后结束建图，12,267 帧融合、0 丢弃；
  后续 1,364 帧继续更新实时地图；基准快照不变，保存与进程重启重载一致。
- 单帧投影正确不意味着全局地图几何一致。全量原始位姿叠加仍有整体重叠，
  本轮不声称已经得到正确闭合的实验室地图，也不把它归因于某个未经验证的单一原因。

~~~sh
python3 tests/intel_geometry_check.py build-product/robot_runtime
python3 tools/mapping/replay.py --scans 0 --finish-after 12267 --output build-product/full-result
~~~
输出目录不可覆盖。replay.html、final.png、summary.json、输入 gzip 和回执 JSONL
保留真实运行结果；summary 的 passed 只表示上述业务链路断言通过，不代表定位精度。

## 2026-10-10：与 history 的同条件对照

之前把 history 的短片段图与当前全量图直接比较，不足以判断重构是否造成几何退化。
本次使用未修改的归档 intel_replay 和其原有 BT 动态库重新运行，未修改位姿、外参或 scan 角度。

| 对照条件 | history | 当前 |
|---|---|---|
| 短片段 | 输入 600 帧，第 480 帧结束，建图约 94.0 秒 | 完全相同 |
| 长片段 | 输入 13,631 帧，第 12,267 帧结束，建图约 2,423.0 秒 | 完全相同 |
| 原始占据地图 | 导出的 intel.rfmap，不膨胀 | final.rfgrid，不膨胀 |

证据：
- 13,631 帧经新旧 Python 解析器得到的时间戳、位姿、全部点坐标逐项完全相同。
  两者量程下限写法稍有差别，但在该数据中没有造成任何输入差异。
- 重跑旧版短片段的 intel.rfmap 与归档文件逐字节一致，确认历史基线可复现。
- 同条件短片段的新旧图轮廓一致；已知区域交并比 98.05%。
- **旧版运行长片段同样出现全局扭曲、重叠**；新旧长片段已知区域交并比 98.59%。
  该指标衡量覆盖差异，不衡量地图几何精度。
- 旧页面 map.svg 来自额外做过 0.2 m 膨胀的导航地图；当前 PNG 是未膨胀占据图。
  对照页使用原始占据图，也单独提供旧版膨胀图，避免比较不同图层。
- 旧版 insert_rays 用本帧射线直接覆盖格值；当前 OccupancyMapComponent 跨帧累积证据，
  因此墙线、噪点、占据格数量并不完全相同。这不解释两者共有的整体错位。

结论：本次显示差异主要来自建图区间和显示图层，未发现重构引入 scan/位姿解码变化。
这不代表原始位姿就是真值，也不证明两版共有的角度/标定假设正确。
此前单帧投影测试只能验证给定约定下的计算；不应将其写成实际传感器标定或全局地图验收。
若继续解决长片段几何问题，应独立检查原始位姿与重复观测的一致性，以及采集设备的角度/标定信息；
不能为了接近短片段外观而修改外参、隐藏后续帧或改变 odom。

本地对照结果：build-product/geometry-compare/index.html；机器数据 comparison.json、parser-check.json。
运行产物保留在 build-product，不参与源码构建。

复现回放（输出目录需使用新名字，避免覆盖结果）：
~~~sh
python3 tools/mapping/replay.py --scans 600 --finish-after 480 --output build-product/history-prefix-compare
LD_LIBRARY_PATH="$PWD/history/2026-10-09-full-prototype/build/_deps/roboframe_bt-build" \
python3 history/2026-10-09-full-prototype/examples/intel_demo.py \
  --scans 13631 --finish-after 12267 --output build-product/history-full-compare
~~~
归档动态库只用于这次对照，不是当前框架的新依赖。

## 边界与兼容
C++14、静态库；integration/写图串行，planner 单独读取。没有 BT、线程池或插件注册器。
地图固定 odom 系，尺寸预配置；越界射线跳过，机器人出界整帧拒绝。
FileMapComponent 保留原 RFGRID1 格式与校验和，可读取旧地图；
文件 fsync 不等于目录 fsync，仍不承诺断电持久化。
包名和公开类名已改变，应使用新的构建/安装前缀，勿混用旧头文件和旧库。
真实异步图像/模型融合不在本次实现范围。

类型归属与 SDK 扩展规则见 [SDK 说明](../sdk/README.md)；所有延期项见 [统一待办](../TODO.md)。

SDK 发布版本由 sdk/VERSION 管理；产品选择集中在 cmake/Dependencies.cmake。
组件源码没有手写 VERSION；实际编译 SDK 和源码指纹自动写入库与安装清单，
启动时输出到 stderr，不改变 IPC。integration 不再指定组件契约版本。
组件依赖只存在于 integration 私有实现中；独立编译验证会实际移除组件与私有头文件，
检查 interface 和 main 只靠 SDK 与公开入口能否通过编译。
