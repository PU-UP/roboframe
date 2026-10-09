# 基于最小骨架的建图实现与验收

基线分支 codex/initial-framework 仍指向 dd998ab，没有被合并或改写。
开发全部位于 codex/mapping-pipeline。此前的历史数据与构建产物不进入 Git。
分阶段计划见 [mapping-development.md](mapping-development.md)。

## 先看结果

已用 history/2026-10-09-full-prototype/data/intel-lab/intel.log.gz 完成真实数据回放：

| 项目 | 全量验收 |
|---|---|
| 输入 | 13,631 scan / 26,915 ODOM |
| 显式结束建图 | 第 12,267 帧之后，剩余数据仍继续输入 |
| 实际融合 | 8,758 帧 |
| 时间匹配拒绝 | 3,509 帧，缺少合格的 odom 时间配对 |
| 队列溢出 / 图外机器人 / 图外射线 | 0 / 0 / 0 |
| 结束后忽略扫描 | 1,364 帧 |
| 使用射线 | 1,525,716 条 |
| 地图 | 1085 × 990，0.1 m/cell |
| 已知 / 占据格 | 159,830 / 15,583 |
| 结束后地图不变 | 保存前后逐字节一致 |
| 新进程重载再保存 | 逐字节一致 |

验收文件：
- build-mapping/full-result/replay.html：可播放、拖动的真实检查点地图和轨迹。
- final.rfgrid / final.png：程序保存的地图与直接转换的预览。
- progress-*.rfgrid / progress-*.png：建图过程中实际保存的检查点，不是事后伪造增长。
- summary.json：机器可读统计、校验值、冻结与重载结论。
- mapping-input.txt.gz：实际发送的完整 IPC 指令与数据。
- mapping-events.jsonl：每条请求对应的回执、原始数据行号与原始时间。
- mapping-stderr.log：组件激活、任务阶段与保存日志。
- reload-*：独立新进程的加载/保存记录。

**passed 只说明数据/业务/保存链路通过，不表示所有扫描都被接受，更不表示 SLAM 精度达标。**
此处为 raw odometry + scan 的占据建图，没有扫描匹配和回环修正，完整地图会保留里程计漂移。

## 构建与运行

默认构建仍保持原有累加器骨架；建图产品显式启用：

~~~sh
cd /home/watermango/work/roboframe
cmake -S . -B build-mapping -DROBOFRAME_BUILD_MAPPING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-mapping -j4
ctest --test-dir build-mapping --output-on-failure

# 默认 3000 帧，2400 帧后结束；输出目录必须不存在。
python3 tools/mapping/replay.py --output build-mapping/my-run

# 完整数据，复现上述验收。
python3 tools/mapping/replay.py --scans 0 --finish-after 12267 \
  --output build-mapping/my-full-run
~~~

输出目录已存在时拒绝覆盖。直接打开生成的 replay.html 即可，页面不依赖外部服务。
需要浏览器 URL 时，在根目录启动 python3 -m http.server 8766 --bind 127.0.0.1。
不要把输出目录里的地图或输入日志误当成代码；这些都在忽略的 build 目录下。

--max-odom-gap-ms 可调整最大 odom 括号间隔，默认 250；增加阈值改变数据可信度策略，
并不是提高定位精度。完整原始 ODOM 相邻间隔最大约 3.107 s，不能假设始终 10 Hz。
回放程序保留微秒时间，离线排序修复了文件中的 602 处 ODOM 倒序和 631 处 scan 倒序。
运行时依然要求每个输入流严格递增，不偷偷重定时，也不取最新 odom 冒充匹配。

## 架构、依赖与职责

~~~mermaid
flowchart LR
  D[Intel 原始日志] --> P[Python 格式适配]
  P -->|独立 ODOM / SCAN / 指令| I[MappingStdio]
  I --> N[MappingIntegration]
  N --> O[OdomBuffer]
  O -->|At 扫描时间| N
  N -->|只读 Scan + 同时刻 Pose| G[OccupancyGrid]
  G -->|只读 GridSnapshot| N
  N --> S[MapStore]
  S --> F[RFGRID1 文件]
~~~

| 文件 / 模块 | 职责 |
|---|---|
| mapping_main.cpp | 创建宿主、选择存储目录、异常退出；没有算法逻辑 |
| interface/mapping_stdio.hpp | 有界 IPC、字段检查、调用 integration、编码回执 |
| integration/mapping.hpp | 产品流程接口、组件所有权与有界待匹配队列 |
| integration/mapping.cpp | 装配、版本核对、启停、Begin/Finish/Cancel/Save/Load |
| integration/mapping_data.cpp | odom/scan 路由、时间匹配、明确丢弃 |
| components/odom-buffer/include/odom_buffer/component.hpp | 有界里程计历史、插值查询；不估计 VIO/LIO |
| components/occupancy-grid/include/occupancy_grid/component.hpp | 连续占据栅格更新；不维护产品建图流程 |
| components/map-store/include/map_store/component.hpp | 地图文件保存/加载与完整性；不判断产品状态 |
| sdk/include/roboframe/geometry2d.hpp | 真正共用的 Pose2D、TimedPose2D 与单位约定 |
| sdk/include/roboframe/grid.hpp | 真正共用的地图快照与栅格约定 |
| tools/mapping/replay.py | 外部数据适配、产品指令、验收与报告生成；没有建图算法 |

每个组件的接口、实现、CMake、README 和直接调用组件的测试 main 都在自己的目录。
三个组件相互不包含彼此的头文件。只有 integration 同时认识三者并负责连接。
SDK 不包含任何具体组件。Scan 属于占据栅格输入契约，放在组件内，不再集中堆积所有 contract。

为什么没有一个“mapping component”：
Begin/Finish/Save 是这个产品的流程；栅格算法只做 Reset/PushScan/Snapshot。
它没有等待 Goal 的任务语义，所以不强制套 Action。未来真正的 SLAM 后端可以另设组件，
其估计/建图能力要按算法实际输入输出定义，不能把这个业务流程类换个名字塞进去。

## 业务线与模块阶段

| 阶段 | 指令 / 结果 | 参与模块 |
|---|---|---|
| 宿主启动 | READY mapping-v1 | interface 可用，尚未配置组件 |
| 配置 | CONFIG → OK CONFIG | 检查 SDK/组件版本，激活 store、odom-buffer、grid |
| 启动建图 | BEGIN id → OK BEGIN | integration 进入 mapping；grid 重置，历史 odom 保留 |
| 输入数据 | ODOM / SCAN → OK | odom 入缓存；scan 等匹配，符合时间条件后更新 grid |
| 检查点 | SNAPSHOT name → OK | 复制当前地图快照，store 保存；继续 mapping |
| 提前结束 | FINISH → OK FINISH | 能匹配的队列先处理，其余计数丢弃；冻结快照，进入 ready |
| 正式保存 | SAVE name → OK SAVE | store 完成文件写入与原子发布；失败仍保留 ready 地图，可换名重试 |
| 后续数据 | ODOM / SCAN | odom 继续更新；scan 只计 ignored，不修改冻结地图 |
| 新进程加载 | LOAD name → OK LOAD | store 校验文件，integration 得到只读快照并进入 ready |
| 退出 / EOF | QUIT → BYE / EOF | 放弃未完成会话，清队列、停用组件；不会暗中 Finish 或 Save |

Begin 的 session id 必须递增；忙碌时拒绝新的 Begin。
新的 Begin 是明确开始新地图，会丢弃 integration 当前持有的旧地图引用；
已经保存的文件与其他读者持有的旧快照保持有效。
Cancel 丢弃未完成会话，不生成成功地图；空地图不能 Finish。
Load 失败不替换既有地图；正在建图时不能 Load。

FINISH 只表示内存地图完成；SAVE 回执才表示文件完成发布。
两者都不是机器人停车确认；这次没有执行器，也不伪造停车信号。
组件生命周期与产品阶段分开：Finish 后仍 active，只有业务数据门禁改变。

## 数据线与所有权

1. Python 分别解析真实 ODOM 与 FLASER，不用 FLASER 同行 pose 冒充独立 odom。
2. 激光过滤无效、0、>=20m 返回，沿用归档适配器的 -90..-1,+1..+90 度约定；
   当前文件的激光/底盘 pose 重合，非零外参直接拒绝，不猜测标定。
3. interface 将数值解码为带采集时间的类型。零点仅做共同平移，时间差保持原样。
4. integration 接收 shared_ptr<const Scan>；待匹配队列最多 64 帧，满时丢最旧帧并计数。
5. OdomBuffer 最多 512 个样本。At(t) 只返回同时间样本或合格括号插值，yaw 走最短圆弧。
6. 等不到右侧 odom 就等待；超出历史、括号过宽、Finish 时仍缺右侧样本则拒绝。
7. OccupancyGrid 再核对 scan/pose 时间完全一致，更新固定栅格；快照仅在检查点和结束时复制。
8. 保存和其他读者共用冻结快照。无“最新 pose”全局变量、无 Any、无运行时 topic 字符串分发。

输入从 IPC 到 C++ 必须解码，不能声称端到端零拷贝。
只读 shared_ptr 是所有权契约，不会自动阻止生产者通过另一个可变别名修改数据；
本接口会移交唯一的构造引用，其他接入方也必须遵守不可变约定。
本实现串行执行，没有解决多线程安全、异步取消、跨时钟域转换或在线重排。

地图 bounds 由离线输入数据预先计算并作为产品配置传入。没有隐式动态扩图。
它在 odom 坐标系中，没有凭空建立 map→odom 的全局定位关系。

## IPC v1

每行一条请求、一个回执；行长 <=65536，scan <=2048 个点；非法包完整解析前不交给业务。
所有时间为非负 int64 微秒，坐标为有限浮点数。

~~~text
CONFIG width height resolution origin_x origin_y max_odom_gap_us
BEGIN session_id
ODOM stamp_us x y yaw
SCAN stamp_us count x1 y1 x2 y2 ...
SNAPSHOT basename
FINISH
SAVE basename
LOAD basename
CANCEL
STATUS
QUIT
~~~

STATUS 顺序：
phase session received integrated ignored dropped pending used_rays outside_rays map_revision
drop_sync drop_queue drop_bounds drop_cancel。

phase 是 idle/mapping/ready；计数属于最近一次建图会话，map_revision 属于当前冻结/加载地图。
运行中 map_revision 为 0，用 integrated 表示更新进度。每次 Begin 重置会话统计，
扫描时间单调性与 odom 历史保持在同一进程时钟上。重播一个从 0 开始的数据集应启动新进程。
没有回调把输出错误穿透到组件中；调用返回后由 interface 写回执。
输出通道断开会失败退出，不能假定外部收到终态。

## 多组件构建

全源码只需根目录配置；安装包/混合模式仍通过稳定的命名 target 连接：

~~~sh
# 导出当前 SDK 和组件包（已有源码构建之后）。
cmake --install build-mapping --prefix "$PWD/build-mapping/packages"

# 只有 occupancy-grid 源码参与算法开发；其余组件和 SDK 用包。
cmake -S . -B build-mixed \
  -DROBOFRAME_BUILD_MAPPING=ON \
  -DROBOFRAME_USE_INSTALLED_COMPONENTS=ON \
  -DROBOFRAME_OCCUPANCY_GRID_SOURCE=SOURCE \
  -DCMAKE_PREFIX_PATH="$PWD/build-mapping/packages"
cmake --build build-mixed -j4
ctest --test-dir build-mixed --output-on-failure
~~~

组件来源选项为 ROBOFRAME_SAMPLE_ACCUMULATOR_SOURCE、ROBOFRAME_ODOM_BUFFER_SOURCE、
ROBOFRAME_OCCUPANCY_GRID_SOURCE、ROBOFRAME_MAP_STORE_SOURCE；SDK 为 ROBOFRAME_SDK_SOURCE。
值仅为 SOURCE/PACKAGE，不自动回退。SDK 与所有组件仍共用一套公开 SDK target。

旧整体开关只提供初次配置的默认值；组件来源是 CMake 缓存项。
切换已有构建目录时显式修改对应 *_SOURCE，或像示例一样使用新的构建目录，
不能假定改整体默认值会覆盖之前的逐组件选择。

## 验证与复查结论

- Release / AddressSanitizer + UndefinedBehaviorSanitizer：10 组测试通过。
- Sanitizer 下 600 帧真实数据回放、保存、重载通过。
- Release 全量 13,631 帧回放及提前结束通过。
- 4 个组件（含原累加器）仅使用独立 SDK 安装包，在隔离目录构建/测试/安装通过。
- 不含 SDK/组件源码的宿主仅用搬迁后的包构建并运行两个 IPC 产品通过。
- occupancy-grid 源码 + 其余组件包、SDK 包通过；替换为 SDK 源码也通过。
- 仅修改 VERSION 自动更新生成头并重新编译；13 个已安装公开头可单独按 C++14 编译。
- 缺少显式指定源码时直接报错；没有悄悄改用包。

隔离检查：python3 tests/mapping_build_check.py。
原基础框架安装包检查：python3 tests/isolation_check.py。

| 识别的问题 | 本次处理 | 主线建议 |
|---|---|---|
| 全局源码/包开关不能支持多人只改一个算法 | 增加小型逐组件 CMake 选择函数；组件 target 不变 | 可优先合入基础框架 |
| VERSION 文件变化不触发重新配置 | CMAKE_CONFIGURE_DEPENDS 登记真实依赖 | 可直接合入 |
| 包配置会重复要求安装 SDK，妨碍源码 SDK + 组件包 | 已有 Roboframe::sdk 时复用；运行时仍校验实际编译元数据 | 随构建改进合入 |
| 原骨架只有单数据流，没有时间与坐标约定 | 新增最少共用值类型、显式同时间匹配与单位 | 需要多传感器时合入，勿顺手引入整个地图产品 |
| “数据送进去”没有等待、过期、背压策略 | integration 有界队列；分类丢弃统计；拒绝最新值替代 | 保留原则，具体策略由产品选 |
| component 状态和产品业务状态容易混淆 | 组件仅维护生命周期；产品才有 mapping/ready；不强制 Action | 合入设计约定即可 |
| 地图会被后续更新或新会话无意修改 | 明确生成只读快照；仅显式更新可变工作图 | 有此类共享数据时采用，不抽出万能仓库 |
| 内存 Finish 和文件 Save 容易被混作一次成功 | 分离回执；不覆盖旧文件；失败可重试；校验重载 | 存储组件留功能分支，原则可写回 SDK 指南 |
| 验收只有打印难以判断实际效果 | 真实地图检查点、轨迹、完整请求与回执、冻结/重载证明 | 回放工具可独立保留，不进入 SDK |
| 组件目录完整但独立性缺少多模块证明 | 独立 SDK、无源码宿主、混合构建真实矩阵测试 | 主线保留精简后的构建回归 |

提交分层：
- 38b49d7：逐组件来源选择、VERSION 自动重新配置、最初开发计划。
- 36c8287：SDK 1.1 的二维值类型与三个独立组件。
- f565b3b：可选建图产品、IPC、真实回放及构建/行为验证。

建议最小合入范围：基础 CMake 改进与 VERSION 修复（首个功能提交 38b49d7）。
geometry2d/grid、三个组件、建图 integration、IPC 和回放属于本次功能验证，
暂不要求全部进入基础主线。未自动合并任何提交。

仍未解决：
- 没有 VIO/LIO、扫描匹配、回环、重定位；导航与地图 frame 对齐需后续明确。
- 线程、在线乱序、跨设备时钟同步不在当前串行骨架能力内。
- 原累加器的同步回调模型未改为通用执行模型，后续应独立明确回调异常/重入规则。
- 地图文件依赖 Linux/POSIX 与硬链接；未验证 ARM 或掉电一致性、磁盘满等故障注入。
- 固定范围、20m 激光约定和证据更新参数属于当前算法，不应上升为所有组件的框架规则。
