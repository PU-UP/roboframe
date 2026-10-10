# roboframe
C++14 的 interface → integration → component 骨架。只有一个产品进程 robot_runtime；
建图是 RobotIntegration 中的一项流程。本地 Git，无 remote/submodule。
history/ 和 build*/ 中旧数据、产物不参与构建，也不代表当前实现。

~~~text
main.cpp                               持有 integration，向 interface 注入引用
interface/stdio_interface.*             IPC 编解码、输入校验、回执
integration/robot_integration.hpp       仅依赖 SDK 的产品公开入口
integration/robot_integration_impl.hpp  私有组件装配与状态
integration/robot_integration.cpp       入口实现、版本核对、生命周期
integration/data_flow.cpp              数据接线、同步、分发
integration/pose_buffer.*               产品内部的有界缓存与插值工具
integration/mapping_flow.cpp            建图流程、地图操作
components/odom/                        PassthroughOdomComponent
components/map-updater/                 OccupancyMapComponent
components/map-server/                  FileMapComponent
components/planner/                     ReadOnlyPlannerComponent（读取验证，不生成路径）
sdk/                                   公共数据、生命周期、可选任务、日志、版本
tools/mapping/                          外部回放和地图查看
tools/build-studio/                     源码/安装包/混合构建命令预览
tests/                                 产品流程、IPC、独立交付验证
~~~

每个 component 只依赖 SDK，不包含 integration 或其他组件头文件。
跨 interface 边界的自定义数据统一定义在 SDK；组件专有配置/局部结果与声明集中在具名组件头文件，
当前无需额外 contract.hpp。SDK 的准入规则与未来产品分区见 [SDK 说明](sdk/README.md)。
具体实现采用具名组件和文件，可以分别作为独立仓库交付；不要求所有里程计实现集中在一个 odom 仓库。
具体实现由 integration 私有实现静态装配，没有动态插件或实现注册框架。
main 创建并持有 RobotIntegration；StdioInterface 只引用公开入口。
CONFIG 调用 Configure 后才创建并激活组件；失败不留下半配置的对象，可以重试。
成功后拒绝重复配置。QUIT/EOF 关闭组件；异常退出由 integration 析构兜底。
组件依赖为 PRIVATE，不再通过 integration 向 interface 传播组件头文件。

Odom 是能力；PassthroughOdomComponent 校验并透传已有 odom，不估计或修正轨迹。
Map updater 更新占据地图，Map server 保存/读取地图；它们不决定何时开始建图。
MappingStatus 只表示建图会话，不代表组件生命周期或整个产品状态。
同一个 integration 可以多次建图、取消、加载；其他流程可以复用相同组件，
无需重启或新建 Integration 基类。

## 架构原则：能力封装与按需组合
**默认由一个 component 完整承担一项产品能力；integration 具备组合能力，但不要求算法都拆成小组件。**
组件粒度以开发、复用和维护成本为依据，不以算法内部有多少步骤为依据。

以局部里程计为例，odom 表示产品需要的能力，不等于必须有一个包住所有实现的 OdomComponent。
产品可以要求 50 Hz 的有效局部位姿，由 VIOComponent、DRComponent 或 LIWComponent 提供。
这些实现允许位于各自独立仓库，使用不同传感器、输入接口和配置。
integration 按产品选择并接线，下游消费约定的位姿输出。
替换算法可能需要调整传感器接线和配置，不承诺所有输入接口完全相同。

| 边界 | 负责内容 |
|---|---|
| SDK | 稳定的跨模块数据语义：单位、坐标系、时间、所有权及必要的有效性信息；按实际需求增加 |
| component | 算法实现及满足输出要求所需的内部传播、滤波、融合和内部同步 |
| integration | 产品装配、跨组件数据分发和时间匹配、配置、启停、业务编排及下游输出来源选择 |
| 产品要求 | 输出频率、延迟、数据新鲜度和失效行为；由组件实现，并通过产品集成验证 |

例如视觉估计只有 10 Hz，可以在 VIOComponent 内组合高频运动传播和融合，完整输出产品需要的位姿。
内部模块如何拆类或库，由算法团队决定，无需把每个内部模块都注册为框架 component。
50 Hz 是产品要求的例子，不是 SDK 对所有产品的固定限制；重复发送旧估计不等于获得新的有效估计。

当多个产品确实需要独立复用、发布或自由组合 DR、VIO 和融合时，也允许三个 component 同时运行，
由 integration 连接它们。融合估计计算仍由融合 component 承担，不搬进 integration。
跨组件对齐和转发数据与算法内部状态传播是不同职责，按具体消费者需求确定边界。

通用性首先来自清晰的数据语义和基本生命周期，不强制所有实现继承统一 odom 基类，
不预建动态插件或包住所有算法的通用包装层。普通扩展组件同样可以接入，不限于预定义能力清单。
当前 components/odom 只是透传实现的目录，不是其他里程计算法必须加入的容器；
它跟随输入输出，当前并不保证 50 Hz，也未实现 VIO、DR 或融合。
下一步先明确局部位姿输出契约，待真实需求出现再扩充实现与仓库。

## 构建与运行
~~~sh
cmake -S . -B build-product -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tools/mapping/replay.py --scans 0 --finish-after 12267 \
  --output build-product/full-result
~~~
输出目录必须不存在，防止覆盖地图。回放需要 history 中的 intel.log.gz。
查看输出 replay.html；[链路、协议与地图验证](docs/mapping.md)。
FINISH 保存基准快照，之后 scan 继续更新实时地图。SAVE 保存基准，SNAPSHOT 保存当前地图；
planner 读取共享 scan 和地图。详见 [大数据共享与验收](docs/data-sharing.md)。

## 独立开发与安装包
每个组件 tests/ 下的具名测试都是直接调用组件的 main，可独立运行和调试。
不再维护累加器、数值缩放器或另一套复制型演示工程。
~~~sh
cmake -S sdk -B build-sdk
cmake --build build-sdk
cmake --install build-sdk --prefix "$PWD/build-packages"
cmake -S components/odom -B build-odom -DCMAKE_PREFIX_PATH="$PWD/build-packages"
cmake --build build-odom
ctest --test-dir build-odom --output-on-failure
cmake --install build-odom --prefix "$PWD/build-packages"
~~~
map-updater、map-server、planner 同样构建安装。仅使用安装包：
~~~sh
cmake -S . -B build-packaged -DROBOFRAME_USE_INSTALLED_COMPONENTS=ON \
  -DCMAKE_PREFIX_PATH="$PWD/build-packages"
~~~
只开发地图更新组件：
~~~sh
cmake -S . -B build-mixed -DROBOFRAME_USE_INSTALLED_COMPONENTS=ON \
  -DROBOFRAME_MAP_UPDATER_SOURCE=SOURCE -DCMAKE_PREFIX_PATH="$PWD/build-packages"
~~~
SDK 和组件均可独立选择 SOURCE/PACKAGE；缺少所选源码或包时明确报错。
BUILD_TESTING=OFF 不要求 Python，不编译测试 main。
完整隔离验证：python3 tests/build_isolation_test.py。
升级后使用新 build 目录；旧包名、类名和 mapping_runtime 不再保留。

## 版本维护
SDK 维护者只维护 sdk/VERSION；产品维护者在 cmake/Dependencies.cmake 选择 SDK
精确版本与组件清单。当前使用 SDK 3.0，Grid 的分块存储接口有变化，旧组件包须重建。
component 不再有手写 VERSION 或 contract_version，也不自行填写 SDK 要求。
源码标识和实际编译 SDK 自动生成，安装包携带原始记录；启动时只检查 SDK 兼容性。
源码变化自动更新指纹，SDK 与产物记录不同则按兼容规则拒绝，不篡改旧包记录。
源码指纹用于追溯，不是兼容性保证。[SDK 规则](sdk/README.md)。

integration 和地图写入仍串行；planner 有一个独立只读线程。静态链接，无 BT、线程池、动态插件或 ARM 工具链。

多产品布局、BT、异步数据、ROS/DDS、ARM 和工具演进统一记录在 [TODO.md](TODO.md)。
