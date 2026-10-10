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
sdk/                                   公共数据、生命周期、可选任务、日志、版本
tools/mapping/                          外部回放和地图查看
tools/build-studio/                     源码/安装包/混合构建命令预览
tests/                                 产品流程、IPC、独立交付验证
~~~

每个 component 只依赖 SDK，不包含 integration 或其他组件头文件。
跨 interface 边界的自定义数据统一定义在 SDK；组件专有配置/局部结果与声明集中在具名组件头文件，
当前无需额外 contract.hpp。SDK 的准入规则与未来产品分区见 [SDK 说明](sdk/README.md)。
未来 DRComponent 可放在 odom/dr_component.hpp 和 src/dr_component.cpp。
多个实现真正共享协议时，再提取 odom 自己的契约。
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
map-updater、map-server 同样构建安装。仅使用安装包：
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
精确版本与组件清单。当前使用 SDK 2.0，旧组件包须重建。
component 不再有手写 VERSION 或 contract_version，也不自行填写 SDK 要求。
源码标识和实际编译 SDK 自动生成，安装包携带原始记录；启动时只检查 SDK 兼容性。
源码变化自动更新指纹，SDK 与产物记录不同则按兼容规则拒绝，不篡改旧包记录。
源码指纹用于追溯，不是兼容性保证。[SDK 规则](sdk/README.md)。

当前串行调用、静态链接；没有增加 BT、线程池、动态插件或 ARM 工具链。

多产品布局、BT、异步数据、ROS/DDS、ARM 和工具演进统一记录在 [TODO.md](TODO.md)。
