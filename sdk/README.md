# Roboframe SDK 1.1

SDK 是 integration 与 component 共用的最小基础约定，也可以被 interface 使用。
SDK 不包含具体组件、产品 integration、ROS/DDS、图像处理或 BT；依赖方向只有上层 → SDK。
当前它在 roboframe/sdk 下，是可单独配置、测试、安装的 CMake 工程。未来迁到独立仓库
不会要求组件修改 include 或链接目标。目前随 roboframe 纳入本地 Git，未拆分独立仓库或子模块。

## 运行时接口与开发模板分开

| 文件 | 责任 | 不负责 |
|---|---|---|
| component.hpp | 组件生命周期接口：Activate / Deactivate / lifecycle | 数据输入、任务目标、配置格式 |
| lifecycle.hpp | inactive / active | 产品业务状态、算法质量、故障恢复 |
| task.hpp | 可选任务状态、ID、合法转换 | 排队、线程、Action 传输、BT |
| log.hpp | 日志级别、统一输出、可替换 sink | 异步日志、文件轮转 |
| version.hpp | 版本比较、已编译组件信息、兼容性检查 | C++ 二进制 ABI 保证 |
| templates/ | 可复制的组件与 integration 项目 | 强制继承的产品逻辑 |

Component 基类只统一可以共同维护的生命周期。Task 是可选成员，持续输出里程计、
感知等组件无需为了“统一”伪造任务。SDK 没有通用 AddData，也没有强制 Integration
基类；目前 integration 的端口与流程随产品变化，复用模板和组件组合即可。

组件自己的 Config、Input、Output、Goal、Result 属于组件契约，放在组件目录。
SDK 只是公共契约，不等于每个组件的完整 contract。小组件可以把几个类型放在
component.hpp；类型增长后再拆 contract.hpp，不建立中央“所有组件数据字典”。

## 生命周期、配置与调用约定

组件初始 inactive。持有者先接线再 Activate；停用前结束活动任务。
Activate/Deactivate 在本例可重复调用，但任务进行时 Deactivate 会明确拒绝。
错误输入在修改内部状态前拒绝；算法任务失败不自动变成组件失效。

配置由 integration 选择，组件自己定义并校验；模板示范构造时传入不可变 Config。
没有真实配置项的累加器不设空 Config。“本次累计几个样本”属于 Goal。
动态改参、线程、异步取消需要具体需求之后另行定义，不能用同一个 Config 偷偷兼任。

组件调用和回调由单线程串行执行。回调不抛异常、不重入、不销毁组件。
持有者负责停止任务、停止数据输入、再停用/销毁；析构不发送业务回调。
这些执行约定是契约的一部分，并非继承基类后自动获得的线程安全。

## 日志

~~~cpp
#include <roboframe/log.hpp>
ROBO_LOG_INFO("vio") << "frame=" << frame_id;
ROBO_LOG_WARN("planner") << "no path";
~~~

默认 info 级别，格式为 [info] [模块] 消息，输出 stderr，避免污染 IPC stdout。
宿主可调用 SetLogLevel；组件不擅自修改全局设置。
SetLogSink 接收一个回调，可接产品已有日志库；空回调表示关闭输出。
sink 调用串行化，不能重入日志或日志配置 API。sink 抛出的异常被吞掉，
但消息构造仍可能分配内存。这是同步日志，不承诺实时线程无阻塞，
被过滤的日志仍会计算流表达式，不应在日志表达式中执行有副作用的业务操作。
线程安全只覆盖日志，不改变组件自己的单线程约定。

当前 SDK 是 header-only，正式组件使用静态库，宿主内共用同一份日志设置。
未来采用动态插件时，需要重新设计跨动态库日志实例与 ABI；本轮没有伪装成已支持插件。

## 三种版本分别维护

| 版本 | 位置 | 含义 |
|---|---|---|
| SDK 版本 | sdk/VERSION | 公共接口、生命周期及 task 等约定 |
| 组件实现版本 | components/sample-accumulator/VERSION | 具体算法/实现发布版本 |
| 组件契约版本 | contract.hpp 中 contract_version | 输入输出、单位、坐标系、时序、取消语义等 |
| 主框架版本 | 根目录 VERSION | 产品装配发布标记，不参与替代上述兼容性检查 |

破坏公共接口/语义升级 major；向后兼容扩展升级 minor；兼容修复升级 patch。
SDK 与组件的版本号无需相等。实现改变而接口语义不变，可以只升级实现版本。
模板用自己 CMake project(VERSION) 标记实现版本，正式组件用 VERSION 文件集中管理。

1. 配置期：find_package(RoboframeSDK 1.0 CONFIG REQUIRED)，版本文件采用 SameMajorVersion。
2. 编译期：组件 static_assert 最低 SDK 1.0.0 和相同 major；源码直接链接也生效。
3. 启动期：GetComponentInfo() 在组件 .cpp 实现，返回库编译时的 SDK、实现和契约版本。
   integration 在激活前核对：宿主 SDK >= 组件编译 SDK 且同 major，
   组件提供的契约 >= 宿主要求的契约且同 major。不兼容抛异常，终止启动。

这是一项保守的版本策略：即便组件没有使用新 SDK 功能，用较新 SDK 编译也要求宿主
至少同样新。版本数值不是自动兼容性证明，发布者仍需守住上述语义版本规则。
运行时检查只处理程序能够正常链接和进入执行后的语义版本错误，不能修复 ABI 错误，
也不允许 x86 库直接在 ARM 运行。使用不同编译器/标准库或目标架构时须重新编译。

sdk_version.hpp 是 CMake 从 sdk/VERSION 生成的版本头。它出现在 build/sdk/generated
及安装目录是正常构建产物；业务源码只在源目录维护，不修改 build 中的副本。

## 独立交付与模板

~~~sh
cmake -S sdk -B build-sdk
cmake --build build-sdk
ctest --test-dir build-sdk --output-on-failure
cmake --install build-sdk --prefix /tmp/roboframe-sdk
~~~

消费者只需 find_package(RoboframeSDK 1.0 CONFIG REQUIRED)，链接 Roboframe::sdk。
安装包包含公开头文件、CMake 包和 share/roboframe/templates，不包含产品实现。
开发模板说明见 [templates/README.md](templates/README.md)。

## 建图分支的最小数据类型扩展

1.1 增加 geometry2d.hpp 和 grid.hpp，定义跨 odom/grid/store 共用的位姿和只读地图。
具体 Scan 仍由组件定义；没有引入中央组件数据注册表。相关验证与适用范围见 ../docs/mapping.md。
