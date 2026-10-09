# roboframe：最小 SDK 与三层骨架

C++14。当前主线为 interface → integration → component，SDK 为这些层提供公共约定。
当前使用本地 Git 仓库，未配置远程或 submodule；components/sample-accumulator 是完整、可独立构建的组件目录。
旧实现原样保留在 history/，历史设计回顾见 [design-reset.md](docs/design-reset.md)。

## 目录与职责

~~~text
roboframe/
├── CMakeLists.txt / VERSION             主框架构建与版本
├── sdk/                                可独立构建、安装的公共 SDK
│   ├── include/roboframe/
│   │   ├── component.hpp               最小生命周期基类
│   │   ├── lifecycle.hpp / task.hpp     生命周期、可选任务状态
│   │   └── log.hpp / version.hpp        统一日志、版本核对
│   ├── cmake/                          包配置、生成版本头模板
│   ├── templates/                      可复制、可编译的开发起点
│   │   ├── component/                  持续数据型 value-scale + 自测 main
│   │   └── integration/                配置、接线与启停示例
│   └── tests/                          SDK 公共逻辑自测
├── main.cpp                            进程入口
├── interface/stdio_interface.*         IPC 编解码
├── integration/
│   ├── components.hpp                 组件所有权、版本核对与启停顺序
│   ├── data_flow.cpp                  强类型数据转发
│   └── demo_integration.*             任务调用与输出接线
├── components/sample-accumulator/       独立组件边界
│   ├── include/sample_accumulator/     自有接口与载荷契约
│   ├── src/component.cpp              实现与编译版本信息
│   ├── examples/main.cpp              脱离 integration、IPC 的直接调用
│   ├── tests/component_check.cpp      组件行为测试
│   └── CMakeLists.txt / VERSION        独立构建、安装包与实现版本
├── examples/demo.py                    外部进程，通过管道 IPC 驱动
└── tests/                              IPC、包隔离与开发模板验证
~~~

~~~mermaid
flowchart LR
    P[外部进程] -->|管道 IPC| I[interface]
    I --> N[integration]
    N --> C[component]
    N -.公共约定.-> S[SDK]
    C -.公共约定.-> S
    M[算法开发者自己的 main] --> C
~~~

Integration 分装配、数据路由、任务编排三个职责，但没有引入三套管理器。
组件状态仍由组件唯一维护；SDK 的 Component 基类只要求生命周期，
Task 保持可选。不同产品可写自己的 integration，组件和 SDK 不需要知道产品流程。

SDK 可以独立交付给算法开发者；组件不包含主框架私有头文件、不读取父目录。
主框架默认从源码连接它，也可以只消费安装包。文件夹嵌套不改变依赖边界。
公共 SDK 与组件私有契约、日志、版本规则详见 [sdk/README.md](sdk/README.md)。

## 默认构建：一条源码依赖链

~~~sh
cd /home/watermango/work/roboframe
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure

# 外部 Python → IPC → interface → integration → component
python3 examples/demo.py

# 算法开发者自己的 main → component
./build/components/sample-accumulator/accumulator_demo
~~~

默认构建 SDK、正式组件、主程序和独立 demo。修改源码后直接重新 build，
不需要先安装头文件或静态库。SDK 模板不加入主程序，只在隔离验证时单独构建运行。
CMAKE_PREFIX_PATH 在默认源码模式下不用于选择本地组件，避免误链接旧安装包。

| 选项 | 默认值 | 用途 |
|---|---|---|
| BUILD_TESTING | ON | 构建并注册测试；OFF 时不要求 Python |
| SAMPLE_ACCUMULATOR_BUILD_DEMO | ON | 构建组件的独立 main |
| ROBOFRAME_USE_INSTALLED_COMPONENTS | OFF | ON 时从安装包接入组件，不读取本地 SDK/组件源码 |

关闭测试不会关闭独立 demo；需要精简产物时分别设 OFF。
源码模式只在根 CMake 声明有哪些组件；组件 CMake 只依赖 Roboframe::sdk。
C++14、编译警告、include 和链接依赖通过各自目标声明，没有全局 include/link 搜索目录。

## 独立交付：算法作者只拿 SDK

~~~sh
cd /home/watermango/work/roboframe

# SDK 可单独构建；交付 install-sdk 给组件作者即可。
cmake -S sdk -B build-sdk
cmake --build build-sdk -j2
cmake --install build-sdk --prefix "$PWD/build/install-sdk"

# 这个组件源码目录可以复制到任何位置，父目录不参与构建。
cmake -S components/sample-accumulator -B build-component \
  -DCMAKE_PREFIX_PATH="$PWD/build/install-sdk"
cmake --build build-component -j2
./build-component/accumulator_demo
ctest --test-dir build-component --output-on-failure
cmake --install build-component --prefix "$PWD/build/install-component"

# 产品集成也可以仅消费包；与源码模式使用不同 build 目录。
cmake -S . -B build-packaged \
  -DROBOFRAME_USE_INSTALLED_COMPONENTS=ON \
  -DCMAKE_PREFIX_PATH="$PWD/build/install-sdk;$PWD/build/install-component"
cmake --build build-packaged -j2
ctest --test-dir build-packaged --output-on-failure

# 安装包中的可复制组件/integration 模板，见 sdk/templates/README.md。
python3 tests/isolation_check.py
~~~

算法作者可直接替换 examples/main.cpp，链接 SampleAccumulator::component，
给它送数据并观察回调，不需要启动主框架或模拟 IPC。
组件详细接口见 [components/sample-accumulator/README.md](components/sample-accumulator/README.md)。

build* 只放生成文件、可执行文件、库和测试输出。版本头 sdk_version.hpp 从
sdk/VERSION 生成；安装包的 include 是公开头文件的副本，都不应作为源码修改。
本轮未引入动态加载、注册机制、BT 或线程池。

## 状态谁维护

生命周期目前只有 Inactive / Active。没有实际故障恢复实现，所以不预设一个空的 Faulted。
任务失败与组件失效不同：本例累计溢出结束当前任务，组件仍可以接受下一个目标。
将来需要故障恢复时再定义相应生命周期语义。

Task 是**可选成员**，没有组件必须继承的 ActionServer 基类：
- Planner、Controller 等目标型能力可使用它。
- 连续估计组件可只使用 lifecycle.hpp，加自己的估计质量输出。
- 普通同步查询也不必变成 Action。

Task 状态为 idle（尚无目标）、running、canceling、succeeded、failed、canceled。
接受目标是 Submit 成功的响应，不再保存一段没有等待队列的 Accepted 状态。
Canceling 仍保留，避免把“请求取消”和“确实结束”混为一谈。
本例没有异步工作，所以 Cancel 中连续完成两个转换，调用者通常直接观察终态。

只有组件中的 Task 修改任务状态，integration/未来 BT 只能提交、取消、查询。
公共 Task 不管理数据、回调、线程、重试、排队、超时、资源或产品流程。
新目标与已有任务互斥；目标 ID 在组件实例存续期间严格递增。
任务终态保留到下次成功提交；没有历史结果仓库或断线续传协议。

## 数据与任务分开

~~~text
GOAL 1 2 → integration.Submit(Goal) → component.Submit(Goal)
SAMPLE 3 → integration.PushSample(Sample) → component.PushSample(Sample)
CANCEL 1 → integration.Cancel(id) → component.Cancel(id)

component Feedback / Result → integration 接线 → interface 编码 → IPC
~~~

PushSample 表示主动送入一个具名数据类型，不是通用 AddData 或回调 OnSample。
当前 Sample 只有整数值，不加无用途的 frame、timestamp、metadata。
将来图像等数据的采集时间和坐标属于那种数据的契约，不放进万能 Sample。

任务是“累计目标接受后收到的 N 个样本”，N 是 Goal 参数，不是算法配置。
新目标自动清空本任务统计；不需要额外 RESET 协议和 integration 会话状态。
组件现在无配置，保持无配置，不添加空 Config 类。

## IPC 示例

宿主启动后已激活组件，输出 READY。stdout 是协议，stderr 是分层日志。
demo 将它们合并，便于阅读。每行最多 128 字节，必须以换行结束。

~~~text
GOAL 1 2
  ACCEPTED 1
SAMPLE 3
  FEEDBACK 1 1 3
SAMPLE 5
  FEEDBACK 1 2 8
  RESULT 1 succeeded 2 8 none
STATUS
  STATUS active 1 succeeded
GOAL 2 3
  ACCEPTED 2
SAMPLE -2
  FEEDBACK 2 1 -2
CANCEL 2
  RESULT 2 canceled 1 -2 none
~~~

FEEDBACK：goal_id / 已累计数量 / 总和。
RESULT：goal_id / 终态 / 已累计数量 / 总和 / 错误码（none 或 overflow）。
CANCEL 在本例直接返回终态结果，不另发一条重复的成功回执。
STATUS：生命周期 / 最后接受的 goal_id / 当前或最后任务状态。
ACTIVATE、DEACTIVATE 返回 OK；有活动任务时 DEACTIVATE 被拒绝。
无效请求输出 ERROR，不改变已有任务。失败目标输出 RESULT failed，不伪装成输入拒绝。
一次 SAMPLE 可以同时产生反馈和终态结果，不再沿用“一请求只有一回执”的旧协议。
旧 START / STOP / RESET 已移除，避免与任务取消、组件启停混淆。

QUIT/EOF 经 integration 取消活动任务，再停用组件；QUIT 最后输出 BYE。
无换行的残缺请求不执行。若输出通道已断，不能承诺客户端收到了终态。
这是同步单客户端演示；取消完成不是实机停止确认，连续传感器也不受此示例协议约束。

## 契约刻意保持小

公共部分定义生命周期、可选任务状态、日志和版本约定；不收集各组件的业务载荷。
组件部分只定义该模块确实需要的载荷。
没有 Any/Data 大包、topic 路由、类型注册、契约继承树或序列化依赖。

执行和回调仍在一个线程串行完成；回调不得抛异常、重入或销毁组件。
公共 helper 并未解决线程安全。未来引入异步时，取消完成必须等到实际工作结束，
并用原目标 ID 丢弃过期完成通知；不能只把枚举改成 canceled。

验证产物：build/demo-output.txt、build/isolation-check.txt。
历史总结记录了各阶段方案，不代表旧协议或旧目录仍是当前实现。

## 可视化构建原型

[tools/build-studio](tools/build-studio/README.md) 提供无需前端依赖的本地网页：
当前整体源码/安装包模式可生成命令，混合来源与 ARM 为明确标记的设计预览。
工具不参与主框架构建；未来组件发现、工具链与 UI/CLI 共用入口的约定记录在其 README 中。

## 本地版本管理

当前 SDK、components、工具与文档作为同一仓库的普通目录提交，暂不拆分子仓库。
历史归档 history/、根目录 build*/ 和工具缓存保留在本机，但不纳入版本管理。
因此当前仓库不包含历史归档的备份；需要保留归档时应单独备份该目录。
