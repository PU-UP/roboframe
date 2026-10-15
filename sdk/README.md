# Roboframe SDK 10.0

SDK 只维护跨产品的运行机制与数据语义，不收集每个产品的协议、配置和诊断。
interface、integration、component 按需引用同一个 SDK；各产品可以固定不同发布版本。

## 目录与依赖

~~~text
sdk/
├─ include/roboframe/
│  ├─ core/
│  │  ├─ integration.hpp     Start/Stop/Reset、代次与完成结果交接
│  │  ├─ input_gate.hpp      输入开关、短临界区及接纳时的运行代次
│  │  ├─ execution.hpp       有界队列、过载、协作取消
│  │  └─ component/lifecycle/task/log/runtime/version.hpp
│  └─ datatype/
│     ├─ geometry2d.hpp / transforms2d.hpp
│     ├─ scan2d.hpp / submap2d.hpp
│     └─ grid.hpp / grid_cells.hpp
└─ templates/integration/
   ├─ product_integration.hpp   可编译参考模板，不是新基类
   ├─ check.cpp
   └─ CMakeLists.txt
~~~

core 只依赖 core，datatype 只依赖 datatype；可选 orchestration 只依赖 core/自身。各层不包含产品、组件或传输实现。
sdk/tests/header_layers_test.py 自动检查，安装头文件也独立编译。
生成的 sdk_version.hpp 只是版本常量。没有总头文件，不创建通用消息总线。

## 公开契约放在哪里

| 内容 | 归属 |
|---|---|
| 跨产品的位姿、图像、地图等数据与必要能力语义 | SDK datatype，按实际需求添加 |
| 某组件的专有参数、输入/结果 | 组件具名公开头；不是私有实现 |
| 产品请求、业务配置、统计汇总 | 产品的公开 contract 目录 |
| 算法状态与内部融合 | 组件内部 |
| 组件选择、连接、跨组件对时、任务编排 | 产品 integration |

**interface 可见的自定义类型必须来自公开契约，但不必全部来自 SDK。**
数据语义相同，interface、integration、component 可以直接使用同一类型，不按层强制转换。
算法组件可以使用公共能力请求/结果；产品的“开始任务”与算法的“处理数据”语义不同时，
integration 仍需要执行编排，不能为了省转换把整个产品流程交给组件。

demo_robot 的公开声明位于 products/demo_robot/contract/include/demo_robot，使用 demo_robot 命名空间，
通过 DemoRobot::contract 显式提供。通用 component 默认只链接 SDK，无法自动获得产品头路径。
产品定制 component 可以显式依赖该契约，但会成为产品相关组件；多产品共享的能力契约应独立提炼。
产品契约不包含 integration 类或具体组件，协议适配仅依赖 SDK + 契约。

## SDK 准入规则

1. 有真实生产者/消费者；不根据未来模块清单预建类型或空接口。
2. core 不包含算法/产品概念。datatype 说明单位、时钟、坐标、有效性与所有权，不携带产品开关。
3. 私有类型先留组件/产品；需要公开不等于必须进入 SDK。不得追加 products/<name> 协议树到 SDK。
4. 共享语义一致才复用类型，不把不同任务/算法状态塞进一个万能结构。
5. 大数据用不可变共享引用；写入建立自己的工作区或采用已有写时复制。不得维护第二套产品专属 worker。
6. 公开头独立编译；变更同步核对调用方、配置、安装导出和兼容性。
7. 每个新类型说明实际消费者与维护归属，单个产品的字段不自动升级成全框架标准。

GridSpec 的现有结构校验仍混有资源上限，已记录 TODO；datatype 名称不代表所有数值策略已通用。
图像、IMU、3D 位姿等待真实接入需求，不在此创建占位。

`navigation2d.hpp` 新增 Path2D（Planner 产出、Controller 消费）与 Twist2D（Controller 产出、产品输出）。
路径记录坐标系、规划地图版本、起点采集时间和不可变位姿数组；任务身份仍归产品 contract。
这些头尚未发布新 SDK 版本，接入本次源码的产品需重建 SDK/组件包；版本由维护者发布。

## Server 与可选 BT

core/server.hpp 的 Server<Goal, Feedback, Value> 是组件可选持有的执行器，
不继承 Component、不持有算法组件，也不含产品业务。组件管理自身生命周期，通过 server() 暴露执行接口。
Integration::UseServer 注册组件；BT ServerAction 绑定其 server()，普通组件不必提供此能力。

| 接口 | 语义 |
|---|---|
| Activate / active | 开放请求并提升 epoch；不表示算法数据已就绪 |
| Submit(goal) | 返回 ServerHandle(epoch, id)；inactive、忙碌或结果未领取时拒绝 |
| RequestCancel(handle) | 非阻塞，撤销状态反馈并唤醒等待；旧 handle 不影响新目标 |
| GetStatus(handle) | 最新 state / feedback / sequence / steady_clock 更新时间 |
| TakeResult(handle, result) | 终态仅领取一次，包含 value 和 exception_ptr |
| RequestDeactivate / WaitIdle | 关闭接纳并取消 / 等待清理；Deactivate 组合两者 |
| Shutdown | 最终停止并 join；持有者须在销毁被钩子引用的成员前调用 |

容量为一个目标加一个未领取结果，无隐式队列、替换或抢占。
start / step / finish 在同一工作线程锁外串行执行；每个已接纳目标都执行 finish，
包括 start 前取消、start/step 抛异常，因此 finish 必须允许尚未初始化的状态。
钩子不可调用同一执行器的等待/关闭接口；异常作为失败结果返回，升级策略归产品。

零周期调用一次 step，必须返回终态；正周期仅 RUNNING 时继续，以 steady_clock 调度。
周期不重叠、错过跳过、不补发；Context 提供取消检查和可唤醒等待。
取消立即撤销状态反馈，迟到 step 不能重新提交；finish 完成后才报告终态。
Feedback 默认值须表示无有效输出；组件输出回调的停止语义由组件实现，例如 Controller 在 finish 输出零速度。
取消不能强杀阻塞算法，也没有退出硬时限保证。

Roboframe::orchestration 是可选导出目标，提供记忆式 Sequence 和 ServerAction。
ServerAction::Halt 请求取消，Drained 确认退出；Reset 不允许遗弃在途目标。
具体树、session、路径上下文和失败策略留产品，组件不包含 BT 头。
完整图与开发分工见 [架构](../docs/architecture.md)，可运行用法见 [模板](templates/integration/server_product.hpp)。

## 输入、计算与结果

InputGate 的 Lock 返回短期 Access；锁内可检查 open、取得 generation、提交有界工作。
Close 等待当前接纳操作退出，阻止后续接纳；随后 Integration 才失效执行组并等待协作退出。
Lock 也可用于读取同一入口的轻量状态。禁止锁内等待计算、回调重入 gate 或执行重算法。
Open/Close 由控制 owner 管理，generation 来自 Integration；跨产品不能复制一份自写开关。
它不是队列、时间匹配器或算法锁，不能自动管理组件自己的后台线程。

ExecutionChannel 管排队/计算/完成的总容量；Integration::Dispatch 在控制 owner 收取并过滤旧代次结果。
算法自己的线程/回调需要其自身的关闭与结果代次约定。运行命令、Poll 和常规输出回调在控制 owner，控制速度回调直接从 Controller 工作线程进入 interface 的线程安全队列；
传感器生产者通过并发入口投递。新组件并不必然新建线程。

## 如何使用 integration 模板

templates/integration/product_integration.hpp 展示：
输入校验 → InputGate → ExecutionChannel → Dispatch → 下游/输出回调。
构造函数是 integration 的装配位置；真实产品在那里创建组件和连接具名端口。
组件只知道自身的输入输出，不知道谁装配它，也不需要调用其他组件。

模板注入一个计算函数，以便不用伪造组件也能运行验证。
新产品参考它填写自己的真实端口、组件、预算和重置策略；只复制产品代码，公共机制直接链接 SDK。
模板不是要求继承的中间层，也不引入 WireComponents 基类。

~~~sh
cmake -S sdk -B build-sdk
cmake --build build-sdk
ctest --test-dir build-sdk --output-on-failure
cmake --install build-sdk --prefix /tmp/roboframe-sdk
cmake -S /tmp/roboframe-sdk/share/roboframe/sdk/templates/integration \
  -B build-template -DCMAKE_PREFIX_PATH=/tmp/roboframe-sdk
cmake --build build-template
ctest --test-dir build-template --output-on-failure
~~~

模板验证并发接纳期间关闭、协作取消、重新启动后的迟到结果和回调线程。
SDK 原有双执行组测试继续验证慢组不阻塞快组，地图快照测试验证写时复制。

## 内存与交付边界

shared_ptr<const T> 发布后生产者也不能通过可写别名修改数据。不同引用可以并发读取，
同一个 shared_ptr 变量的并发修改仍需同步。容量按条目计，单条大小、组件历史和外部快照必须另设预算。
进程内共享不是 IPC 零拷贝，详见 [共享数据](../docs/architecture.md)。

SDK 维护者发布时修改 sdk/VERSION，产品在自己的 Dependencies.cmake 选择精确 SDK 和组件清单。
组件不手写 SDK/契约版本：构建自动记录名称、源码 SHA256、实际编译 SDK；安装清单记录工具链/架构。
指纹覆盖 src/include/cmake/CMakeLists，不涵盖外部依赖和所有编译选项，不是 ABI 保证。
源码/安装包均检查 SDK 同 major 且宿主不低于组件所用版本。头文件、库、导出和清单必须配套。

10.0 将 data 改为 datatype，并移出产品协议；旧源码 include 和旧组件包必须迁移重建。
stdio v4 线协议及 robot_runtime 文件名保持不变，产品选择名改为 demo_robot。
[架构与产品指南](../docs/architecture.md)
