# 架构与新产品接入

本页统一维护架构图与开发分工。产品行为见 [建图 demo](demo_robot.md)、[导航 demo](navigation-demo.md)；
公共接口见 [SDK](../sdk/README.md)，线协议见 [IPC](ipc.md)。

## 完整架构与线程

SDK 是被各层引用的库，不是独立线程：core 提供 Integration、Server、InputGate、ExecutionChannel；
可选 orchestration 提供 Sequence、ServerAction；datatype 提供共享数据类型。

~~~mermaid
flowchart TB
    subgraph MAIN["T1 主线程：命令 / 生命周期 / Poll"]
        IPC["interface 输入适配"]
        INT["integration<br/>持有组件、任务与数据接线<br/>输入接纳 / Odom 透传"]
        BT["产品 BT：ComputePath → FollowPath<br/>复用 SDK Sequence / ServerAction"]
        MAP["基准地图<br/>FINISH 冻结 / MapServer 加载"]
        IPC --> INT
        INT --> BT
        MAP -->|"地图与目标上下文"| BT
    end
    LOC["T2 定位通道<br/>RecordedSLAM + PoseGraph"]
    BUILD["T3 建图通道<br/>LocalMapBuilder + MapUpdater"]
    subgraph PLANNER["PlannerComponent · Component"]
        PLAN["持有 SDK Server；算法在组件内<br/>T4 单次规划"]
    end
    subgraph CONTROLLER["ControllerComponent · Component"]
        CTRL["持有 SDK Server；算法在组件内<br/>T5 默认 50ms 周期控制"]
    end
    TX["T6 interface 发送线程 → IPC<br/>有界队列；速度保留最新待发值"]
    INT -->|"匹配 scan / odom / 参考位姿"| LOC
    LOC -->|"定位结果，经主线程 Poll 交接"| BUILD
    BUILD -->|"地图结果，经 Poll / FINISH"| MAP
    INT -->|"最新 odom 推导 TF，经接线"| CTRL
    LOC -->|"定位修正更新 TF，经接线"| CTRL
    MAP -->|"UpdateMap 快照"| CTRL
    BT -->|"server()：Submit 地图 / TF / 目标；Cancel"| PLAN
    PLAN -->|"TakeResult：路径存入任务上下文"| BT
    BT -->|"server()：Submit 路径；Cancel"| CTRL
    CTRL -->|"GetStatus / TakeResult"| BT
    CTRL -->|"速度回调直接入队，不经 Poll"| TX
    IPC -->|"命令响应"| TX
    INT -->|"常规事件"| TX
~~~

实线表示调用或经 integration 接线的数据流，不表示组件互相持有。
integration 持有各组件；PlannerComponent / ControllerComponent **继承 Component，直接实现算法并持有 SDK Server 执行器**。
算法状态和生命周期只有一份，不再嵌套另一个算法 Component。
SDK Server 不继承 Component、不持有组件、不包含产品业务；普通组件可以完全不用它。

Server 是按组件需求选择的能力，不是所有组件的必选基类或运行方式：

| 组件 | 当前执行方式 | 是否需要 Server |
|---|---|---|
| PlannerComponent / ControllerComponent | 单次目标 / 周期跟踪，由组件持有执行器 | 是，统一提交、取消与结果 |
| Odom | 输入入口短锁下的同步轻量调用 | 否 |
| RecordedSLAM / PoseGraph | integration 的定位 ExecutionChannel 串行执行 | 否 |
| LocalMapBuilder / MapUpdater | integration 的建图 ExecutionChannel 串行执行 | 否 |
| MapServer | 主线程同步资产操作 | 否 |

新增 Server 不改变普通 Component 接口，已有持续流组件不需要为 BT 改写；真实算法接入时再按执行语义决定。

图中共六个线程。接线回调按来源线程执行：定位修正更新 TF 在定位线程；
当前 odom 更新 TF 在输入调用线程（stdio demo 为主线程）；速度输出在控制线程。
这些交接使用短锁端口或有界队列。建图使用扫描采集时刻的 TF；
控制使用最新 odom 与有效定位修正，延迟扫描不能回退当前位姿。

## 职责边界

| 层 | 直接负责 | 留给其他层 |
|---|---|---|
| SDK core | 生命周期、执行线程、周期、取消、状态/结果、有界数据流 | 算法、产品策略、IPC |
| SDK 可选 BT | Sequence、ServerAction 的提交/查询/取消机制 | 具体树和任务语义 |
| component | 算法、具名输入输出、自身生命周期；按需持有 Server | 其他组件、BT、integration 和产品 session |
| integration | 选组件、配置、接线、任务状态、路径上下文、具体 BT 与失败策略 | 算法实现、重复实现调度器 |
| interface | 契约编解码、订阅、背压、独立发送 | 调用组件或执行产品流程 |

组件默认只依赖 SDK 和自身公开接口。interface 依赖公开产品 contract，
由 main 绑定输入/输出，不包含 integration 实现或组件头。共享语义直接复用 SDK 类型，不为分层强制转换。

## BT 与能力调用

BT 在主线程只操作组件的 server()：Submit、GetStatus、TakeResult、RequestCancel。
耗时规划和周期控制由执行器线程调用算法，不在 BT Tick 中运行。
ServerAction::Halt 请求取消，Drained 确认清理完成；Reset 不得遗弃仍执行的目标。

导航树为 ComputePath → FollowPath。Planner 的不可变 Path2DPtr 进入产品任务上下文，
再作为 Controller 目标提交；Planner 不直接调用 Controller。
建图树为 Collect → Drain → Freeze；持续传感器流不经 BT。
当前 Sequence 只有记忆式顺序执行、失败短路和 Halt，没有 XML、反应式分支或自动恢复。

## 生命周期与资源

- Integration 的 Start / Stop / Reset / Poll / 查询由同一 owner 调用；传感器入口允许并发。
  UseServer 注册组件生命周期，UseExecution 注册持续数据流通道；active 不代表定位有效或有任务。
- 停止顺序：关闭输入、请求所有 Server 取消、失效数据流执行组、等待退出、停用组件；
  Controller 清理时输出零速度，之后 interface 排空输出并关闭。接收者必须存活到组件停止。
- Server 只接纳一个目标并保留一个未领取结果；周期跳过错过的时刻，不补发。
  取消是协作式，清理完成后才报告终态，不能强杀阻塞算法。
- 数据流完成结果携带 generation，Dispatch 丢弃旧代次；产品 session 标识业务任务。
  CANCEL 不停止持续流；FINISH 固定有限接纳边界，后续输入不延长建图任务。
  算法失败不保证事务回滚，demo 数据流故障进入 ERROR，恢复时清理工作状态。
- InputGate 只保护接纳，不运行耗时算法。ExecutionChannel 容量包含排队、计算中及完成未领取结果；
  reject_newest / latest_pending 按输入语义选择，IMU 等历史输入不能任意合并。
- Scan2DPtr / Path2DPtr / GridSnapshot 为不可变共享数据；GridCells 按块写时复制。
  这不是 IPC 零拷贝；外部引用、历史数据、传输缓存仍须分别限定预算。

主线程停顿时 Controller 计算、TF 超时检查、速度入队和发送继续；
若 odom 输入也停流，则控制线程超时归零。任务状态需主线程恢复后领取结果再更新。
共享 stdout 的大消息/慢读仍可能延迟速度，传输故障不承诺送达或设备停止。

## 新产品开发者写什么

从 [Server + BT 模板](../sdk/templates/integration/server_product.hpp) 起步；
持续数据流参考 [输入模板](../sdk/templates/integration/product_integration.hpp)，不复制整个 demo_robot。

| 新产品需要填写 | 导航 demo 的具体例子 | 可直接复用 |
|---|---|---|
| 依赖、配置与组件装配 | Dependencies.cmake；创建 Planner/Controller，UseServer 注册 | SDK 构建与生命周期机制；现有组件 |
| 数据接线 | 地图/TF 放进规划目标；持续更新 Controller 输入；速度回调绑定 interface | 不可变路径/地图、执行器、有界输出队列 |
| 任务与具体树 | Navigate/Cancel、session、路径上下文、ComputePath → FollowPath、失败处理 | Sequence、ServerAction，不重写工作线程或周期循环 |
| 产品契约与入口 | main 绑定 contract 和 interface，管理回调寿命 | 同协议可复用适配器；不同协议写自己的编解码 |
| 算法（仅换能力时） | Goal/Feedback/Result、start/step/finish、具名输入口、控制失效归零 | SDK Server 的线程、调度、协作取消与结果交接 |

integration 模板是可复制的装配示例，不是新的中间基类。
初期一对 integration.hpp/.cpp 即可，sensor_ingress / wiring / tasks 只是 demo 的阅读分区。
只做导航无需复制建图任务，也无需照搬录制 scan/odom 的配对规则。
新产品仍需选择对时、过载、资源保留和故障策略。

代码阅读入口：[装配](../products/demo_robot/data_flow/wiring.cpp) →
[导航任务](../products/demo_robot/tasks/navigation.cpp) →
[Controller 组件](../components/controller/include/controller/controller_component.hpp) →
[发送线程](../interface/stdio_channel.cpp)。

## 验证入口

[control_output_test](../tests/control_output_test.cpp) 检查主线程阻塞时真实管道持续收到速度、TF 超时/取消/停止归零；
controller_execution_test 覆盖异常清理；server_test 覆盖取消、迟到反馈和周期。
[build_isolation_test.py](../tests/build_isolation_test.py) 检查 SDK、八组件、安装模板和纯包/混合构建。
运行方法见 [README](../README.md)；未完成项统一维护在 [TODO](../TODO.md)。
