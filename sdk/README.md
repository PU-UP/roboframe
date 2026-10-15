# Roboframe SDK 10.0

SDK 提供机制与共享数据，不包含产品协议、具体树、算法或产品任务策略。
Component 的公共基类只规定 Activate、Deactivate 和 lifecycle；
Server、ExecutionChannel、Integration、BT 都是按需选用。

## Component 开发 skill

[roboframe-component](skills/roboframe-component/SKILL.md) 引用现有公开接口和执行模板，
指导组件实现、直接端口测试、独立构建与安装交付，不另建组件框架。
可在新任务中明确要求“读取并使用 sdk/skills/roboframe-component/SKILL.md，
按我的组件需求开发”；无需依赖这段对话或历史 demo。

SDK 安装时同时放入 share/roboframe/sdk/skills/roboframe-component；
相邻 templates 也随 SDK 安装。接口从实际选中的 SDK 头文件读取。
这里提供技能文件，不修改用户的全局技能配置；可直接按路径要求助手读取。
所有技能共用 [反馈规范](skills/feedback-format.md)，每次使用记录背景、验证及遇到的问题；
实例放仓库 .roboframe/feedback/（默认不提交），状态与评价由 SDK 维护者处理。

组件可由普通 main 直接使用和验证，输入可来自 interface、ROS、文件或测试数据，无需 Integration。
宿主负责接入和格式转换，组件公开输入输出、线程与生命周期契约；README 给出最小调用示例。
异步线程由组件自身管理；测试等待其结果，不靠产品 Poll 驱动算法。
直接示例见 [component_check.cpp](templates/integration/component_check.cpp)。
跨组件业务关系和产品任务编排另做集成验证。
源码仓库可从根目录用 -DROBOFRAME_COMPONENT=<name> 选择单个源码组件，
与 -DROBOFRAME_PRODUCT 互斥；日常开发使用固定 build-dev，具体命令见仓库 README。
安装包用户继续使用组件自身的独立 CMake 入口。

## 选择执行方式

| 工作 | 选择 | 所有者 |
|---|---|---|
| 快速同步处理 | 普通组件方法 | 组件 |
| 连续输入、串行异步计算 | ExecutionChannel<Input,Output> | 组件 |
| 单次目标或周期执行，需状态/取消/结果 | Server<Goal,Feedback,Result> | 组件 |
| 算法自带线程 | 直接封装其生命周期 | 组件 |

不要求每个组件一条线程，也不要在自带异步算法外再无条件套一层线程。
两种 SDK 执行器职责不同，均不持有算法 Component。
[两个可运行模板](templates/integration/README.md)统一示范组件管理执行，
integration 只接线、协调生命周期和产品任务。它们不是必须继承的新基类。

## 目录与依赖

- core/：Component、Integration、InputGate、Server、ExecutionChannel、任务/运行状态。
- datatype/：位姿、TF、scan、submap、grid、路径与速度等共享语义。
- orchestration/：可选记忆式 Sequence 和 ServerAction，只依赖 core/自身。
- templates/integration/：数据流组件与 Server/BT 的可编译装配示例。

core 只依赖 core，datatype 只依赖 datatype；自动测试约束层间依赖。
没有总头、万能消息总线或必须使用的导航模块。

## 接口与生命周期

Server 接纳一个目标并保留一个未领取结果；inactive、忙碌或结果未取走时拒绝新目标。
Submit 返回 epoch/id；GetStatus 查询最新反馈；TakeResult 一次领取终态及 exception_ptr；
RequestCancel 非阻塞请求取消，finish 完成后才报告终态。
Activate 提升 epoch；RequestDeactivate 关闭接纳并取消；Deactivate 等待完成；
Shutdown 最终停止并 join。生命周期调用只有一个 owner。

start/step/finish 在同一 worker 锁外串行运行，每个已接纳目标都执行 finish，
包括 start 前取消或 start/step 异常；finish 必须容忍部分初始化。
零周期只调用一次 step；正周期用 steady_clock，错过跳过、无补发、不重叠。
反馈默认值表示无有效输出；停止时零速度等算法输出由组件 finish 负责。
钩子不得等待/关闭自身执行器；持有者须在销毁钩子引用的状态前关闭执行器。
取消无法强杀第三方阻塞调用，也没有硬实时承诺。

ExecutionChannel 有界容量包括排队、运行和完成未领取结果。
reject_newest/latest_pending 由组件按输入语义选择；IMU 积分等不能任意丢旧帧。
Reset(0) 请求取消并清待处理输入，仍须 WaitIdle 并 Take 释放已完成结果；
Stop 最终 join。组件自管 epoch 与旧结果，产品 generation 不必传入内部队列。
取消、异常和无消费者时的行为必须由组件测试说明。

InputGate 仅保护短期接纳，锁内不做算法、等待或可重入回调。
Integration 的 Start/Stop/Reset/Poll 由一个控制 owner 驱动；
多组件先全部请求取消再等待，以免逐个阻塞时其他组件继续产生输出。
已有 UseServer/UseExecution/Dispatch 是兼容的便利方法，不是必选骨架；
不要将组件持有的执行器再交给 integration 管理。
OnStop/CloseInputs 和析构必须不抛异常，输出回调遵守各端口的线程与寿命契约。

## BT 与业务边界

Sequence 只有记忆式顺序执行、失败短路和 Halt；没有 XML、反应式树或自动恢复。
ServerAction 适配 Submit/GetStatus/TakeResult/RequestCancel；
Halt 只请求取消，Drained 确认退出，Reset 不能遗弃在途工作。
具体树、task/session、路径上下文、超时/重试/失败策略留产品。
组件不包含 BT；interface 负责 IPC，宿主的适配代码将通信数据转换为组件或产品公开契约。

## 类型与交付

公共类型不必全部进入 SDK。组件参数留其公开头；产品配置/请求/诊断留产品 contract。
只有有真实共同生产者和消费者、单位/时间/坐标/所有权一致的语义才加入 datatype。
大载荷使用不可变共享引用；禁止发布后通过可写别名继续修改。
队列条目、单帧大小、历史和外部快照须分别设预算；进程内共享不等于 IPC 零拷贝。

GridSpec 仍含原 demo 的资源限制，详见 [审查](../docs/sdk-review.md)。
不用该类型的产品不受它约束；需要不同网格范围时需拆分校验并验证消费者。

sdk/VERSION 由维护者发布。产品选择精确 SDK，组件构建自动记录源码 SHA256、
编译 SDK、工具链和架构；组件不手工维护版本号。
roboframe_component_metadata 生成并安装 <namespace>/version.hpp，
通过 target 自动传递生成目录。指纹不等于全部依赖/ABI 的兼容证明。
本次机制仍属未正式发布的 10.0 源码，旧组件包应重建。

## 独立编译

~~~sh
cmake -S sdk -B build-sdk -DCMAKE_BUILD_TYPE=Debug
cmake --build build-sdk -j4
ctest --test-dir build-sdk --output-on-failure
cmake --install build-sdk --prefix "$PWD/build-packages"
~~~

安装模板独立编译方法见 [模板说明](templates/integration/README.md)。
根目录 tests/build_isolation_test.py 验证安装、公开头、模板和 SOURCE/PACKAGE 选择。
