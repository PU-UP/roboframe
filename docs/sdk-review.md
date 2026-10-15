# SDK 与 demo 审查（2026-10-15）

审查对象：当前未提交的导航、Server、BT、异步定位及独立发送改动；
沿 SDK → 模板 → demo 接线 → Controller → stdio 检查生命周期、取消、所有权和安装边界。
负载假设为单个 demo 进程、有界队列、无真实设备执行速度。不是实机实时性认证。

## 结论

核心能力可以保留，不应再造统一调度器。Server 是单目标/周期执行；
ExecutionChannel 是有界数据流，两者用途不同，不能仅因都创建线程就合并。
Component 只有生命周期约束，不强迫使用二者；算法已有线程时也可自行封装。
没有发现本次审查范围内需要阻止 demo 归档的确定性运行错误；
以下设计限制不能当作已经解决。

## 需要明确的设计问题

1. **两套模板的执行所有权不一致。**
   Server 模板由组件持有执行器，数据流模板由 integration 持有执行器。
   若照抄后者，新产品仍要设计线程、取消和队列收尾。
   最小处理：新主分支的数据流模板改为组件持有 ExecutionChannel；
   integration 只调用组件端口与生命周期。demo 归档保留已验证的产品执行组。

2. **UseServer 会穿透组件边界。**
   Integration::UseServer 直接调用 component.server().RequestDeactivate/Shutdown；
   自带第三方线程的组件无法直接使用这个便利方法。
   它不是 Component 的必要接口；新模板直接在生命周期钩子调用组件，
   不再登记组件内部执行器。保留辅助方法供既有 demo 使用，不新增通用注册框架。
   UseExecution 同理只适用于明确由 integration 所有的执行组。

3. **部分共享数据校验含 demo 资源策略。**
   ValidateGridSpec 拒绝小于 0.02m 的分辨率、超过 4096 的边长及八百万格。
   因此需要 0.01m 网格的产品即使结构合法也会被 SDK 拒绝。
   暂不贸然放宽：既有算法的索引和资源预算依赖这些限制；
   下一真实消费者出现时拆分结构校验与消费者资源限制，并一起验证。

4. **BT 单元测试错误地依赖产品构建。已修正。**
   原 tests/behavior_tree_test.cpp 只引用 SDK Sequence，却链接 demo integration。
   已移入 sdk/tests 并直接链接 Roboframe::orchestration，
   以后独立构建/安装 SDK 也能验证这一能力。

## 使用边界

- Component、Integration、Server 和 ExecutionChannel 都不含产品 task/session 策略。
  Integration 的基类提供可选钩子；不要求为每项能力创建新继承层。
- BT 仅为记忆式 Sequence + ServerAction，失败恢复/反应式树并未实现。
- 控制计算和 stdio 发送独立于主线程；地图结果转交仍依赖 Poll。
- 停止是协作式；算法不响应取消时无法保证退出时限。
- 零速度的生成和进入发送队列不等于实机已收到或停止；慢读仍会延迟共享 IPC。
- SDK 头文件发布了机制，不意味着已经冻结第三方算法、ABI 或全部资源策略。
- 已有回调契约要求不抛异常、不阻塞、不重入同一生命周期；
  模板销毁前必须停止后台执行，不能仅依靠成员析构顺序。

## 验证

本次新建 Debug 构建并运行完整 demo 测试：36/36 通过。
包括控制输出在主线程停顿时持续、TF 超时/取消/停止归零、Controller 异常收尾。
BT 测试归属调整后完整测试仍为 36/36；独立交付验证全部通过：
八组件独立构建/测试/安装、安装模板、纯包与混合构建、第二产品隔离、
元数据/版本拒绝规则、公开契约隔离和 36 个安装公开头独立编译。
Build Studio 命令生成自检通过。
未验证真实 SLAM、实机闭环、ARM、Windows 原生构建或硬实时期限。
