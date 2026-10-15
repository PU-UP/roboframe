# 骨架边界

~~~mermaid
flowchart LR
    IPC["interface：IPC 输入 / 独立发送线程"] --> INT["integration：主线程<br/>生命周期协调 / 接线 / 产品 BT"]
    INT --> C["Component：算法 + 自身生命周期"]
    C --> S["可选持有 Server<br/>目标 / 周期 / 取消 / 结果"]
    C --> E["可选持有 ExecutionChannel<br/>有界输入 / 异步处理 / 完成结果"]
    C --> A["或同步调用 / 算法已有线程"]
    S -->|短回调入队| IPC
~~~

SDK 是机制库，不是单独的线程。Component 不是 Server；
组件可持有其中一种执行器，也可以不用，不能嵌套另一个算法 Component 只为了调度。
组件不认识 integration、BT 或其他组件。

- SDK core：Component 生命周期、Integration 生命周期钩子、InputGate、两类执行器。
- SDK orchestration：可选 Sequence/ServerAction；不是产品任务定义或完整 BT 引擎。
- SDK datatype：共享数据语义，可按需引用；不要求新产品使用二维导航类型。
- 模板：演示组件所有权、关停顺序和 BT 适配；没有新框架基类。
- 产品：组件选择与配置、接线、公开契约、具体 BT、任务/故障策略。
- interface：通用 IPC；宿主适配组件或产品公开契约，不在传输层运行算法。

图示为产品装配；独立宿主 main 也可接入 interface、ROS 或文件，直接调用组件，无需 Integration。

## 执行与停止

线程由组件管理，SDK 执行器负责具体线程/队列机制。
生命周期在一个控制线程调用；输入方法的并发支持由组件公开说明。
integration 先关闭输入并请求所有组件取消，再等待各组件 Deactivate，
最后清理产品任务；停止发送线程前排空输出，接收者必须活得更久。

数据流模板通过 Poll 领取结果，跨组件交接仍依赖主线程。
周期输出可以经线程安全短回调直接入 interface 队列，不经过 Poll。
这两种输出契约需明确，不能只因“组件有线程”就宣称整条流水线独立运行。

取消是协作式；不响应取消的第三方算法无法强杀。
ExecutionChannel 的容量包含排队、计算、已完成未领取；外部快照和算法历史另设预算。

Integration 的 UseExecution/UseServer 是保留的兼容接口，由 sdk/tests 覆盖；
组件开发模板不调用它们，避免 integration 与组件同时管理同一个执行器。
多组件统一生命周期注册是否值得提炼，留给实际第二个产品验证。

阅读入口：[模板](../sdk/templates/README.md) → [SDK](../sdk/README.md)。
审查中尚未解决的自由度限制见 [SDK 审查](sdk-review.md)。
