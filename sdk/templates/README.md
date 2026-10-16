# 最小可编译示例

这些是参考实现，不是必须继承的框架。组件示例只依赖 SDK core，可由普通 main 直接使用；
integration 示例另行展示接线与 BT，复用前者，不再定义第二份组件。

| 目录 | 内容 | 直接验证 |
|---|---|---|
| component/ | StreamComponent：持有 ExecutionChannel；GoalComponent：持有 Server | stream_check.cpp、goal_check.cpp，不构造 Integration |
| integration/ | StreamIntegration：领取组件结果；BtIntegration：BT 调用组件目标接口 | stream_check.cpp、bt_check.cpp，检查装配、取消和重启 |

组件负责接纳、取消、等待及执行器析构。RequestDeactivate 是可选组件接口，
不是 Component 基类要求。示例中的字节数组、整型目标、容量和周期仅用于展示机制。

integration 负责生命周期协调与接线：先请求所有组件停止，再逐个等待。
它不使用 UseServer/UseExecution 接管内部执行器；这些旧 API 的兼容测试留在 sdk/tests。
GoalComponent 的 server() 只供示例访问目标/状态/取消/结果，生命周期仍由组件管理。
数据流组件自己的 epoch 与产品 generation 分开，停用时清理旧结果。
BtIntegration 的 result() 同时保留正常、取消和失败结果；ServerAction 结果回调在控制线程执行一次，
包括 settled()/Stop 收尾时。收尾完成不等于成功，检查 Result.state/error；回调不抛异常、不重入生命周期。

StreamIntegration 在主线程 Poll 领取结果，所以主线程停顿会延迟这段接线。
需要独立输出的组件可提供线程安全短回调；接收者须存活到组件停用，回调不等待 IPC。
具体算法、任务参数、树和结果解释由开发者实现，不由模板规定。

## 验证

源码 SDK 的测试包含上述四个示例；安装包也通过同一个 templates/ 入口构建：

~~~sh
cmake -S /path/to/sdk/share/roboframe/sdk/templates -B build-template -DCMAKE_PREFIX_PATH=/path/to/sdk
cmake --build build-template
ctest --test-dir build-template --output-on-failure
~~~

component/ 与 integration/ 按职责分组，共用父目录 CMakeLists.txt。
需要独立编译组件示例时，可只构建 stream_component_check 或 goal_component_check 目标。
旧 templates/integration 构建入口和混合头文件已移除；验证安装包时使用干净安装前缀。
