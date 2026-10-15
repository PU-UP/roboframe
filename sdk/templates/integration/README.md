# 可编译的最小模板

两个示例采用同一所有权约定：integration 持有组件，组件按需持有 SDK 执行器。
它们是复制/阅读材料，不是必须继承的第二层框架；整型和字节数组只是测试输入。

| 需求 | 从这里开始 |
|---|---|
| 连续输入、异步处理 | stream_component.hpp → product_integration.hpp → check.cpp |
| 单目标、周期任务、BT | server_product.hpp → server_check.cpp |

组件管理接纳、取消、等待、旧结果清理和执行器析构。
integration 的 OnStart 调用组件 Activate；CloseInputs 请求所有组件停用；
OnStop 等待 Deactivate 后处理任务状态。多组件时先请求全部退出，再逐个等待。
RequestDeactivate 是示例组件的具名接口，不是 Component 新增的强制虚函数。

数据流组件自己的 epoch 与产品 generation 分开，停用时丢弃旧结果；
产品无需访问内部 ExecutionChannel。模板容量 8、单帧 1MiB 不是 SDK 全局策略。
结果由主线程 Poll 交给下游，所以主线程停顿仍会延迟这种接线。
要求独立输出的组件可提供线程安全输出回调；接收者必须存活到组件停止，
回调只做短操作或有界入队，不要在算法线程等待 IPC。

Server 示例组件通过 server() 提供目标/状态/取消接口供 ServerAction 使用；
线程和清理仍由组件管理。具体树、任务参数与结果解释属于 integration。
不登记 UseServer/UseExecution，不让 integration 接管组件内部执行器。

源码 SDK 测试会编译运行两个模板。使用安装后的 SDK 也可独立验证：

~~~sh
cmake -S /path/to/sdk/share/roboframe/sdk/templates/integration -B build-template -DCMAKE_PREFIX_PATH=/path/to/sdk
cmake --build build-template
ctest --test-dir build-template --output-on-failure
~~~
