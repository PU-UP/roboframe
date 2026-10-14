# 最小 integration 使用模板

先读 product_integration.hpp，再运行 check.cpp。这里注入计算函数，不虚构第二套算法组件。
Input/Output 是示意类型，新产品替换为实际公开接口，组件对象及其回调由 integration 装配。
不是要求真实产品继承 ProductIntegration，也不要复制 SDK InputGate/ExecutionChannel 的实现。

读代码的顺序：构造中的装配 → Push 的输入接纳 → OnPoll 的结果分发 → 生命周期钩子。
容量8、单帧1MiB 是这个模板的明确资源策略，不是全框架默认上限。
短输入锁内只投递，耗时计算在执行组；输出回调由 Poll 所在线程执行。
STOP 会拒绝新输入并协作取消；RESET 后重启会过滤旧代次结果。
同一非线程安全组件的多个输入归一个串行上下文；不同独立慢通路可以再使用执行组。

源码 SDK 的 CTest 自动验证此模板。安装 SDK 后，也可单独构建该目录：
~~~sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/sdk
cmake --build build
ctest --test-dir build --output-on-failure
~~~
