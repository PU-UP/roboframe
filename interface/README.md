# 通用 stdio 传输

本目录一直保留在 main：stdio_channel.hpp/.cpp 是实现，tests 是验证。
之前移除的是 products/demo_robot/interface 的旧产品协议适配器；
该适配器依赖 demo_robot 契约，完整版本保留在 demo 分支。

StdioChannel 是 POSIX 字节行传输，没有产品命令或组件依赖。
输入由一个 owner 调用 NextLine；Queue 可从组件线程并发调用，
内部有界队列由独立 writer 发送，replace_key 合并尚未开始发送的同类消息。

宿主 main 提供通信契约和编解码适配，可直接调用组件，也可绑定产品 integration。
独立使用或验证组件不需要创建 Integration，组件库不依赖该传输。
正常关闭顺序：停止所有生产者 → Drain → 销毁传输。
传输不拥有文件描述符；宿主保持描述符有效并负责关闭，处理 SIGPIPE
（例如在进程入口忽略，由错误返回检测断连）。

队列上限 20MiB、可靠消息 256 条、停滞/排空超时 2 秒是当前传输策略。
大消息和慢读会延迟后续消息，独立发送线程不代表实时送达。
组件输出回调只做短操作/入队；设备停止确认必须由真实产品实现。

tests/stdio_channel_test.cpp 不依赖任何产品，验证 owner 停顿时持续输出，
以及取消和异常后最终零值。没有在这里定义导航协议或真实 Controller。

## 独立构建

只构建通用传输不需要 SDK 或产品：

~~~sh
cmake -S interface -B build-interface -DBUILD_TESTING=OFF
cmake --build build-interface -j4
~~~

运行独立发送测试需要已安装的 SDK（测试用 Server 产生周期输出）：

~~~sh
cmake -S interface -B build-interface-test -DCMAKE_PREFIX_PATH="$PWD/build-packages"
cmake --build build-interface-test -j4
ctest --test-dir build-interface-test --output-on-failure
~~~

build-packages 的 SDK 安装步骤见根目录 README。运行库本身仅依赖标准线程库。
