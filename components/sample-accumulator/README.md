# sample-accumulator：最小任务型组件

完整、独立构建的 C++14 组件。只依赖 Roboframe::sdk；
不依赖 roboframe 应用、interface、integration、BT 或通讯协议。

## 公开文件和职责

- contract.hpp：Goal、Sample、Feedback、Result、Error 和契约版本。
- component.hpp：继承 SDK 最小生命周期接口；提供本组件自己的强类型端口。
- src/component.cpp：累计逻辑与已编译版本信息，不为简单加法再拆算法层。
- examples/main.cpp：算法开发者直接调用组件的入口，脱离 IPC 和 integration。
- tests/component_check.cpp：完成、取消、非法状态、溢出和复用验证。

Submit({id, sample_count}) 累计之后收到的 N 个样本。
每个样本产生一次 Feedback，满 N 个产生一次 Succeeded Result。
Cancel(id) 产生 Canceled Result，保留已经累计的部分结果。
溢出在修改统计前检测，产生 Failed Result；组件仍 Active，可接收新目标。

N 属于本次 Goal，不是算法配置。本组件没有配置项，所以不添加空 Config。
配置的示范见 SDK 的 value-scale 模板：构造时传入 gain，由组件自己校验。
新任务自动清空统计；不提供会与任务边界冲突的外部 Reset。

Component 只接受一个活动任务，忙碌时拒绝其他目标。
ID 非零，在本实例中严格递增；拒绝的目标不占用 ID。
有活动任务时不可 Deactivate。没有活动目标的 PushSample 被拒绝，不缓存到下一任务。

## 构建和直接调试

在 roboframe 根目录构建后，直接运行：

~~~sh
./build/components/sample-accumulator/accumulator_demo
~~~

独立构建只需已安装的 RoboframeSDK 1.x；在本组件目录运行：

~~~sh
cmake -S . -B build -DCMAKE_PREFIX_PATH=/path/to/sdk-prefix
cmake --build build -j2
./build/accumulator_demo
ctest --test-dir build --output-on-failure
cmake --install build --prefix /path/to/component-prefix
~~~

main 中只有创建组件、接回调、Activate、Submit、PushSample、检查结果、Deactivate。
修改它即可衔接自己的算法数据或调试器；这些数据无需先编成 IPC 消息。
框架外另建项目也可 find_package(SampleAccumulator 1.0 CONFIG REQUIRED)，
将自己的可执行目标链接 SampleAccumulator::component。

SAMPLE_ACCUMULATOR_BUILD_DEMO=OFF 关闭示例程序，BUILD_TESTING=OFF 关闭测试。
核心库不依赖测试和示例入口。本组件使用静态库，避免当前阶段引入动态插件问题。

## 调用、版本与回调契约

所有调用串行，回调同步；回调不能抛异常、重入或销毁组件。
回调参数仅在调用期间有效，留存时复制。无后台线程，也不保存历史结果。
每个已接受任务最多回调一次终态。持有者负责在销毁前结束活动任务；
析构不发隐式业务回调，主框架 Shutdown 已处理正常退出。

VERSION 是实现版本；contract.hpp 中 contract_version 是模块契约版本。
GetComponentInfo() 在库内实现，返回编译此库时的 SDK/实现/契约版本。
宿主在激活前调用 CheckCompatibility，独立 main 也示范了这一步。
日志统一使用 ROBO_LOG_INFO；组件不会更改宿主的日志级别或 sink。

Task 是本组件的可选成员，不要求持续里程计或其他所有组件都采用 Goal。
