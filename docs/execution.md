# 执行边界：明确接线、并发交付、有序状态

integration 是产品的装配与编排层，不是一条线程。组件的算法身份不决定执行机制。
SDK 提供可复用的有界执行通道；当前产品用它连接实际算法，不再为某种算法编写专用 worker。

## 三个边界

| 位置 | 职责与线程归属 |
|---|---|
| interface | 解码、校验、交付输入、编码输出。stdio 当前只有一个接收线程；ROS/DDS 可以有多个数据接收线程。 |
| integration 数据通路 | `PushOdometry/PushScan/PushMapReference` 可以从不同线程并发调用，也可以与业务指令同时执行。产品的 SensorFlow 只管理相关传感器的对时、TF 和路由状态。 |
| integration 业务状态 | 配置、查询、开始/结束/取消/保存/加载、SetOutputs、Poll 由一个业务 owner 有序执行。该限制不覆盖传感器入口，也不覆盖算法执行。 |
| component | 按明确输入输出处理算法。可以同步调用、已有内部异步，或按需使用 SDK 执行通道；不要求所有组件独占线程。 |

~~~mermaid
flowchart LR
  S[并发传感器入口] --> R[明确类型的产品接线]
  R --> A[A 的有界执行通道]
  R --> B[B 的有界执行通道]
  A --> RA[类型明确的结果]
  B --> RB[类型明确的结果]
  C[有序业务指令] --> G[会话和任务状态]
  G -->|配置 / 关闭交付 / 取消| R
  RA -->|业务需要的结果| G
  RB -->|业务需要的结果| G
~~~

SensorFlow 的短锁只保护这组相关传感器的缓存和路由状态，不覆盖算法计算、用户回调、文件 I/O、等待线程退出。
无关联的数据通路不需要经过这把锁。它是本产品的数据接线，不是 SDK 的万能数据分发器。
没有广播到所有组件，没有 common::Data/any，没有动态字符串类型判断。
输入锁内仅调用当前已知快速、同步的适配组件；真实耗时算法必须配置执行边界，不能直接替换后继续在输入锁内运行。
同一个不可变 scan 引用经录制 lidar-map、地图算法和 planner 读取；输入转换只在边界做一次。

## SDK 的最小公共机制

`execution.hpp` 提供 `ExecutionChannel<Input, Output>`，不引用组件、地图、建图状态或产品配置。
构造时指定容量、处理函数和过载策略；类型在编译时确定。工作线程只调用处理函数并交付结果。

- `Submit(generation, input)` 可由多个生产者调用，立即返回 accepted/replaced/full/closed/stale；不会等待算法或等待空位。
- `Take(result)` 获取完成结果，包含原始输入、输出、generation、取消标志和异常；只取结果，不隐式调用业务代码。
- 容量包含 **待处理 + 执行中 + 已完成待取**，不是三份各自无限扩展的队列。
- 默认 reject_newest 保留已接受 FIFO；latest_pending 只替换未开始的待处理输入，不取消当前计算，不覆盖完成结果。
- `Reset(new_generation)` 清除待处理输入、取消旧计算。非零 generation 必须递增，防止旧任务重新有效；0 表示禁止交付。
- 处理函数通过 `Cancellation` 检查取消；`WaitFor` 提供可中断等待。结果消费者必须检查 generation，不能把已完成的旧结果视为当前结果。
- `Stop` 关闭提交、唤醒等待、join、释放结果；不能强杀不合作的算法。生命周期操作由一个管理者调用。
- `WaitIdle` 仅供独立测试/离线客户端；接收线程和业务事件循环不能调用它等待计算。

这是一个按需创建的串行执行组：一个处理函数可以串接多个快速组件；不同的慢计算组使用不同实例并行执行。
它不自动为每个组件创建线程，不提供线程池、插件注册、动态执行图或任务优先级调度。
同步算法独立 main 无需使用通道。已有异步组件也不必再套一个异步层。
输入/输出应为小型值或不可变共享引用；通道只管理交付，不深拷贝大数据、不管理算法私有缓存。

处理函数也可以将 typed 输出提交到另一个通道，不需要先回业务线程；此时须显式处理下游 Admission。
当前产品选择由 Poll 汇集地图结果并发布，因为这一步同时决定基准地图和建图终态。这是产品选择，不是 SDK 对全部模块的限制。

## 当前产品如何使用

| 文件 | 内容 |
|---|---|
| sdk/include/roboframe/execution.hpp | 唯一公共执行机制：有界容量、过载、异常结果、代际取消、关闭。 |
| integration/sensor_flow.hpp/.cpp | 并发 typed 入口；快速 odom/lidar-map 接线、64 帧对时队列、512 个 pose、完整 TF 匹配。 |
| integration/robot_integration_impl.hpp/.cpp | 装配算法与 `ExecutionChannel<ScanWork, MapResult>`；处理函数中只有调用算法、Reset 和生成结果。 |
| integration/data_flow.cpp | 业务 owner 收取结果、会话检查、状态统计、共享地图发布。 |
| integration/mapping_flow.cpp | 业务流程与数据接收门禁的有序切换。 |
| components/planner/... | 复用同一个 ExecutionChannel，删除原先重复的线程循环；仍仅作共享读取验证。 |

地图计算通道总容量 8；planner 通道总容量 2，采用 latest_pending。planner 收取结果后再提交新输入，
保留“最多一个执行中、一个最新待处理”的运行方式；算法读取与锁分离。
某个通道满时不会等待其他通道，也不会影响传感器入口继续接收。相同输入流仍需由生产者保持时间顺序；
并发安全不等于自动按时间戳重排。对时使用采集时间，不能选择当前最新 pose 代替对应时刻的变换。

当前 PushScan 返回表示校验和接线尝试完成，不承诺一定计算；对时超限和计算过载通过 STATUS/RESOURCES 计数。
SCAN 太快时，64 帧对时队列丢最旧未配对数据；计算通道满则拒收最新计算输入。每帧最多 2048 点。
任务指令仍由业务 owner 串行处理，不与 scan 共用丢帧队列。多线程中间件应只序列化业务命令，不应把全部数据回调也串行化。

对外 ProductOutputs 在业务 owner 上执行。SensorFlow 为 odom/correction/TF 各保留一个最新待发值；
不 Poll 时不会无限累积遥测，也不会停止算法路由。业务事件由 owner 直接产生，不被遥测合并。
因此并发生产者不直接调用 stdio 对象，SetOutputs 也不与传感器线程争用可变回调。
Poll 的每轮计算结果预算为 8，stdio 每条输入前和空闲轮询期间调用它。

## 接受、完成、取消与关闭

stdio v3 保持不变：FINISH 返回 ACCEPTED，随后由同 session 的 finished/failed 事件报告完成。
finishing 期间忽略新 scan/reference，odom 继续接收和路由其有效输出；SAVE 只在 ready 成功。
完成后冻结基准，后续 scan 可继续更新 live 图；这是当前产品流程，不是所有组件的统一业务状态机。

切换顺序是：关闭旧路由交付 → 记录未完成数量 → 使旧执行 generation 失效 → 打开新 generation。
输入门禁和缓存操作使用同一条短锁形成明确边界；业务状态不由传感器线程修改。
取消后的旧结果仍占原容量直到被取走；Poll 检查 generation 后只计 stale_results，不发布旧地图、不修改新会话统计。
算法部分修改由下一代 Reset 清理；已发布不可变快照仍可供旧读者读取。

Configure 完成后才启动输入生产者。Shutdown 可与输入调用并发：先关闭输入，后停止并 join 计算；
后来的输入抛出“已关闭”。对象析构前调用方必须结束并 join 所有生产者，不能在对象销毁后继续调用成员方法。
业务方法彼此仍不能并发调用；这里没有给整套 integration 加一把全局互斥锁。

SAVE/LOAD/SNAPSHOT 和完整地图编码仍在业务 owner 同步运行，可能延迟下一条业务指令；
其他线程的传感器交付和计算不因此持有业务锁，但结果未消费时会达到已声明容量。不能据此宣称硬实时或无限吞吐。

## 验证

~~~sh
cmake -S . -B build-product -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tests/build_isolation_test.py
python3 tools/mapping/replay.py --dataset tf --scans 0 --finish-after 12267 --output build-product/execution-result
~~~

- SDK execution_test 使用同一不可变输入连接两个独立通道：一个等待 10 秒，另一个正常完成；四个并发生产者压满慢通道，容量仍为 8。还检查处理异常、旧代结果、generation 禁止复用、关闭唤醒及关闭后的拒收。
- concurrent_ingress_test 用两个真实输入线程与业务线程同时运行，地图计算延迟 5 秒；持续接收期间取消并切换 19 次会话，关闭后两个生产者收到明确拒收。
- mapping_execution_test 核对结果/scan 引用总量、旧成功结果丢弃、部分取消后的 Reset、失败终态和业务回调线程。
- 真实 IPC 测试保留慢计算、慢读、断连、EOF/QUIT、会话切换；独立 planner 测试验证 latest_pending 和共享数据不变。
- Python 离线回放每 4 帧等待空闲，验证确定性地图内容；过载测试有意不节流，不能混作硬件吞吐结论。

SDK 5.1 新增 execution.hpp，不改变已有 SDK 数据布局；组件构建仍自动记录实际 SDK 身份。
旧产品公开入口的 mapping_status 改为返回值快照，调用方应重编；不要跨线程持有可变状态引用。
SDK 与 product 的输出机制不会无限保存历史快照；外部消费者若永久持有快照，其内存仍由消费者负责。

本次验收：17 项常规测试和相同测试的 ASan/UBSan 通过；五组件独立安装、混合源码/包构建、
25 个公开头文件隔离编译通过。全量回放 13,631 帧、910 次校正，计算过载拒收为 0，
计算通道持有峰值 3 / 8，planner 峰值 2，读取不一致为 0。17 份地图文件与上一版逐字节一致。
168 帧初始 TF 未就绪跳过与之前一致；结束前融合 12,099 帧、结束后更新 1,364 帧。
产物：`build-product/execution-verified/replay.html` 和 `summary.json`。
当前没有 TSan 通过结论（此前 WSL 初始化失败）；ASan/UBSan 和并发测试不等于证明不存在所有数据竞争。
