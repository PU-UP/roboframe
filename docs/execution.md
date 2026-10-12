# 当前执行模型：串行产品入口，按需异步计算

目的：慢地图计算不能阻塞后续传感器输入和取消命令。integration 仍不支持直接并发调用；
没有通用调度器、线程池或每组件一线程的要求，也没有新增规划控制。

## 线程与职责

| 边界 | 当前执行方式 | 约束 |
|---|---|---|
| interface/stdio_channel、stdio_interface | 主线程非阻塞收发、解析和编码 | 每次处理一行之前服务计算完成队列；空闲时最多 5 ms poll 一次。没有无限输入消息队列。 |
| integration 的所有公开调用（包括查询、Poll） | 同一 owner 主线程 | 接线、对时、状态变化、结果筛选和 ProductOutputs 回调都在这里；不是线程安全对象。 |
| PassthroughOdomComponent、RecordedLidarMapComponent | owner 上同步执行 | 当前实现只是校验/录制数据转发，返回快；换成真实耗时算法时需重新选择执行边界。 |
| MapUpdateWorker（integration 私有适配器） | 一条计算线程独占 OccupancyMapComponent | 提交不可变 scan + 已匹配 TF；只通过受限输入/结果队列与 owner 通信，绝不从 worker 调用产品回调。 |
| OccupancyMapComponent | 保持可独立调用的同步算法 API | 不管理产品会话、不自行创建线程；可选取消谓词用于长循环，取消后必须 Reset 才能复用。 |
| ReadOnlyPlannerComponent | 原有异步只读线程 | 仅验证共享快照，不新增规划能力。 |
| map-server、完整地图编码 | owner 上同步执行 | 当前 SAVE/LOAD/SNAPSHOT/GET_MAP 仍可能耗时；这是明确保留的边界，不能宣称整个进程硬实时。 |

~~~mermaid
flowchart LR
  IPC[IPC 输入] --> I[interface 主线程]
  I --> G[integration owner: 接线 / 状态 / 对时]
  G --> O[快速 odom / 录制 lidar-map]
  O --> G
  G --> Q[有界提交: scan + TF + generation]
  Q --> W[地图工作线程: 同步 component]
  W --> R[有界完成队列: 不可变结果]
  R --> P[owner Poll: 检查 generation]
  P --> G
  G --> CB[owner ProductOutputs 回调]
  CB --> OUT[interface 有界输出]
~~~

关键文件：`integration/map_update_worker.hpp/.cpp` 负责计算通道；`data_flow.cpp` 负责提交和 Poll，
`mapping_flow.cpp` 负责产品会话；`stdio_channel.cpp` 只提供 owner 服务时机，不认识地图。
组件独立测试可以直接同步调用算法，无需依赖 integration、IPC 或后台线程。
未来多个耗时组件可选择内部异步或私有适配器；有实际需求才共享线程池。把全部 integration 搬到一个线程仍会串行堵住所有业务，不能替代计算隔离。

替换 ROS/DDS 时：其多个接收回调不能直接并发调用 integration。适配层必须串行交付输入、业务调用与 Poll；
并为自己的跨线程输入队列定义容量和过载规则。std::shared_ptr 的线程安全不等于 integration 线程安全。

## 接收、计算和完成

1. `PushScan/PushOdometry` 校验并接线，不等待地图计算；输入有效并不代表一定计算。
   scan 等待配套 pose；匹配后生成确定的完整 TF，计算线程不再读取“最新 pose”。
2. 计算通道接受的是共享只读 scan、值类型 TF 与 generation；一个有状态 mapper 按接受顺序计算。
   算法私有 evidence 只由 worker 修改，完成后交付分块写时复制的不可变快照。
3. worker 完成不改变产品状态、不发产品回调。owner 定期 `Poll()`，每次最多取 8 个结果，
   判断 generation 后计数、发布地图及任务事件。因此独立 main 也必须在自己的串行循环中调用 Poll。
4. `ProductOutputs` 全在 owner 线程，不得阻塞、抛异常或重入 integration。需要耗时处理的消费者应自己有界接收。

stdio **v3**：`REQ id SCAN ...` 的 `OK SCAN` 表示调用已返回，不能视为计算成功回执；
`DATA` 不逐帧应答。通过 STATUS/RESOURCES 观察已融合、丢弃、待计算和待取结果。

`FINISH` 处理现存对时队列并返回 `ACCEPTED FINISH`，进入 `finishing`；不等待已接受的地图计算。
此时新的 scan/MAP_POSE 不加入本次地图，odom 仍正常输出。完成后 Poll 发 `finished`，冻结基准并进入 ready；
ready 后的 scan 继续更新 live 地图，沿用此前共享验收的产品策略。无有效结果或计算失败发 `failed`。
`SAVE` 只在 ready 成功；`SNAPSHOT` 保存最近已完成版本，不等待排队数据。
异步 finished/failed 事件的 request_id 为 0，以 session 关联；接受不是完成，完成也不是保存。
`WAIT_PLANNER` 现在仅检查空闲，忙时立即返回 pending 错误，由调用方稍后重试。

## 容量、过载与内存

| 资源 | 上限与策略 |
|---|---|
| scan 等待 odom | 64 帧；满时丢最旧未配对帧，保留较新输入；每帧最多 2048 点。 |
| pose history | 512 项；原有时间匹配及过期规则不变。 |
| 地图计算通道 | **排队 + 执行中 + 已完成待取结果合计 8 项**；满时拒收最新计算输入，已接受的 FIFO 保留。 |
| planner | 一份执行中、一份最新待处理；新提交替换旧待处理项。 |
| IPC 输出 | 20 MiB 编码文本、256 条可靠消息；连续遥测合并，可靠队列耗尽或持续不读则明确连接失败。 |

通道锁只覆盖取放和少量状态，不覆盖算法，也不等待队列腾出空间。结果使用提交时的容量信用，
不会因为 owner 暂时没取结果形成另一个无限队列。取消清除排队输入；已完成/执行中的旧结果仍占原来的信用，直到 owner 取走丢弃。
`compute_pending/active/results/retained/peak_retained/rejected/stale_results` 可用于核对行为；
`pending()` 只统计当前会话待对时和未收取的计算，旧结果持有量以 compute_retained 为准。

队列容量有界不是固定 RSS：地图最大 8,000,000 格，mapper 仍需私有 evidence/observations/工作栅格，
integration 保留完成、live、baseline 快照，planner 保留至多两项；取结果时可临时持有一项。
快照共用未修改块；每次完成生成快照可能比原先按发布周期取快照多复制块，统计不冒充零拷贝。
应用主动永久保存历史快照仍会增长内存，需由应用限制；框架不会使外部引用失效。

## 取消、会话切换与关闭

`CANCEL` 在 owner 改变业务状态，清空待对时和待计算输入并使当前 generation 失效，立即返回；
它表示该任务已不再有效，不表示底层线程在返回瞬间已经停止。worker 在取消检查点结束当前更新。
每个结果携带提交时的 generation；新 BEGIN/LOAD 改变 generation，迟到结果只增加 stale_results，
不增加新会话 integrated、不覆盖地图、不发旧 finished。部分更新的 mapper 在下一会话计算前 Reset。
已经发布给外部消费者的旧不可变快照可以继续读，消费者仍应核对其 generation。

QUIT、EOF 或 IPC 失败会停止接收、请求取消、唤醒并 join worker，再解除/关闭其余组件；
不隐式完成或保存地图，不使用 detach 或强制杀线程。计算延迟注入也可被取消/关闭唤醒。
真实算法的协作取消点位于每条射线及每 256 个待更新格之间；Reset、分配和单次射线本身不可抢占。
因此当前关闭可控但不是硬实时期限；以后接入不支持取消的第三方阻塞调用时必须重新评估其关闭契约。

## 验证与复现

~~~sh
cmake -S . -B build-product -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tests/build_isolation_test.py
python3 tools/mapping/replay.py --dataset tf --scans 0 --finish-after 12267 --output build-product/async-result
# 手动注入每次地图计算 1000 ms 的延迟（默认 0，最大 5000）
./build-product/robot_runtime /tmp/existing-map-directory --map-delay-ms=1000
~~~

Python 回放持续接收事件，每 4 帧等待计算空闲，FINISH 后等同 session 的 finished 再 SAVE；
这保证离线几何回归不因灌入速度而丢帧，不是实时吞吐成绩。过载测试有意不采用这类节流。

- `mapping_execution_test.cpp`：1000 帧不取结果，8 个完成项占满通道，992 个新任务被拒收；
  weak_ptr 核实仅有 8 份 scan 被持有，丢弃旧结果后全部释放；owner 未 Poll 前没有地图回调。取消后开启新会话，8 个已成功完成的旧结果全部丢弃，新地图没有旧占据格。
  回调线程检查、FINISH 两阶段及 5 秒延迟期间的快速输入/取消/关闭同时覆盖。
- `mapping_execution_test.py`：真实管道注入 1 秒计算延迟，200 组数据与状态查询在 0.7 秒内返回；
  队列最多 8 项。FINISH 等待中仍可 CANCEL/BEGIN；stdin 空闲时也能发布 finished。
  QUIT/EOF/输出断连在 5 秒延迟期间均在 1 秒内退出。
- mapper 单测验证取消部分射线后 Reset，再计算的栅格与未中断参考一致。
- 原有慢读、持续不读、可靠队列溢出、保存/加载、共享快照和 TF 测试继续运行。

这些时间阈值是当前测试环境的回归检查，不是实时调度保证。SDK 5.0 改变公开完成语义和状态布局，
stdio v3 改变 FINISH 回执，旧客户端及旧 major 组件包需更新/重建。

本次实测：15 项 CTest 与相同测试的 ASan/UBSan 均通过；五组件独立构建/安装、混合包构建、
24 个公开头文件隔离编译通过。全量 13,631 帧、910 次校正，建图融合 12,099 帧，
168 帧因初始 TF 未就绪跳过（与之前一致），完成后更新 1,364 帧；计算过载拒收为 0，通道最大持有 4 项。
17 份落盘地图（含全部检查点、基准、实时及重载）与上一轮同步 IPC 回放逐字节一致。
本次累计地图块复制 276.52 MiB；planner 持有上限 2、不一致为 0。
产物：`build-product/async-verified/replay.html`、`summary.json`、`mapping-events.jsonl`。
未新增 TSan 通过结论：此前当前 WSL 环境无法初始化 TSan；ASan/UBSan 不能替代数据竞争检测。
