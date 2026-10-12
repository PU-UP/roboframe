# 双向产品边界与 stdio v2

仅实现当前建图流程。integration 的业务入口与组件装配保持原结构，没有工作模式、导航、BT 或通用消息总线。

## 职责与文件

| 文件 | 职责 |
|---|---|
| sdk/include/roboframe/outputs.hpp | ProductOutputs 的具名回调、MappingEvent、MapView；只引用 SDK 和标准库 |
| integration/robot_integration.hpp/.cpp | 注册输出、公开业务入口、组件生命周期；不认识 IPC 编码 |
| integration/data_flow.cpp | odom 独立输出、全局校正/TF 接线、共享地图更新通知 |
| integration/mapping_flow.cpp | 开始、完成、取消、保存、加载的实际状态变化 |
| interface/stdio_interface.hpp/.cpp | 输入解析、响应/事件编码、订阅和快照读取 |
| interface/stdio_channel.hpp/.cpp | POSIX 非阻塞管道、行边界、有界队列、慢读与断连处理 |
| main.cpp | 装配 integration 和 interface 回调；销毁接收方前解除回调 |
| tools/mapping/replay.py | Python 持续读取输出，分流响应/事件；发送数据和业务请求 |

~~~mermaid
flowchart LR
  P[外部 Python] -->|REQ / DATA| I[interface]
  I -->|SDK 输入 / 产品意图| G[integration]
  G --> C[components]
  C -->|数据与结果| G
  G -->|具名回调 / SDK 数据| I
  I -->|RES / EVENT| P
~~~

integration 所有公共调用及输出回调都在同一串行调用线程，回调不得阻塞或重入；普通接收方不得抛异常。
内存分配失败按进程错误退出。当前只有一个输出接收端；不引入多订阅者注册表。
stdio 的回调只编码小消息并排队；地图通知只包含元数据。组件工作线程不直接写 stdout。
地图回调接收同一 GridSnapshot，可保留只读引用。跨 IPC 的完整地图会序列化，不宣称零拷贝。

## 线协议

启动输出 `READY roboframe-v2`。stdin/stdout 为独立方向，stderr 只用于日志。
输入每行最多 65536 字节。v2 替代原来的裸命令；SDK 发布版本与线协议版本独立。

~~~text
REQ 1 SUBSCRIBE ON
RES 1 OK SUBSCRIBE
REQ 2 CONFIG 20 20 1 -5 -5 100
RES 2 OK CONFIG
REQ 3 BEGIN 1
EVENT 1 TF unavailable
EVENT 2 MAPPING started 3 1 1 0 0 -
RES 3 OK BEGIN
DATA ODOM 0 0 0 0
EVENT 3 ODOM 0 0 0 0
DATA SCAN 0 1 2 0
REQ 4 FINISH
...
EVENT <seq> MAPPING finished 4 1 1 1 0 -
RES 4 OK FINISH
REQ 5 GET_MAP baseline
RES 5 MAP baseline <generation> <revision> <stamp_us> <frame> <width> <height> <resolution> <origin_x> <origin_y> <hex_cells>
REQ 6 QUIT
...
RES 6 BYE
~~~

示例事件序号仅用于说明；可合并的数据可能造成序号跳跃，接收顺序始终递增。
REQ ID 必须非零并在连接内递增。BEGIN 的 session ID 是建图任务标识，不是某次调用的请求 ID。
RES 对应一次调用，OK 表示当前同步操作成功返回；不是对未来异步任务完成的承诺。
MAPPING 事件描述已经发生的状态变化，finished/canceled 才表示建图会话终态；SAVE 独立报告保存结果。
响应和事件可以穿插，甚至事件先于对应响应到达，不允许客户端按“下一行就是我的回复”读取。

DATA 只接受 ODOM、SCAN、MAP_POSE，不逐帧确认。非法 DATA 或无法识别请求 ID 的输入返回 `RES 0 ERROR ...`，
有效 REQ 的参数错误返回对应 ID 的 ERROR；非法参数不执行操作。请求 ID 验证通过即消费，失败重试需新 ID。
可以使用 REQ ODOM/SCAN/MAP_POSE 获取逐次回执，但批量数据推荐 DATA。
其他命令及参数沿用 [建图协议](mapping.md) 和 [TF 契约](transforms.md)。

### 输出类别

| EVENT 内容 | 含义与策略 |
|---|---|
| ODOM stamp x y yaw | 独立 odom 输出；无需处于建图会话，也无需 scan 到达。保留最新待发值 |
| CORRECTION stamp x y yaw | lidar-map 的 T_map_odom 输出。保留最新待发值 |
| TF valid scan_stamp correction_stamp x y yaw / TF unavailable | 最近处理 scan 的 T_map_base 或该链不可用；不是墙钟定位健康状态。保留最新待发值 |
| MAP generation revision stamp frame width height resolution origin_x origin_y | 当前 live 快照已更新；保留最新通知 |
| MAPPING kind request_id session generation integrated dropped name | started/progress/finished/canceled/saved/loaded/stopped；只有 progress 可合并 |

MAPPING 的 request_id 为触发该同步事件的请求，DATA、EOF 引发的事件为 0；session 记录当前/最近建图会话，
loaded/stopped 不创建新任务。今后的异步任务依赖任务 ID 关联，不能沿用瞬时 request_id。
进度合并仅表示最新计数；可靠业务事件保留产生当时的值。
地图 generation 只在同一进程内标识地图上下文，每次 BEGIN 或成功 LOAD 增加，revision 标识其中快照版本。
新 generation 不能与旧地图/TF 混用。连接重建后重新获取状态；它不是持久化地图资产 ID。
加载地图不表示已定位。CANCEL 清理可获取的本次地图；已交给消费者的旧快照仍然有效。

默认不订阅主动事件。SUBSCRIBE ON 从此刻接收后续变化，已有状态可用 STATUS、TF、GET_MAP 查询。
SUBSCRIBE OFF 删除尚未发送的连续数据；已经部分发送的消息会完整结束，已排队的可靠事件先发完。
OFF 的响应之后不再发送新事件；重新 ON 不补发历史事件。

GET_MAP live/baseline 返回调用当时的完整只读快照，包含实际 generation/revision；不保证与先前通知版本相同。
hex_cells 是行优先、y 向上的原始字节十六进制编码，0/254/205 表示占据/空闲/未知。
最大地图 8,000,000 格，完整响应约 16 MB；不是每次地图更新都发送完整地图。
编码在 interface 调用线程进行，有上限但不是恒定耗时。大地图高频外传时再考虑分块/二进制协议。

## 背压、断连与关闭

- 同一个串行 poll 循环负责收发；文件描述符设为 O_NONBLOCK，析构恢复原标志。
  不增加算法线程、发送线程或锁。每轮最多写 64 KiB，随后处理输入。
- 队列最多 20 MiB 编码文本，可靠消息最多 256 项（包括正在发送的消息）；连续输出只有五种替换键。
  该上限不包括构造响应时的临时字符串、内核管道或地图本身，不是 RSS 上限。
- 只替换尚未写出的同类数据；已写出部分的消息保持完整。输入顺序、scan 对时、地图生成不受替换影响。
- 响应、开始/终态/保存等事件不能被遥测覆盖。可靠队列满则连接失败、停机并以非零码退出，stderr 说明原因。
  不能在连接失效后承诺剩余消息送达，也不将这些消息伪装成已确认。
- 有待发数据且连续 2 秒无写进展判定慢读失败；即使输入空闲也会检查。
  输出管道读端关闭会被 poll/write 检出，不需要等下一条输入。
- QUIT 或 stdin EOF 先关闭组件，允许剩余消息最多排空 2 秒。正常 QUIT 有 BYE；EOF 有 stopped 事件但无 BYE。
  不隐式 FINISH/SAVE。截断的末行不执行，报告 ERROR 后关闭。失败路径解除回调并清理组件。
- 这些时间检查在现有串行 integration 调用之间执行；组件计算或文件写入仍可能阻塞调用线程，
  不保证抢占长算法任务，也不代表控制周期或硬实时支持。未来 controller 调度仍需单独设计。
- IPC_STATUS 提供编码队列的 bytes/peak_bytes/pending/replaced，用于本机验收；不属于 SDK 算法状态。

## 验证

~~~sh
cmake -S . -B build-product -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tools/mapping/replay.py --dataset tf --scans 0 --finish-after 12267 --output build-product/ipc-result
~~~

Python 有一个持续接收线程和有界响应队列，事件写入 JSONL，不在内存堆积所有历史事件。
回放使用 DATA 输入、带 ID 的业务请求，验证主动事件出现，并通过 GET_MAP 对比基准/实时地图与保存文件的全部字段和栅格。
stdio_publication_test 使用真实 4 KiB 管道，覆盖无查询发布、退订、快照/地图上下文、暂时慢读、持续不读、
可靠队列溢出、读端断连、EOF、QUIT 排空期限和非法/截断输入；mapping_check 验证进程内回调收到共享快照。
SDK 4.1 仅新增输出契约；现有组件数据布局未改。产品选择 SDK 4.1，组件仍由构建记录 SDK 身份。

本轮验收：13 项 CTest（其中管道专项含 9 个场景）及 ASan/UBSan 通过；五个组件独立安装、混合构建和
24 个公开头文件隔离编译通过。全量 13,631 帧输入、910 次校正，收到 41,308 条主动事件；
基准与实时地图通过 IPC 逐字段/逐栅格校验，落盘文件与上一轮 TF 回放逐字节一致。
事件数量会随消费者速度及合并情况变化，不代表必须交付每一帧遥测。
