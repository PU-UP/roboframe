# stdio v4：运行控制与可选任务

CONFIG 只配置产品；运行由 RUN / STOP / RESET 控制。RUNTIME 查询返回 IDLE / RUNNING / ERROR 和运行代次。
SUBSCRIBE ON 后主动收到 EVENT id RUNTIME state generation。协议适配只依赖 SDK + 产品公开契约，由 main 绑定 integration。

~~~text
REQ 1 CONFIG 100 100 0.2 -10 -10 250000
REQ 2 RUN
DATA ODOM 0 0 0 0
DATA SCAN 0 1 2 0
REQ 3 BEGIN 1
...
REQ 4 CANCEL
DATA ODOM 100000 1 0 0
DATA SCAN 100000 1 2 0
REQ 5 STOP
REQ 6 RESET
REQ 7 RUN
~~~

无 BEGIN 时照常处理数据；CANCEL 只取消任务。STOP 拒绝新的算法输入并协作取消在途计算，
RESET 清运行数据并留在 IDLE。FINISH 仍异步返回 ACCEPTED，等待 MAPPING finished/failed。
LOAD 要求 IDLE，仅提供 GET_MAP baseline；不能把加载栅格当成已重定位。
v4 不兼容旧脚本的“CONFIG 后直接 BEGIN”，Python 回放已显式调用 RUN。
详细语义见 [运行骨架](architecture.md)。

## 绑定与推进

DemoStdioInterface 只依赖 SDK + DemoRobot::contract；main 绑定输入/输出，驱动 ProcessNext 与 Poll。
传输层复用、线程和关闭职责见 [架构](architecture.md)。当前产品在连接结束时停止组件，
正常路径排空输出；异常路径先停止组件，再解除输出回调；传输失败不承诺送达。interface 不自行调用 Shutdown。

## 线协议

启动输出 `READY roboframe-v4`。stdin/stdout 为独立方向，stderr 只用于日志。
输入每行最多 65536 字节。SDK 发布版本与线协议版本独立。

~~~text
REQ 1 SUBSCRIBE ON
RES 1 OK SUBSCRIBE
REQ 2 CONFIG 20 20 1 -5 -5 100
RES 2 OK CONFIG
REQ 3 RUN
EVENT 1 RUNTIME RUNNING 1
RES 3 OK RUN
REQ 4 BEGIN 1
EVENT 2 MAPPING started 4 1 1 0 0 -
RES 4 OK BEGIN
DATA ODOM 0 0 0 0
DATA SCAN 0 1 2 0
REQ 5 FINISH
RES 5 ACCEPTED FINISH
...
EVENT <seq> MAPPING finished 0 1 1 1 0 -
REQ 6 GET_MAP baseline
RES 6 MAP baseline <generation> <revision> <stamp_us> <frame> <width> <height> <resolution> <origin_x> <origin_y> <hex_cells>
REQ 7 QUIT
EVENT <seq> RUNTIME IDLE 1
...
RES 7 BYE
~~~

示例事件序号仅用于说明；可合并的数据可能造成序号跳跃，接收顺序始终递增。
REQ ID 必须非零并在连接内递增。BEGIN 的 session ID 是建图任务标识，不是某次调用的请求 ID。
RES 对应一次调用，OK 表示当前同步操作成功返回；不是对未来异步任务完成的承诺。
MAPPING 事件描述已经发生的状态变化，finished/failed/canceled 才表示建图会话终态；SAVE 独立报告保存结果。
响应和事件可以穿插，甚至事件先于对应响应到达，不允许客户端按“下一行就是我的回复”读取。

DATA 只接受 ODOM、SCAN、MAP_POSE，不逐帧确认。非法 DATA 或无法识别请求 ID 的输入返回 `RES 0 ERROR ...`，
有效 REQ 的参数错误返回对应 ID 的 ERROR；非法参数不执行操作。请求 ID 验证通过即消费，失败重试需新 ID。
可以使用 REQ ODOM/SCAN/MAP_POSE 获取逐次调用回执，但批量数据推荐 DATA。
OK SCAN 仅表示输入已校验并完成路由尝试，不承诺已计算或一定入队；过载见 STATUS/RESOURCES。
FINISH 返回 ACCEPTED，进入 finishing，直到 finished/failed；没有已完成基准时 SAVE 拒绝；已有基准仍可保存，CANCEL 仍可执行。
### 输入命令

| 命令 | 参数与作用 |
|---|---|
| CONFIG | width height resolution origin_x origin_y max_pose_gap_us [MAP_TF max_correction_age_us]；配置一次，失败可重试 |
| RUN / STOP / RESET / RUNTIME | 启动/停止/清工作状态/查询运行状态和 generation |
| BEGIN | 严格递增的正 session ID |
| FINISH / CANCEL / STATUS | 异步结束/取消任务/查询任务状态 |
| ODOM / MAP_POSE | stamp_us x y yaw；局部位姿/录制修正位姿 |
| SCAN | stamp_us count x1 y1 ...；base 坐标系命中点 |
| SAVE / SNAPSHOT / LOAD | name；保存基准/保存实时地图/在 IDLE 加载基准 |
| TF / RESOURCES | 查询有效变换链/产品诊断统计 |
| GET_MAP | live 或 baseline |
| SUBSCRIBE | ON 或 OFF |
| IPC_STATUS / QUIT | 查询传输队列/关闭连接 |

STATUS 返回字段依次为：
phase session received integrated ignored dropped pending used_rays outside_rays
map_revision drop_sync drop_queue drop_bounds drop_cancel。
RUNTIME 返回 state generation。
TF 返回 odom_stamp correction_stamp odom_x odom_y odom_yaw corr_x corr_y corr_yaw
global_x global_y global_yaw，或 unavailable。
地图和坐标含义见 [demo_robot 示例产品](demo_robot.md)。

### 输出类别

| EVENT seq 后的内容 | 含义与策略 |
|---|---|
| RUNTIME state generation | 运行状态，与可选任务分开；可靠事件 |
| ODOM stamp x y yaw | 独立 odom 输出；无需处于建图会话，也无需 scan 到达。保留最新待发值 |
| CORRECTION stamp x y yaw | PoseGraph 根据 SLAM 参考位姿计算的 T_map_odom。保留最新待发值 |
| TF valid pose_stamp correction_stamp x y yaw / TF unavailable | 最近处理 odom 或 scan 的 T_map_base 或该链不可用；不是墙钟定位健康状态。保留最新待发值 |
| MAP generation revision stamp frame width height resolution origin_x origin_y | 当前 live 快照已更新；保留最新通知 |
| MAPPING kind request_id session generation integrated dropped name | started/progress/finishing/finished/failed/canceled/saved/loaded/stopped；只有 progress 可合并 |

MAPPING 的 request_id 为触发该同步事件的请求，DATA、EOF 引发的事件为 0；session 记录当前/最近建图会话，
loaded/stopped 不创建新任务。异步 finished/failed 的 request_id 为 0，客户端以 session 关联完成，不沿用瞬时 request_id。
进度合并仅表示最新计数；可靠业务事件保留产生当时的值。
地图 generation 表示生成它的运行上下文，每次 RUN 增加；BEGIN/CANCEL 不改变它。加载的资产标为 0。
保留的基准快照保持创建它的 generation，revision 标识其中快照版本。
新 generation 不能与旧地图/TF 混用。连接重建后重新获取状态；它不是持久化地图资产 ID。
加载地图不表示已定位。CANCEL 不清实时地图或上一份基准；已交给消费者的旧快照仍然有效。

默认不订阅主动事件。SUBSCRIBE ON 从此刻接收后续变化，已有状态可用 STATUS、TF、GET_MAP 查询。
SUBSCRIBE OFF 删除尚未发送的连续数据；已经部分发送的消息会完整结束，已排队的可靠事件先发完。
OFF 的响应之后不再发送新事件；重新 ON 不补发历史事件。

GET_MAP live/baseline 返回调用当时的完整只读快照，包含实际 generation/revision；不保证与先前通知版本相同。
hex_cells 是行优先、y 向上的原始字节十六进制编码，0/254/205 表示占据/空闲/未知。
最大地图 8,000,000 格，完整响应约 16 MB；不是每次地图更新都发送完整地图。
编码在 interface 调用线程进行，有上限但不是恒定耗时。大地图高频外传时再考虑分块/二进制协议。

## 背压、断连与关闭

- 主线程接收请求并驱动 Poll；独立发送线程唯一写出 stdout，文件描述符为 O_NONBLOCK，析构恢复原标志。
  发送队列支持并发入队；每轮最多写 64 KiB，等待管道可写时释放队列锁。空闲等待最多 5 ms。
  Controller 速度回调直接入队，事件序号/订阅与入队串行化；速度仍只保留最新待发值。
- 队列最多 20 MiB 编码文本，可靠消息最多 256 项（包括正在发送的消息）；连续输出有六种替换键（含 cmd_vel）。
  该上限不包括构造响应时的临时字符串、内核管道或地图本身，不是 RSS 上限。
- 只替换尚未写出的同类数据；已写出部分的消息保持完整。输入顺序、scan 对时、地图生成不受替换影响。
- 响应、开始/终态/保存等事件不能被遥测覆盖。可靠队列满则连接失败、停机并以非零码退出，stderr 说明原因。
  不能在连接失效后承诺剩余消息送达，也不将这些消息伪装成已确认。
- 有待发数据且连续 2 秒无写进展判定慢读失败；即使输入空闲也会检查。
  输出管道读端关闭会被 poll/write 检出，不需要等下一条输入。
- QUIT 或 stdin EOF 先关闭组件，允许剩余消息最多排空 2 秒。正常 QUIT 有 BYE；EOF 在运行时有 RUNTIME IDLE（活跃任务另发 stopped），但无 BYE。
  不隐式 FINISH/SAVE。截断的末行不执行，报告 ERROR 后关闭。失败路径解除回调并清理组件。
- 发送超时/断连检查由发送线程独立执行；文件写入仍可能阻塞命令调用线程，
  文件操作和大地图编码尚非异步；地图计算已独立执行且可协作取消，见 [执行边界](architecture.md)。这不代表硬实时支持。
- IPC_STATUS 提供编码队列的 bytes/peak_bytes/pending/replaced，用于本机验收；不属于 SDK 算法状态。

## 导航扩展

所有命令继续使用 `REQ id ...`；速度输出需 `SUBSCRIBE ON`。stdio v4 为兼容扩展，不改原建图命令。

| 请求 / 事件 | 格式 |
|---|---|
| NAVIGATE | `session goal_x goal_y goal_yaw`；目标固定为 map 坐标；返回 ACCEPTED NAVIGATE |
| NAV_CANCEL | 无参数；接受取消请求，返回 OK NAV_CANCEL，等待 NAV canceled 确认 Server 退出 |
| NAV_STATUS | 返回 `NAV phase session generation reason` |
| GET_PATH | 返回 `PATH session generation map_revision map count x y yaw ...` |
| EVENT NAV | `phase session generation reason`；可靠任务事件，reason 无内容时为 `-` |
| EVENT PATH | `session generation map_revision count`；路径通知，完整路径通过 GET_PATH 获取 |
| EVENT CMD_VEL | `session generation sequence pose_stamp_us linear angular`；保留最新未发值，包括零速度 |

速度 sequence 区分同一采集位姿上的多次输出；pose_stamp_us 是传感器时钟，不是到期时间。
这是一条供观察的演示输出，没有设备确认，不是带租约/硬件急停的安全执行协议。

建图命令语义不变；速度无实际设备消费者，不构成设备停止确认。
Controller 通过线程安全回调直接入队，stdio 独立发送，不依赖主线程 Poll。
RESOURCES 的 localization_retained/active/results 和 pipeline_pending 覆盖异步定位；
等待全链路完成应看 pipeline_pending，compute_retained 仅表示建图阶段。
导航的地图与任务规则见 [导航 demo](navigation-demo.md)。

## 验证

~~~sh
cmake -S . -B build-product -DROBOFRAME_PRODUCT=demo_robot -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tools/mapping/replay.py --dataset tf --scans 0 --finish-after 12267 --output build-product/ipc-result
~~~

Python 有一个持续接收线程和有界响应队列，事件写入 JSONL，不在内存堆积所有历史事件。
回放使用 DATA 输入、带 ID 的业务请求，验证主动事件出现，并通过 GET_MAP 对比基准/实时地图与保存文件的全部字段和栅格。
stdio_publication_test 使用真实 4 KiB 管道，覆盖无查询发布、退订、快照/地图上下文、暂时慢读、持续不读、
可靠队列溢出、读端断连、EOF、QUIT 排空期限和非法/截断输入；mapping_check 验证进程内回调收到共享快照。
