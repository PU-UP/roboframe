# 大数据共享：scan 与持续更新地图
本轮沿用真实 Intel odom/scan，增加一个只读 planner 验证资源传递；没有加入路径规划算法。
同一算法进程内共享引用，IPC 解码仍然创建输入数据，不宣称端到端零拷贝。

## 所有权与接线
~~~text
IPC 解码 scan → shared_ptr<const Scan2D>
                         ├─→ map-updater（串行读取，更新私有 evidence）
                         └─→ planner（发布地图时收到同一个 scan）
map-updater → GridSnapshot ──→ integration 当前地图
                           ├─→ planner（独立线程）
                           └─→ map-server（按指令保存）
FINISH → 保留基准快照；后续 scan → 新的实时地图版本
~~~
Scan2D 禁止隐式复制，可以移动；跨模块复制的是共享指针。发布后所有生产者都不得通过可写别名修改内容。
planner 单元测试检查 scan 对象、点数组和地图对象的地址，确认收到的是同一份数据。
算法确需修改输入时应明确生成私有结果，不偷偷修改共享输入。

integration 仍负责配对 scan 与 odom，以及决定何时发布；地图内部更新和块复制由 component/SDK 存储处理。
按传感器时间间隔 100 ms 发布，即数据时间最多 10 Hz；FINISH、WAIT_PLANNER 可强制刷新尾帧。
离线回放按最快速度运行，不代表实际硬件的 10 Hz 墙钟吞吐性能。并非每帧 scan 都送给 planner，
每次送入 planner 的 scan 与对应地图的时间戳一致。

## 地图快照
Grid::cells 改为 GridCells：按行主序拆成 4096 字节块，不提供可写指针。
复制 Grid 只复制块引用表；Set 只在值变化且该块被其他快照引用时复制对应块。
算法的 evidence/observations 仍是私有连续数组；更新命中格时同步更新交付栅格。
Snapshot 在版本不变时复用同一个对象，不重新遍历整个 evidence 生成全图。

新旧快照共享未修改块；旧读者继续使用旧块。快照不会被后续写入改变。
没有共享的可变整图，也不需要让消费者持有地图写锁。
map-server 分块读写，RFGRID1 的磁盘布局与校验和保持不变，不额外拼接完整像素缓冲。

SDK 测试构造 8,000,000 格地图，保留 21 个逻辑版本，每次修改一个块：
仅额外复制 20 × 4096 字节，未修改块地址相同；旧版本释放后弱引用失效。
这是针对存储行为的测试，不是进程峰值内存测量。

## planner 与有界持有
ReadOnlyPlannerComponent 是独立可构建/安装的组件。Submit 快速提交只读引用，
一个工作线程读取所有地图字节和 scan 点，等待后再次读取，比较内容是否一致。
integration 配置 5 ms 读取间隔以模拟耗时消费者；正常构造默认不人为等待。
它只验证读取，不生成路径。读取异常或不一致计入 inconsistent。

最多持有一份正在读的输入和一份最新待处理输入。新输入替换旧待处理项，不打断当前读取。
每项包含一个 scan 引用和一个地图引用；等待锁仅涉及提交、取走及状态更新，不覆盖整图读取。
WaitIdle 返回时工作线程已释放输入引用；Deactivate 唤醒并加入线程，不遗留后台访问。
完成等待时 received = completed + replaced，pending/active 均为零。
替换数量会随机器调度变化；不能把它解释为建图丢帧。

scan/odom 配对队列仍限制 64 帧，每帧最多 2048 点；planner 的引用持有上限为两项。
integration 还持有最新 scan、基准地图和当前地图，地图组件持有工作状态与缓存快照。
本产品内持有数量有界，但任意外部消费者主动永久保存快照仍会增加内存，框架不能替它回收。
分块写时复制不消除全图更新、初始分配、算法私有缓存或引用表的成本。

## 指令与观察
- FINISH：结束本次基准建图，冻结基准；scan 继续更新实时地图。
- SAVE name：保存基准地图，前后结果必须一致。
- SNAPSHOT name：保存当前实时地图，FINISH 后仍允许。
- RESOURCES：读取发布、实时更新/丢弃、复制字节以及 planner 计数。
- WAIT_PLANNER：强制发布最新版本并等待读取完成，便于确定性验收。
- LOAD：读取已有地图并送 planner，但不从占据值反推 evidence；加载后不继续更新，需 BEGIN 开始新建图。
- CANCEL/QUIT：停止该建图更新或关闭进程，不隐式保存；组件析构负责线程清理。

publications/shared_scans/live_updates/live_dropped 是进程内累计计数；
copied_bytes 是当前 mapper 在最近一次 Reset 后因共享块写入而复制的字节。
map_bytes 为逻辑像素字节数，table_bytes 为当前快照引用表的逻辑大小；
它们不是 RSS，不包含分配器开销或所有组件内存。清零或切换会话时不要直接混用这些统计口径。

## 验收入口
~~~sh
cmake -S . -B build-product -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tests/build_isolation_test.py
python3 tools/mapping/replay.py --scans 0 --finish-after 12267 --output build-product/shared-data-result
~~~
输出目录须不存在。浏览 replay.html 可对比基准/实时地图并查看资源指标；
summary.json 和指令回执保留实际执行记录。新版本 SDK 为 3.0，所有组件开发包需重建。

一次全量验收：13,631 帧，基准建图 12,267 帧，之后更新 1,364 帧；
实时地图相对基准变化 37,315 个格，基准文件与改造前逐字节一致。
6,979 次发布，地图块复制约 382.70 MiB；
按每次发布都复制完整像素估算约 7149.21 MiB，不含引用表/初始化/IPC 成本。
planner 最多持有两项，读取不一致为零；具体完成/替换次数随调度改变。
这些结果不证明定位或地图几何质量，原始地图重叠问题继续暂停。

并发检查限制：当前 WSL 的 TSan 在进入测试前因 unexpected memory mapping 退出，
未取得 TSan 通过结论；ASan/UBSan 与异步共享行为测试通过。
