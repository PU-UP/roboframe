# 执行边界

interface 解码 SDK 输入并调用已绑定入口；输入接收成功不表示计算完成。main 负责产品装配、Poll 与退出策略，interface 不创建 integration。

- 传感器入口允许并发，SensorFlow 的短锁只覆盖有界缓存、同步和快速路由。禁止在锁内等待算法。
- 工作流指令、Poll、对外回调由同一业务 owner 有序调用；不能直接并发调用整个 integration。
- 耗时算法按需使用 SDK ExecutionChannel。一个通道是串行执行组，不强制每个组件独占线程。
- LocalMapBuilder 与 MapUpdater 当前共用地图计算组；Odom、录制 SLAM、PoseGraph 属于轻量输入通路。真实后端耗时后再独立执行。
- 当前地图计算容量为 8，包含排队、运行和已完成未领取结果。满时拒绝新输入，统计丢弃；传感器同步缓存另限 64 scan / 512 odom。
- Reset 改变执行 generation，清空待执行输入并协作取消旧计算。旧完成结果仍由 Poll 领取并丢弃，不得污染新任务。
- FINISH 停止当前任务输入并等待已接纳结果；完成后冻结基准快照。本产品仍允许实时更新，用于验证旧快照稳定，并非通用业务规则。
- CANCEL/Shutdown 关闭输入、使旧计算失效；Shutdown 等待组件安全退出。算法必须协作检查取消，不能承诺强杀任意阻塞库。
- stdio 非阻塞输出，遥测合并为最新值，终态/响应使用有界可靠队列；可靠队列耗尽时连接失败，不能静默丢终态。

地图保存、加载和完整地图编码目前仍在业务 owner 执行。接入周期控制之前必须隔离这些耗时操作。输入驱动通道不等于实时调度器。

## 验证

运行 `ctest --test-dir build-product --output-on-failure`。
execution_test 验证容量、取消、异常和 latest_pending；mapping_execution_test / IPC 测试验证慢计算时持续接收、会话隔离和关闭；
concurrent_ingress_test 验证并发传感器与任务切换；stdio_publication_test 验证慢读、断连、EOF/QUIT。
tests/support/SharedMapReader 仅用于共享快照并发读取验证，不是产品能力，也不参与运行时。
