# 异步组件、Server 与 BT 验证记录

基于已提交的同步 odom_session，在 codex/async-server-bt-validation 分支完成。
保留原同步入口；新增 --mission 模式，两个小组件分别验证 ExecutionChannel 与 Server。
初次验证未改 SDK/skills；后续按用户授权修复 ServerAction 取消结果交付，并同步模板和 skill。
SDK 方法签名保持不变，Drained/Reset 收尾也会触发原有结果回调。个人编辑器配置未改。

## 结论

这条实际链路可以按现有 skills 接入：组件独立管理执行，integration 只接线/协调生命周期和 BT，
interface 独立发送。组件不需要知道其他组件、产品或 BT，也无需把执行器登记给 integration。
没有引入公共执行器包装、额外框架或测试专用算法参数。

异步数据转交和 BT 仍依赖控制线程 Poll；只有 Server 周期计算/输出与 IPC 发送独立。
主线程不 Poll 的真实管道测试验证了这一区分，而不是宣称全链路独立。

## Review 发现与处理

1. **取消不能吞掉收尾失败。** 原 ServerAction::Drained 领取并丢弃 Result。
   注入 finish 回调异常后，初稿把失败标为 canceled；产品曾临时查询终态保护。
   SDK 现通过原有结果回调一次性交付所有终态和 error，产品删除查询绕法并保留原始异常。
   SDK 新回归在旧实现上失败、修复后通过；模板和产品同时覆盖取消、收尾失败及停止重启。
2. **异常退出也需排空发送队列。** 初稿只有正常路径 Drain，异常仅靠析构。
   20 次试验没有观测丢失，但 StdioChannel 析构不保证排空，不能作为输出协议保证。
   当前异常路径先 Shutdown 再 Drain，并验证非法输入后最终输出 inactive。
   这是本次入口实现问题，不归因于 skill 缺陷。

反馈保留在仓库 .roboframe/feedback/；本轮 SDK 维护审查将上述两项标为已解决，
分别记录 SDK 修复和产品入口修复的依据；反馈不提交。

## 验证证据

| 检查 | 结果 |
|---|---|
| build-dev 构建及完整 CTest | 24/24，含 ServerAction 一次性交付、模板与产品回归 |
| 本轮：ServerAction、BT 模板、mission 各重复 20 次 | 60 次通过，无失败 |
| 初次验证：新增 4 项测试各重复 30 次 | 120 次通过，无失败 |
| 原始 intel.log.gz 任务模式 | 两目标成功；26915 样本、509.139428 米，与同步结果一致 |
| 组件独立构建/测试/安装 | odom、path_distance、distance_goal 通过，不需要产品 |
| 安装包搬迁后的外部消费者 | 新组件公开头、生成版本头及导出目标可用 |
| 产品纯包与两种混合来源 | 三种构建均运行产品测试通过 |
| 通用 tests/build_isolation_test.py | 全部通过 |
| 初次验证：AddressSanitizer + UndefinedBehaviorSanitizer | 新组件及 mission 三项测试通过 |
| ThreadSanitizer | 编译成功；WSL 启动失败 unexpected memory mapping，未完成竞态检查 |

具体回归覆盖满队列保留输入、已完成结果占用额度、异步异常、取消/清旧结果、重启清零、
BT 两目标顺序执行、超时失败、取消完成前拒绝复用、收尾异常不误报取消、
主线程不 Poll 时实际 IPC 持续输出，以及正常/异常关闭的最终 inactive 状态。
没有在生产组件内添加人为 sleep 或测试开关。

## 已知边界

这是数据回放和里程阈值任务，不是导航或实机控制器；inactive 不代表物理零速。
Server 为协作式取消与普通线程周期，没有硬实时保证。阻塞的第三方算法/回调不能被强杀。
目标阈值为激活后的绝对累计里程，每个目标默认超时 2 秒；超慢回放可通过公开 Begin 参数配置。
实际 ARM、Windows 原生、实机算法及线程检测器的完整检查未完成。
不把这些限制隐藏为已验证结论。

验收命令与线程图见 README.md；组件各自 README 给出直接使用和独立编译方式。
