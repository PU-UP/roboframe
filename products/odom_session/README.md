# odom_session

用现有 `odom` 组成的单会话产品。组件只校验并原样交出 `TimedPose2D`；
会话开始、停止、重新开始和结果解释在 integration。不使用 BT：odom 没有目标端口，
这条链路也不是多步任务，`ServerAction` 无法适配它。

## 组件与入口

- 组件：`odom`（`Odom::component`，`odom::OdomComponent`）。不改其算法语义。
- 可执行文件：`odom_session`
- 公开契约：`include/odom_session/session_integration.hpp` 的 `SessionView`、`SessionIntegration`
- SDK：`10.0.0`

## 数据流与线程

~~~text
stdin 行 → StdioChannel（输入 owner 为控制线程，写出在传输自己的线程）
        → main 忽略 FLASER / PARAM / 注释，把 ODOM 转成 TimedPose2D
        → SessionIntegration::Push
        → OdomComponent::Push（同一控制线程，同步校验后回调）
        → 会话视图：有效样本数、最新位姿、累计路程
        → 每次会话停止时 Queue 一行摘要，Drain 后退出
~~~

控制线程独占 `Start`、`Stop`、`Reset`、`Poll`、`Push` 和 `view`。
`Poll` 为空：odom 没有异步结果，主循环仍在 `NextLine` 的 service 里调用它。
回调复制已接受的位姿，并按输入顺序累加二维路程；不得重入或销毁组件。
odom 没有 `RequestDeactivate`；`CloseInputs` 关闭产品入口，`OnStop` 再 `Deactivate`。
`Deactivate` 会丢掉组件内保留位姿，产品在停用前复制到会话视图。

损坏的 gzip 由回放脚本在送入程序前拒绝。畸形 `ODOM`、非法位姿或未知 `SESSION` 命令
使进程向 stderr 写原因并以退出码 1 结束，不输出成功摘要。
非 `ODOM` 记录不计数。时间戳不要求递增。

## 会话

进程第一次收到 ODOM 时可自动开始会话；显式停止后，必须先发送 SESSION start 或 SESSION restart 才能继续输入。接受的命令为：

~~~text
SESSION start
SESSION stop
SESSION restart
~~~

`start` 在已有会话时失败。`stop` 关闭输入、停用 odom，并输出本会话摘要；之后的 `ODOM` 会失败。
`restart` 先结束当前会话并输出，再把计数、位姿和路程清零后开始新会话。
输入结束且会话仍打开时，同样停止并输出。空会话输出 `valid=0 distance=0.000000`。
累计路程是本会话内、按输入顺序相邻有效位姿的二维距离之和（米），不含 yaw，也不按时间戳重排。
第一个有效位姿的路程为 0。被 odom 拒绝的样本不参与端点。停止后不再累加；新会话清零，且不与上一会话的末位姿相连。

有样本时为一行：

~~~text
valid=<计数> latest_us=<微秒> x=<米> y=<米> yaw=<弧度> distance=<米>
~~~

## 构建与测试

在仓库根目录：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_COMPONENT= -DROBOFRAME_PRODUCT=odom_session
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

单独回放原始日志：

~~~sh
set -o pipefail
gzip -dc data/intel-lab/intel.log.gz | ./build-dev/products/odom_session/odom_session
~~~

已在 WSL Ubuntu 22.04 / x86_64 上跑通上述配置、构建和 `ctest`：19/19 通过。
正常回放结果为 `valid=26915 latest_us=976055548625127 x=-50.883999 y=-35.825001 yaw=2.538102 distance=509.139428`。
维护前后这一计数和位姿保持不变。损坏 gzip 以退出码 1 拒绝，并报告 `CRC check failed`。
首次自动开始、停止后数据拒绝（含空会话）、停止后显式 start/restart、运行中 restart 和 EOF 收尾由 `odom_session_integration_test` 和 `odom_session_stop_restart` 覆盖。
`intel.gfs.log.gz` 的校正 ODOM 不参与检查。回放只确认记录进入本会话，不是定位精度或实机证明。
未覆盖真实 IPC 以外的传输、ARM、Windows，以及 odom 内部全部单测以外的算法性质；
产品构建会连带运行已加载的 `odom_component_test`。

本次反馈：`.roboframe/feedback/20261015T181530-da72eb63-28b7-4a68-a936-48365193aae0.json`
