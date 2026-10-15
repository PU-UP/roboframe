# odom

同步接收一条里程计位姿，校验后原样交给输出回调，并保留最新一份。
不积分、不滤波、不按时间重排。时间戳不要求递增。没有建图、产品或 Integration。

公开头：`include/odom/odom_component.hpp`，类型 `odom::OdomComponent`。
输入和输出都是 `roboframe::TimedPose2D`（`T_odom_base`，米、弧度、采集时间微秒）。
调用必须在同一个线程上串行完成。回调拿到的引用只在本次 `Push` 内有效，不得重入或销毁组件。
回调若抛出，该样本已经保留，并计入 `accepted()`。
`Deactivate` 丢掉保留位姿；`accepted()` 保留到下一次 `Activate`。

~~~cpp
roboframe::TimedPose2D received{};
odom::OdomComponent odometry([&](const roboframe::TimedPose2D& pose) { received = pose; });
odometry.Activate();
odometry.Push({10, {1.0, 2.0, 0.3}});
odometry.Deactivate();
~~~

组件库只依赖 `Roboframe::sdk`。日志解码和传输放在 `main.cpp`（可执行目标 `odom_receive`）：
它用 `StdioChannel` 从标准输入读行，只把 `ODOM` 记录转成 `TimedPose2D`。
`FLASER`、`PARAM` 和注释行忽略。不创建 Integration 或产品。

仓库内开发使用固定的 `build-dev`：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_PRODUCT= -DROBOFRAME_COMPONENT=odom
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

其中 `odom_replay` 测试会把 `data/intel-lab/intel.log.gz` 的原始 `ODOM` 流送进组件。
这只确认记录被接收，不是定位精度或实机里程计证明。
校正日志里的 ODOM 不参与这次检查。tests/replay_test.py 先校验 gzip，
再检查接收程序退出码及结果；odom_replay_corrupt_gzip 验证损坏 CRC 会被拒绝。
程序统计通过 StdioChannel 的 Queue/Drain 输出，不创建占位管道。

单独运行同一入口：

~~~sh
set -o pipefail
gzip -dc data/intel-lab/intel.log.gz | ./build-dev/components/odom/odom_receive
~~~

独立安装验证使用临时目录和已安装 SDK，不替换 `build-dev`：

~~~sh
cmake -S sdk -B /tmp/roboframe-odom-sdk -DCMAKE_BUILD_TYPE=Debug
cmake --build /tmp/roboframe-odom-sdk -j4
cmake --install /tmp/roboframe-odom-sdk --prefix /tmp/roboframe-odom-prefix
cmake -S components/odom -B /tmp/roboframe-odom -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=/tmp/roboframe-odom-prefix
cmake --build /tmp/roboframe-odom -j4
ctest --test-dir /tmp/roboframe-odom --output-on-failure
cmake --install /tmp/roboframe-odom --prefix /tmp/roboframe-odom-prefix
~~~

只交付组件库时设置 -DODOM_BUILD_RECEIVER=OFF，可在没有 interface/ 和 data/ 的目录构建。
启用独立入口时必须有 interface/；同时开启测试时还必须有数据集，缺失即报错。
组件库仍只链接 SDK，main.cpp 通过已有 roboframe_stdio_transport 目标复用传输。
