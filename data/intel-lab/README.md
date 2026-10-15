> main 仅保留这份原始数据；本页的组件、工具和回放命令请在 demo 分支使用。

# Intel Lab：原始数据与规划参考地图

## 来源与用途
发布页：[StachnissLab / Pre-2014 Robotics 2D-Laser Datasets](https://www.ipb.uni-bonn.de/datasets/index.html)。
Intel Research Lab 原始日志由 **Dirk Haehnel** 提供，使用时应致谢。发布页分别提供 raw-log、corrected-log 和 RBPF-log。
两份压缩日志随仓库提交，作为 example 数据；保留原作者归属、来源和 SHA-256，不额外授予数据许可。

| 数据 | 本地位置 | 内容与用途 |
|---|---|---|
| raw-log | data/intel-lab/intel.log.gz | 13,631 帧扫描与原始估计位姿；保留为链路、资源和未来 SLAM 输入验证 |
| corrected-log | data/intel-lab/intel.gfs.log.gz | 发布者校正后的 910 帧配对扫描/位姿；生成后续规划用的离线参考地图 |

corrected-log 不是本项目运行 SLAM 的结果，不是测量真值。910 个完整量程数组均在 raw-log 中找到唯一、完全相同的量测。
同一组 910 帧配原始位姿仍产生整体扭曲；配校正位姿则形成清楚的实验室轮廓。
所以本次改善来自位姿，而非缩短到前几百帧、改变激光角度或隐藏后续数据。
发布页没有提供误差界或具体生成参数，本项目不推断其精度等级。

## 数据校验

两份数据已随仓库提供，无需下载。从仓库根目录执行：

~~~sh
(cd data/intel-lab && sha256sum -c SHA256SUMS)
~~~

数据解析和建图/导航回放工具保留在 demo 分支。
需要复现旧结果时先切换到 demo，按该分支 README 与 docs 中的说明运行；
不要使用旧构建目录来判断 main 的代码行为。

## 正确解析
- 只使用同一 FLASER 中的 180 个量程和配套位姿；不要按帧号或浮点时间戳把 raw/corrected 两文件硬拼。
- corrected 中两组位姿字段相等，均视作发布者参考坐标系下的已校正位姿；不能因字段叫 odom_x 就当成原始轮速里程计。
- corrected 的独立 ODOM 流不参与建图，其中发现 22 条不满足微秒时间约束的记录；不对无关记录伪造时间修复。
- corrected 的 910 帧 FLASER 时间戳唯一，有 4 次倒序。接入 TF 时以完整量程唯一匹配到 raw 帧，使用 raw 采集时间；原 corrected 时间保留用于追溯，不能直接按两个日志的时间戳配对。
- x/y/range 使用米，yaw 使用弧度。激光/base 位姿字段相等是当前适配前提；非零外参仍明确拒绝。
- 180 束采用 [GMapping 旧 CARMEN 读取代码](https://github.com/ros-perception/openslam_gmapping/blob/master/log/carmenconfiguration.cpp) 中的
  `-90..-1,+1..+90` 度约定；此次查证与现有代码一致，未因外观而换角度模型。
- 有效返回为 `0.01 <= r < 20 m`；远读数丢弃，不压成 20 米障碍，不伪造无返回清空射线。

## 新产品使用

当前 main 不包含 SLAM、建图或导航组件。录制修正轨迹只是输入数据，
不能作为新产品在线定位精度或实机控制闭环的证明。
先选择真实输入语义、时间匹配和坐标系，再独立实现自己的组件。
