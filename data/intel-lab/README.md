# Intel Lab：原始数据与规划参考地图

## 来源与用途
发布页：[StachnissLab / Pre-2014 Robotics 2D-Laser Datasets](https://www.ipb.uni-bonn.de/datasets/index.html)。
Intel Research Lab 原始日志由 **Dirk Haehnel** 提供，使用时应致谢。发布页分别提供 raw-log、corrected-log 和 RBPF-log。
这里不额外授予数据许可；下载文件只保留本地，不提交 Git，源码记录来源和 SHA-256。

| 数据 | 本地位置 | 内容与用途 |
|---|---|---|
| raw-log | history/2026-10-09-full-prototype/data/intel-lab/intel.log.gz | 13,631 帧扫描与原始估计位姿；保留为链路、资源和未来 SLAM 输入验证 |
| corrected-log | data/intel-lab/intel.gfs.log.gz | 发布者校正后的 910 帧配对扫描/位姿；生成后续规划用的离线参考地图 |

corrected-log 不是本项目运行 SLAM 的结果，不是测量真值。910 个完整量程数组均在 raw-log 中找到唯一、完全相同的量测。
同一组 910 帧配原始位姿仍产生整体扭曲；配校正位姿则形成清楚的实验室轮廓。
所以本次改善来自位姿，而非缩短到前几百帧、改变激光角度或隐藏后续数据。
发布页没有提供误差界或具体生成参数，本项目不推断其精度等级。

## 下载与复现
旧服务器的 HTTPS 端口不可用；以下 HTTP 链接来自发布页。文件通过重定向下载，指纹用于锁定本次验收版本，
不是发布者签名；下载到 .part，校验后再放入正式位置。已存在文件时不必重新下载。

~~~sh
mkdir -p data/intel-lab
curl -fL --max-filesize 20000000 \
  'http://www2.informatik.uni-freiburg.de/~stachnis/datasets/datasets/intel-lab/intel.gfs.log.gz' \
  -o data/intel-lab/intel.gfs.log.gz.part
printf '%s  %s\n' \
  '152d4db32761a71ac4a557d2bd383d4ba054864c290b799df92096ba34b70dea' \
  'data/intel-lab/intel.gfs.log.gz.part' | sha256sum -c - && \
  mv data/intel-lab/intel.gfs.log.gz.part data/intel-lab/intel.gfs.log.gz

python3 tools/mapping/replay.py --dataset corrected --scans 0 --finish-after 909 \
  --output build-product/corrected-reference
python3 tests/intel_reference_check.py build-product/corrected-reference/live.rfgrid \
  --output build-product/corrected-reference/connectivity.json
~~~
输出目录/JSON 文件不能覆盖；重跑请换新名称。原始模式保留：`--dataset raw`（默认）。
默认最多回放 3000 帧；corrected 默认使用全部 910 帧，显式 `--scans 0` 始终表示全部。
`--data` 仅改变路径，不取消所选数据版本的校验。

## 正确解析
- 只使用同一 FLASER 中的 180 个量程和配套位姿；不要按帧号或浮点时间戳把 raw/corrected 两文件硬拼。
- corrected 中两组位姿字段相等，均视作发布者参考坐标系下的已校正位姿；不能因字段叫 odom_x 就当成原始轮速里程计。
- corrected 的独立 ODOM 流不参与建图，其中发现 22 条不满足微秒时间约束的记录；不对无关记录伪造时间修复。
- 910 帧 FLASER 时间戳唯一，有 4 次倒序，按它自己的时间戳排序，并以首帧为零点；不借用原始日志时间轴。
- x/y/range 使用米，yaw 使用弧度。激光/base 位姿字段相等是当前适配前提；非零外参仍明确拒绝。
- 180 束采用 [GMapping 旧 CARMEN 读取代码](https://github.com/ros-perception/openslam_gmapping/blob/master/log/carmenconfiguration.cpp) 中的
  `-90..-1,+1..+90` 度约定；此次查证与现有代码一致，未因外观而换角度模型。
- 有效返回为 `0.01 <= r < 20 m`；远读数丢弃，不压成 20 米障碍，不伪造无返回清空射线。

## 接入边界与规划使用
继续复用 Python → IPC → interface → integration → odom透传 / map-updater → map-server / 只读planner。
只改变离线适配器的数据来源，没有增加 SLAM、导航或 BT 实现。

当前框架只有一个隐式工作坐标系，IPC ODOM 是已有位姿输入槽；corrected 回放将这个槽整体用作离线参考坐标系。
因此 summary.json 标为 `corrected_reference`，不能把该位姿当成真实设备连续输出的本地 odom。
**RFGRID1 本身尚未存储 frame 名称，后续加载地图时必须与 summary.json/本说明一起确认来源。**
正式 navigation 接入需要明确地图坐标、定位输出与目标点的坐标关系；不能直接把 raw odom 的位置用于 corrected 地图。

本次输出：
- `final.rfgrid`：前 909 帧固定结果；`live.rfgrid`：包含全部 910 帧，建议后续规划测试读取这一份。
- `live.png`：原始占据图，686 × 689、0.1 m/格、原点 (-31, -44) 米；PNG 顶部为 +y，存储栅格 y 向上。
- 栅格字节为 occupied=0、free=254、unknown=205；不是 ROS OccupancyGrid 的 -1/0/100。
- 尚未膨胀。planner 应使用自己的代价/膨胀层，不能修改共享原始地图；未知区域默认不可通行。
- `connectivity.json` 是离线验收：假设机器人半宽 0.2 m，保守保留 3 格间隔，未知与占据均阻挡。
  四邻接连通性检查找到 20,039 个可达格和 46.5 m 的示例格路径，不是运行时 planner 的输出或实机安全保证。
  同坐标系示例起点 (0.65, -0.05)、终点 (-1.75, -20.35)，均取格中心。

建议下一步固定这份地图，先实现 planner 的请求/取消/结果，再由 navigation 编排；
定位可先明确使用录制参考位姿输入。BT 在这些可单独测试的能力之上调度，之后再接在线 SLAM/重定位。
