# 建图验证入口

当前链路：Python 数据适配 → stdio IPC → SDK 输入 → integration 数据流 → odom / 录制 lidar-map → map-updater → MapServer 与 interface 发布。
构建方式见根 README；共享内存与执行边界分别见 data-sharing.md、execution.md。

```sh
cmake -S . -B build-product -DCMAKE_BUILD_TYPE=Debug
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tools/mapping/replay.py --dataset tf --scans 0 --binary build-product/robot_runtime --output build-product/main-verified
```

协议 REQ <id> <command> 返回 RES；DATA 传感器输入不逐帧确认；EVENT 为主动发布。
CONFIG 配置地图及时间同步；BEGIN 打开会话；ODOM / MAP_POSE / SCAN 输入数据；
FINISH 只确认请求接纳，mapping_finished 事件表示排空成功；SAVE 保存冻结地图；
SNAPSHOT 导出当前地图；GET_MAP 拉取共享快照的 IPC 编码；LOAD 加载；CANCEL 取消；QUIT 退出。
STATUS、TF、RESOURCES 查询任务、坐标和资源。详细字段由 interface/stdio_interface.cpp 与 Python Host 对照维护。

FINISH 后继续更新只用于本产品验证冻结快照。EOF 不自动保存；LOAD 不等于完成定位；保存文件目前没有承诺断电持久化。
回放会生成 replay.html、地图快照、轨迹和资源记录。raw 是原始里程计，corrected 是匹配的修正关键帧，tf 是全量原始数据加稀疏修正。

只读 Planner 已移出产品，测试中的 SharedMapReader 保留并发读取验证。后续先贯通子图/PoseGraph 链路，不增加导航空壳。
