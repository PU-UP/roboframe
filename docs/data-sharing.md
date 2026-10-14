# 大数据共享

传感器在 interface 解码一次，转为 SDK Scan2DPtr（shared_ptr<const Scan2D>）。多个实际消费者共享同一 payload；需要修改的算法建立自己的工作区。
小型位姿/变换按值传递，按采集时间关联，不用最新 pose 冒充采集时 pose。

GridSnapshot 持有不可变 Grid。GridCells 以块为单位写时复制；冻结基准地图、实时发布和保存可同时持有旧版本。复制量统计只含复制的数据块，不等于总内存。
快照不可变不意味着无限持有安全：执行队列、同步缓存以及地图历史必须分别设上限。

## 验收

- sdk/tests/grid_sharing_test.cpp：写入后旧快照不变、未修改块共享。
- tests/shared_map_reader_test.cpp 与 tests/support/：慢读与写入并行，同一 scan/map 地址、latest_pending 容量及引用释放。
- tests/mapping_check.cpp / mapping_ipc_check.py：FINISH 冻结、继续更新、保存与加载一致。
- tools/mapping/replay.py：真实数据回放、输出快照与资源统计。

SharedMapReader 已从原 planner 目录移入测试，当前产品没有 Planner 或导航空壳。WAIT_PLANNER 和产品 planner 统计已移除。
离线回放按小批次等待计算排空，便于确定性验收；这不是实际传感器墙钟频率下的吞吐保证。
