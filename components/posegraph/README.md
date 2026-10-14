# PoseGraph

PoseGraphComponent 接收 odom 与同一时刻的修正 map pose，维护当前 T_map_odom、高频位姿传播和有版本节点。
这是录制修正传播/节点存储实现，不包含图优化、真实传感器融合或 global 坐标层。
Resolve 不拿未来/过期修正；ReviseNode 只修改明确的历史节点，不改变当前定位。
节点最多 32768 个，达到上限明确拒收，Reset 清理；接收线程中的调用同步并由 integration 短锁保护。
独立测试 main 在 tests/pose_graph_component_test.cpp。只依赖 SDK，可独立编译安装。
