# map-updater
公开接口：include/map_updater/occupancy_map_component.hpp，类型：map_updater::OccupancyMapComponent。
实现：src/occupancy_map_component.cpp；独立 main：tests/occupancy_map_component_test.cpp。

接收 scan 与 ScanTransforms2D（本帧 T_odom_base + 已生效 T_map_odom），组合投影到配置的坐标系更新占据图。每帧每格只更新一次，占据优先。Reset/Snapshot 不决定建图业务阶段。边界外射线跳过，不做扫描匹配、回环或膨胀。

只依赖 产品指定的 Roboframe::sdk。输入 Scan2D/ScanTransforms2D 与地图 GridSnapshot 由 SDK 定义；
组件专有 Update 结果、操作与输入策略集中在具名组件头文件。
当前组件最多接收 2048 点、距离 0.01 到 20.001 米的命中点；这些不是 SDK 通用限制。

不维护组件版本或契约版本。独立构建时用 CMAKE_PREFIX_PATH 指向产品指定的 SDK 安装包。
CMake 自动计算源码指纹、记录编译 SDK；安装包 VERSION 是生成记录，不手工编辑。
当前 SDK 4.0 增加地图坐标字段与完整 TF 输入，已有组件包需要重新构建。

Snapshot 共享 GridCells 的未修改块；同一版本复用同一个快照。后续更新按需复制受影响块，旧快照保持不变。
