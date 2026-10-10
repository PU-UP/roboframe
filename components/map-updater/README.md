# map-updater
公开接口：include/map_updater/occupancy_map_component.hpp，类型：map_updater::OccupancyMapComponent。
实现：src/occupancy_map_component.cpp；独立 main：tests/occupancy_map_component_test.cpp。

接收同一时刻的 scan 与位姿，以 T_odom_base 更新占据地图。每帧每格只更新一次，占据优先。Reset/Snapshot 不决定建图业务阶段。边界外射线跳过，不做扫描匹配、回环或膨胀。

只依赖 Roboframe::sdk >=1.2。输入 Scan2D/TimedPose2D 与地图 GridSnapshot 由 SDK 定义；
组件专有 Update 结果、操作与输入策略集中在具名组件头文件。
当前组件最多接收 2048 点、距离 0.01 到 20.001 米的命中点；这些不是 SDK 通用限制。
入参类型迁移后组件版本与契约为 2.0，旧 MapUpdater 1.x 安装包不再满足本产品要求。
VERSION 由维护者发布时调整。支持独立构建、测试、安装；CMAKE_PREFIX_PATH 指向 SDK 安装前缀。
