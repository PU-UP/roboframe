# map-updater
公开接口：include/map_updater/occupancy_map_component.hpp，类型：map_updater::OccupancyMapComponent。
实现：src/occupancy_map_component.cpp；独立 main：tests/occupancy_map_component_test.cpp。

接收同一时刻的 scan 与位姿，以 T_odom_base 更新占据地图。每帧每格只更新一次，占据优先。Reset/Snapshot 不决定建图业务阶段。边界外射线跳过，不做扫描匹配、回环或膨胀。

只依赖 Roboframe::sdk；公开类型与契约说明在同一头文件。
VERSION 由维护者发布时调整。支持独立构建、测试、安装；CMAKE_PREFIX_PATH 指向 SDK 安装前缀。
