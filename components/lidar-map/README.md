# lidar-map
`lidar_map::RecordedLidarMapComponent` 是当前录制校正实现，接口/实现分别为
include/lidar_map/recorded_lidar_map_component.hpp、src/recorded_lidar_map_component.cpp。

只依赖 SDK 4.0；可独立 CMake 构建、安装或用自己的 main 调用。输入 scan + 同时刻 T_odom_base，
配套 MapPose2D 由录制适配器提前注入；仅配套帧输出 T_map_odom，其他帧不伪造新校正。
输出同步回调，不持有大 scan，不生成导航地图；没有在线 SLAM 算法。
PushReferencePose 有序且最多缓存 64 个；过晚、重复或满队列输入拒绝。
Reset 清除参考和计数；Deactivate 清理状态。所有调用串行，回调不得重入/销毁组件。

独立测试 main 位于 tests/recorded_lidar_map_component_test.cpp，涵盖旋转/平移组合、稀疏输出和生命周期。
源码/包选择使用 ROBOFRAME_LIDAR_MAP_SOURCE，包名 LidarMap，目标 LidarMap::component。
源码身份和编译 SDK 自动生成，无组件手工 VERSION。完整接线见 ../../docs/transforms.md。
