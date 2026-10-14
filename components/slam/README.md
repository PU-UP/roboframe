# SLAM：录制输出替身

RecordedSlamComponent 接收不可变 scan、同一采集时刻的 raw odom 和提前注入的 MapPose2D 参考。
只输出匹配的修正后 map pose，不计算 T_map_odom，不建导航地图。
参考缓存最多 64 项；无参考不输出，未来参考不得提前使用。调用同步且须串行，回调不得重入。
这是框架验证实现，不运行 SLAM。未来可替换为独立激光/视觉后端，数据集格式不进入公共契约。
只依赖 SDK；tests/recorded_slam_component_test.cpp 是可直接调试的 main，CMake 支持独立安装包。
