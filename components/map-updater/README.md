# MapUpdater

OccupancyMapComponent 消费 SDK Submap2DPtr 与 PoseNode2D，生成全局占据栅格。
不接原始传感器，不做定位，不决定何时开始建图；occupancy_raster 是私有射线/evidence 算法。
正常只积分子图新增扫描；UpdateNode 接受历史节点新版本后完整重建 atlas，移除旧位置的占据证据。
子图最多 4096，累计最多 4,000,000 个点（共享引用，不是点数组拷贝），每子图最多 64 帧。容量不足明确失败，不默默丢历史。
取消可使私有 evidence 未完成，复用前必须 Reset；已发布的不可变 GridSnapshot 不受影响。
整体重放不适合高频大规模优化，后续按测量结果引入局部重建。子图本身的形变不由整体重定位解决。
独立安装只公开 occupancy_map_component.hpp；测试保留私有 raster 算法验证和公开 atlas 契约验证。
