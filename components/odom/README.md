# odom
公开接口：include/odom/passthrough_odom_component.hpp，类型：odom::PassthroughOdomComponent。
实现：src/passthrough_odom_component.cpp；独立 main：tests/passthrough_odom_component_test.cpp。

接收 TimedPose2D，校验后同步原样输出。没有建图业务、缓存查询或轨迹估计。回调异常时样本已消费，不能用同一时间戳重试。

只依赖 Roboframe::sdk；公开类型与契约说明在同一头文件。
VERSION 由维护者发布时调整。支持独立构建、测试、安装；CMAKE_PREFIX_PATH 指向 SDK 安装前缀。

VERSION 是本组件的唯一版本维护入口：名称、实现版本、契约版本、最低 SDK。
当前源码需要 SDK 1.3 提供的元数据构建工具。公开契约常量由 CMake 生成，
安装包 share/roboframe/本组件名/VERSION 额外记录实际编译 SDK；不能手工改写这个构建事实。
