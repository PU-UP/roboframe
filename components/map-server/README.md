# map-server
公开接口：include/map_server/file_map_component.hpp，类型：map_server::FileMapComponent。
实现：src/file_map_component.cpp；独立 main：tests/file_map_component_test.cpp。

保存/加载只读地图，不决定何时冻结。Linux 文件实现，校验名称、尺寸和校验和；同目录临时文件写入后发布，拒绝覆盖。没有目录 fsync，不承诺断电持久性。

只依赖 Roboframe::sdk；公开类型与契约说明在同一头文件。
VERSION 由维护者发布时调整。支持独立构建、测试、安装；CMAKE_PREFIX_PATH 指向 SDK 安装前缀。

VERSION 是本组件的唯一版本维护入口：名称、实现版本、契约版本、最低 SDK。
当前源码需要 SDK 1.3 提供的元数据构建工具。公开契约常量由 CMake 生成，
安装包 share/roboframe/本组件名/VERSION 额外记录实际编译 SDK；不能手工改写这个构建事实。
