# map-server
公开接口：include/map_server/file_map_component.hpp，类型：map_server::FileMapComponent。
实现：src/file_map_component.cpp；独立 main：tests/file_map_component_test.cpp。

保存/加载只读地图，不决定何时冻结。Linux 文件实现，校验名称、尺寸和校验和；同目录临时文件写入后发布，拒绝覆盖。没有目录 fsync，不承诺断电持久性。

只依赖 Roboframe::sdk；公开类型与契约说明在同一头文件。

不维护组件版本或契约版本。独立构建时用 CMAKE_PREFIX_PATH 指向产品指定的 SDK 安装包。
CMake 自动计算源码指纹、记录编译 SDK；安装包 VERSION 是生成记录，不手工编辑。
当前 SDK 3.0 改变 scan/地图公开接口，已有组件包需要重新构建。
