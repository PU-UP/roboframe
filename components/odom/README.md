# odom
公开接口：include/odom/passthrough_odom_component.hpp，类型：odom::PassthroughOdomComponent。
实现：src/passthrough_odom_component.cpp；独立 main：tests/passthrough_odom_component_test.cpp。

接收 TimedPose2D，校验后同步原样输出。没有建图业务、缓存查询或轨迹估计。回调异常时样本已消费，不能用同一时间戳重试。

只依赖 Roboframe::sdk；公开类型与契约说明在同一头文件。

不维护组件版本或契约版本。独立构建时用 CMAKE_PREFIX_PATH 指向产品指定的 SDK 安装包。
CMake 自动计算源码指纹、记录编译 SDK；安装包 VERSION 是生成记录，不手工编辑。
当前 SDK 4.0 改变 TF/地图坐标公开接口，已有组件包需要重新构建。
