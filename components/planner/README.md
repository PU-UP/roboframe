# planner
公开接口：include/planner/read_only_planner_component.hpp；实现：src/read_only_planner_component.cpp。
ReadOnlyPlannerComponent 验证只读共享地图和可选 scan，暂不生成路径。
独立工作线程最多持有一份当前输入与一份最新待处理输入。新输入替换旧待处理项；
当前输入不变，双次校验检测读取期间内容是否变化。读取延迟是资源验证参数，默认 0。
Activate/Deactivate/Submit 由 owner 串行调用；status/WaitIdle 可跨线程。
单元测试 tests/read_only_planner_component_test.cpp 可独立运行。

只依赖产品指定 SDK；支持独立构建、安装与 SOURCE/PACKAGE 接入。
源码标识和实际编译 SDK 自动生成。当前公共存储接口使用 SDK 3.0。
