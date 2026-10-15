# Planner

PlannerComponent 自身实现 A* 和规划状态，持有 SDK Server 执行器，在独立线程执行单次规划。

四邻域 A* 地图规划。接收 map 快照、TF 和目标，返回不可变 Path2D 或不可达/取消。未知格阻塞，保守半径余量，不修改输入地图。

独立组件只依赖 SDK，不依赖 BT、产品 contract 或其他组件。新 Path2D/Twist2D 头需当前源码构建的 SDK。
组件持有 SDK Server 执行器，组件 Activate/Deactivate 管理生命周期；server() 提供 Submit、RequestCancel、GetStatus、TakeResult。
SDK Server 不持有组件，不包含产品业务。BT 访问执行器，组件不感知 BT。
算法方法和状态直接放在组件内部，只有一套生命周期，没有嵌套算法 Component。
公开头为 planner/planner_component.hpp；算法与执行测试均通过 server() 使用同一个组件。
此次公开头重命名不保留兼容别名，调用方和安装包须配套重建。

```sh
cmake -S components/planner -B build-planner -DCMAKE_PREFIX_PATH=/path/to/sdk-install
cmake --build build-planner
ctest --test-dir build-planner --output-on-failure
cmake --install build-planner --prefix /path/to/components-install
```

完整接口、演示参数、资源限制和已知边界见 [导航 demo](../../docs/navigation-demo.md)。
它是简单可验证的算法实现，不作为实机闭环或安全保证。
