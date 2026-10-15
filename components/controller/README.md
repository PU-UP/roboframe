# Controller

ControllerComponent 自身实现路径跟踪并持有 SDK Server 执行器，默认 50 ms 周期 / TF 接收超时 500 ms。
UpdateTransforms、UpdateMap 是线程安全输入端口；控制计算不需要 integration Poll。
基于路径与 TF 逐点转向/前进，提供 body 速度反馈；IPC 发布与设备执行不属于这个算法组件。
取消清除可见反馈；控制线程在结束、取消、TF 失效或异常时输出零速度，清理完成后才报告终态。
构造时可注入 Output 回调，由控制线程直接调用；回调必须有界且不可执行阻塞 I/O。

独立组件只依赖 SDK，不依赖 BT、产品 contract 或其他组件。新 Path2D/Twist2D 头需当前源码构建的 SDK。
组件持有 SDK Server 执行器，组件 Activate/Deactivate 管理生命周期；server() 提供 Submit、RequestCancel、GetStatus、TakeResult。
SDK Server 不持有组件，不包含产品业务。BT 访问执行器，组件不感知 BT。
算法方法和状态直接放在组件内部，只有一套生命周期，没有嵌套算法 Component。
公开头为 controller/controller_component.hpp；算法与执行测试均通过 server() 使用同一个组件。
此次公开头重命名不保留兼容别名，调用方和安装包须配套重建。

```sh
cmake -S components/controller -B build-controller -DCMAKE_PREFIX_PATH=/path/to/sdk-install
cmake --build build-controller
ctest --test-dir build-controller --output-on-failure
cmake --install build-controller --prefix /path/to/components-install
```

完整接口、演示参数、资源限制和已知边界见 [导航 demo](../../docs/navigation-demo.md)。
它是简单可验证的算法实现，不作为实机闭环或安全保证。
