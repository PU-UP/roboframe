# roboframe

C++14 的机器人算法框架：interface → integration → component。
当前产品 **demo_robot** 用已有 odom、scan 和修正位姿验证持续数据流与可选建图任务；
SDK Server 统一单次/周期执行与取消，可选 BT 编排建图和导航任务；PlannerComponent 规划路径，ControllerComponent 独立计算未被设备消费的速度。
不包含真实 SLAM、Brain 或实机运动闭环。当前不使用 submodule。

## 从哪里读

1. [完整架构图与新产品接入](docs/architecture.md)：组件持有 Server、BT 边界、线程及开发分工。
2. [SDK 接口与模板](sdk/README.md)：生命周期、执行器、共享类型及独立交付。
3. [建图 demo](docs/demo_robot.md)、[导航 demo](docs/navigation-demo.md)：产品数据和运行方法。
4. [IPC 协议](docs/ipc.md)：命令、事件、独立发送及关闭规则。
5. [TODO](TODO.md)：当前边界与下一步；新 agent 先读 [AGENTS.md](AGENTS.md)。

~~~text
sdk/                         core 运行机制、datatype 公共语义、可编译 integration 模板
interface/                   通用 stdio 传输，不认识产品或组件
products/demo_robot/         示例产品：依赖选择、main、integration、任务与数据接线
  contract/                  独立公开的产品契约，不属于 SDK
  interface/                 stdio v4 协议适配，只依赖公开契约和传输
components/                  八个可独立构建的算法/地图/导航组件
data/intel-lab/               录制数据和来源说明
tests/                       产品、IPC、并发与独立交付验证
tools/mapping/               Python 回放与地图查看
tools/build-studio/           构建命令预览，不执行构建
~~~

## 构建与回放

~~~sh
cmake -S . -B build-product -DROBOFRAME_PRODUCT=demo_robot -DCMAKE_BUILD_TYPE=Release
cmake --build build-product -j4
ctest --test-dir build-product --output-on-failure
python3 tools/mapping/replay.py --dataset tf --scans 0 --finish-after 12267   --binary build-product/robot_runtime --output build-product/full-result
~~~

输出目录必须不存在，避免覆盖地图；打开结果中的 replay.html。
数据在 data/intel-lab，无需 history；build* 和 history 不代表当前源码。
旧构建缓存如果选择 robot，须显式改为 demo_robot 或使用新目录。
CONFIG 只配置，RUN 后持续计算；BEGIN/FINISH/CANCEL 只控制任务。详见demo_robot 示例产品。

## 独立开发与混合构建

组件 tests 下的具名 main 可直接调试算法，不必启动整个产品。

~~~sh
cmake -S sdk -B build-sdk
cmake --build build-sdk
cmake --install build-sdk --prefix "$PWD/build-packages"
cmake -S components/odom -B build-odom -DCMAKE_PREFIX_PATH="$PWD/build-packages"
cmake --build build-odom
ctest --test-dir build-odom --output-on-failure
cmake --install build-odom --prefix "$PWD/build-packages"
~~~

其他七组件同样安装后，可只使用头文件和库；也可只开发某个组件：

~~~sh
cmake -S . -B build-packaged -DROBOFRAME_USE_INSTALLED_COMPONENTS=ON   -DCMAKE_PREFIX_PATH="$PWD/build-packages"
cmake -S . -B build-mixed -DROBOFRAME_USE_INSTALLED_COMPONENTS=ON   -DROBOFRAME_MAP_UPDATER_SOURCE=SOURCE -DCMAKE_PREFIX_PATH="$PWD/build-packages"
~~~

SDK/组件各自选择 SOURCE 或 PACKAGE；缺失依赖明确报错。每个产品使用独立 build 目录。
BUILD_TESTING=OFF 不要求 Python。完整交付验证：python3 tests/build_isolation_test.py。

SDK 维护者维护 sdk/VERSION；产品 Dependencies.cmake 选择 SDK 与组件。
组件无需手写版本，构建自动记录源码指纹和实际编译 SDK；兼容规则见 SDK 文档。
当前 SDK 10.0 调整头文件布局并移出产品契约，旧组件源码和包需迁移重建；stdio v4 不变。
