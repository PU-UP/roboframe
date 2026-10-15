# roboframe

C++14 机器人算法框架骨架。main 不包含算法组件或产品实现；
完整建图/导航示例保存在 demo 分支。

~~~text
sdk/                 可选运行机制、公共数据、BT 辅助及可编译模板
interface/           通用 POSIX stdio 实现、独立发送线程与测试（未删除）
cmake/               产品的 SOURCE/PACKAGE 组件选择
components/          空，留给新的算法组件
products/            空，留给新的产品接线、契约和 BT
tests/               SDK 安装与产品/组件构建隔离检查
data/                保留原始录制数据，不参与默认构建
~~~

先读 [架构](docs/architecture.md)、[SDK 审查](docs/sdk-review.md)，
再看 [可运行模板](sdk/templates/integration/README.md)。
当前机制及限制见 [SDK](sdk/README.md) 和 [TODO](TODO.md)。
通用传输的实现和独立构建方式见 [interface](interface/README.md)；
旧 demo 的产品协议适配器保留在 demo 分支。

## 构建并亲自测试

已验证 WSL Ubuntu 22.04 / x86_64，需 CMake >= 3.16、C++14 编译器、Python3（测试使用）。
PowerShell 先执行 wsl -d Ubuntu-22.04，再执行：

~~~sh
cd /home/watermango/work/roboframe
cmake -S . -B build-skeleton -DCMAKE_BUILD_TYPE=Debug
cmake --build build-skeleton -j4
ctest --test-dir build-skeleton --output-on-failure
python3 tests/build_isolation_test.py
~~~

默认编译 SDK 测试、两种模板和通用传输；没有 robot_runtime。
切分支后使用不同构建目录，旧 build-product/build-server 的产物不代表 main。
编辑器使用 build-skeleton/compile_commands.json。
BUILD_TESTING=OFF 不要求 Python；POSIX 传输仅在 UNIX 构建。

完整 demo：

~~~sh
git switch demo
cmake -S . -B build-demo -DROBOFRAME_PRODUCT=demo_robot -DCMAKE_BUILD_TYPE=Debug
cmake --build build-demo -j4
ctest --test-dir build-demo --output-on-failure
git switch main
~~~

切换前先提交或保存你自己的改动。

## 开始一个真实产品

1. 先在 components/<name> 写一个组件及独立测试。只依赖 SDK，
   算法和生命周期在组件内；同步、ExecutionChannel、Server 或已有算法线程按需选择。
2. 在 products/<name> 写公开 contract、integration 与 main。开始时一个接线文件即可；
   不按 demo 拆出大量任务/数据流文件。需要任务顺序时再引入 BT。
3. 创建产品 Dependencies.cmake，明确 SDK 版本与组件清单：

~~~cmake
set(ROBOFRAME_REQUIRED_SDK_VERSION "10.0.0")
function(roboframe_load_components)
    roboframe_component(my-component MyComponent)
endfunction()
~~~

产品 CMakeLists.txt 定义可执行文件并链接 MyComponent::component；
然后配置 -DROBOFRAME_PRODUCT=<name>。cmake/RoboframeComponents.cmake
负责 SOURCE/PACKAGE 选择，不需要产品自己重写查找逻辑。
完整组件 CMake 包示例可在 demo 分支查看，不必复制其算法。

## SDK 独立交付与生成头文件

~~~sh
cmake -S sdk -B build-sdk -DCMAKE_BUILD_TYPE=Debug
cmake --build build-sdk -j4
ctest --test-dir build-sdk --output-on-failure
cmake --install build-sdk --prefix "$PWD/build-packages"
~~~

组件 CMake 调用 roboframe_component_metadata(target namespace) 后，
CMake 在配置阶段生成 build/.../generated/<namespace>/version.hpp，并自动添加头文件路径；
安装时也会包含它。源码目录没有这个文件是正常的，不要手工创建。
SDK 维护者管理 sdk/VERSION，组件来源指纹自动计算。
