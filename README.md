# roboframe

C++14 机器人算法框架骨架。main 不包含具体算法组件或产品实现；
完整建图/导航示例保存在 demo 分支。

~~~text
sdk/                 可选运行机制、公共数据、BT 辅助及可编译模板
interface/           通用 POSIX stdio 实现、独立发送线程与测试（未删除）
cmake/               产品的 SOURCE/PACKAGE 组件选择
components/          留给独立算法组件
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
历史 demo 使用 build-demo，旧 build-product/build-server 的产物不代表 main。
日常开发统一使用下面的 build-dev，编辑器也固定读取该目录。
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

## 开始开发组件

组件开发可使用 [SDK component skill](sdk/skills/roboframe-component/SKILL.md)。
它引用现有接口和模板，包含不依赖 Integration 的测试路径；不替开发者决定算法语义。
例如：“请读取并使用该 skill，按以下输入、输出和行为要求开发我的组件……”。

## 从根目录选择组件

日常开发固定用 build-dev；以下 name 替换为已创建的 components/<name> 目录名：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_PRODUCT= -DROBOFRAME_COMPONENT=name
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

该模式加载 SDK 和指定源码组件及其测试，无需产品；不会加载其他组件或通用 interface。
组件自己的 CMakeLists.txt 无需另写适配。SDK 可改用 PACKAGE 并设置 CMAKE_PREFIX_PATH；
组件选择入口始终读取源码，产品的 SOURCE/PACKAGE 组件选择机制保持不变。
ROBOFRAME_COMPONENT 与 ROBOFRAME_PRODUCT 互斥，名称非法或缺少组件 CMakeLists.txt 时直接报错。

切换组件只需在同一个 build-dev 重新配置 ROBOFRAME_COMPONENT。
切换产品时设置 -DROBOFRAME_COMPONENT= -DROBOFRAME_PRODUCT=<name>；
回到默认骨架时将两项都设为空。构建目录中的旧文件可能保留，以当前生成的目标和测试为准。
更换编译器、生成器或验证历史 demo 时才另用构建目录，不与日常开发缓存混用。

## 编辑器索引

用 VS Code 的 WSL 窗口打开此仓库。将本地 .clangd 的编译数据库固定为：

~~~yaml
CompileFlags:
  CompilationDatabase: build-dev
~~~

VS Code 的 clangd.arguments 不再另设 --compile-commands-dir，避免覆盖 .clangd。
重新配置后 build-dev/compile_commands.json 会更新；跳转仍旧时重启 clangd language server。
生成的 <namespace>/version.hpp 由数据库提供路径，不要手工追加 includePath。
.clangd 与 .vscode/settings.json 的个人修改留在本地；.vscode/extensions.json 保留插件推荐。
忽略规则仅作用于未跟踪文件；历史中已跟踪的编辑器配置仍须排除在提交之外。
独立安装验证可以使用临时目录，不改变日常索引目录，也不需要额外的切换脚本或符号链接。

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
