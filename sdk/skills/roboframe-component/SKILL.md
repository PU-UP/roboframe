---
name: roboframe-component
description: 根据开发者需求创建或修改 Roboframe C++14 组件，复用当前 SDK 接口和执行模板，完成不依赖 Integration 的组件测试及独立构建、安装验证。用于组件开发，不负责产品 BT、IPC 协议或 SDK 架构重构。
---

# Roboframe component 开发

把开发者的算法与端口需求落实成独立组件，不替开发者定义算法业务。
只使用当前工作区或选定安装包的 SDK；不从历史 demo 恢复实现，不新增第二套骨架。

## 保留开发反馈

先读取共用的 [反馈规范](../feedback-format.md)，创建本次运行记录。
遇到说明不清、缺少类型、能力受限或验证受阻时逐条记录背景、证据和临时处理；
开发 agent 只创建未读项，状态评价由 SDK 维护者填写。无问题也保留验证结果。
结束时给出记录路径；不要因为临时解决了问题而删除反馈。

## 找到真实接口

先读当前仓库的开发指引和组件已有代码（如有）。
本文件所在目录向上两级是 SDK 资源目录；其中 templates 在源码与安装包中布局一致。

- 源码 SDK：公开头在 SDK 资源目录的 include/roboframe；
  构建元信息函数在 cmake/RoboframeComponentMetadata.cmake。
- 安装 SDK：由 find_package(RoboframeSDK CONFIG REQUIRED) 加载；
  从导出的 Roboframe::sdk 的 INTERFACE_INCLUDE_DIRECTORIES 找公开头，
  从 RoboframeSDK_DIR 找构建元信息函数。不要求安装包带整个产品仓库。
- 先核对 core/component.hpp、core/lifecycle.hpp 及实际输入输出使用的 datatype 头。
  只有选用异步执行时才读 core/execution.hpp 或 core/server.hpp。
  不根据 skill 中的描述猜测不存在的方法；实际选中的头文件是接口依据。

按需求读取现有材料，不一律复制 integration 模板：

| 需求 | 参考 |
|---|---|
| 快速同步计算 | Component 基类和需要的数据类型即可，不添加执行器 |
| 有界异步数据流 | [StreamComponent](../../templates/integration/stream_component.hpp) |
| 单目标/周期执行及取消 | [WorkComponent](../../templates/integration/server_product.hpp) 中的组件部分；不复制 ServerProduct、BT 或任务 |
| 所有权与停止顺序 | [执行模板说明](../../templates/integration/README.md) |
| 不创建 Integration 的验证 | [直接组件测试](../../templates/integration/component_check.cpp) |

## 形成组件契约并实现

依据开发者需求和已有接口明确：输入输出、有效性、生命周期、允许的调用线程、
同步/异步输出、取消/错误和资源策略。只有会改变行为且无法从上下文确定的缺口才提问；
不要把时间戳必须递增、重复激活行为、丢帧策略等具体选择当作 SDK 的统一要求。

- 在开发者指定的组件目录实现算法与公开具名端口；继承 Component 并实现
  Activate、Deactivate、lifecycle。参数和专有数据类型放组件公开头。
- 组件只依赖 SDK、自身契约及实际算法库，不认识 integration、BT 或其他组件。
  不为一个组件再嵌套一个算法 Component，不新增万能消息总线或执行器包装基类。
- Server / ExecutionChannel 是组件可选持有的能力，不要求每组件一个线程。
  算法自带线程时封装其生命周期，不无条件再套 SDK worker。
- 组件拥有执行时，由组件完成接纳、取消、等待和旧结果清理；不要要求 integration
  用 UseServer/UseExecution 接管内部执行器。RequestDeactivate 是可选组件接口，
  不是 Component 基类要求。
- 析构前停止使用成员状态的后台计算和回调。不得在自身 worker 中等待/关闭同一执行器。
  执行器的取消是协作式，不承诺强杀；大数据发布后保持只读。
- 选用 Server 时依据实际头文件实现 start/step/finish；
  finish 必须容忍目标在 start 前被取消、部分初始化及异常。
  需要零速度等收尾输出时，由该组件的语义决定，不作为所有组件的统一规则。

新增组件通常只需要公开头、实现、tests、CMakeLists.txt、安装包配置及简短 README。
目录名和端口名遵循已有组件或开发者命名；不为单元测试创建产品或 Integration。
如需通过 IPC 单独运行，使用一个薄的入口程序/适配器，将 interface 接到组件的具名输入
和输出回调；组件算法库不依赖 IPC 协议或传输实现。仅在明确需要独立进程时添加入口。
README 说明端口、线程、生命周期与运行验证命令，避免复制 SDK 文档。

## 先独立验证组件

组件测试可有一个普通 main：构造组件 → 注册测试输出接收器 → Activate →
调用公开输入/目标接口 → 比较预期结果 → Deactivate。测试程序不是 Integration。

- 同步组件：直接调用接口，用局部容器或回调记录输出并断言。
- 异步组件：测试仍使用组件自己持有的执行器，按公开结果接口或线程安全回调等待；
  使用有截止时间的条件等待验证完成，不能靠长时间 sleep 猜完成。
- Server 组件：直接 Submit/GetStatus/TakeResult/RequestCancel，
  不需要 BT Tick 或 Integration::Poll。测试组件算法与清理行为，不重复整套 SDK 测试。
- 按组件契约覆盖正常结果、非法输入、边界、停用后的行为、错误恢复和停止重启。
  存在后台工作时再验证在途取消、迟到输出、关停完成及相关资源释放。
- 地图、TF、传感器数据可由测试构造，输出可由测试接收；无需启动其他真实组件。
  外部算法/设备确有依赖时，通过已有或必要的窄接口提供测试替身，不模拟整个产品。
- 预期结果来自算法契约、可手算输入或可信基准，不用被测实现生成自己的标准答案。
  需要数据集/设备才能证明的精度和实时性单独报告，不能把单元测试通过当作这些证明。

后续跨组件时间关联、产品 BT、IPC 端到端行为才属于 integration/系统测试，
不作为组件开发的前置条件。

## 独立构建与交付

复用当前 SDK 的 CMake 导出，不添加新生成器或复制整套工程：

1. C++14，独立项目启用 CTest。仅当 Roboframe::sdk 尚不存在时查找 SDK 包，
   组件 PUBLIC 链接该目标，区分 BUILD_INTERFACE 与 INSTALL_INTERFACE 头文件路径。
2. 库目标提供 PackageName::component 别名，EXPORT_NAME 设为 component；
   调用 roboframe_component_metadata(target namespace) 生成源码身份及 SDK 兼容信息。
   namespace/version.hpp 在配置阶段生成并自动加入 include 路径，不手写或提交生成文件。
3. 安装库、公开头和导出 targets；包配置查找 SDK 后包含导出的 targets。
   同时支持上层 add_subdirectory 源码构建和 find_package 安装包构建。
   源码仓库的 tests/build_isolation_test.py 中有临时组件的现成构建例子，可按需阅读；
   不将这个测试用例或它的模拟产品复制为组件实现。
4. tests 直接链接组件目标，用 add_test 登记。不链接产品或 interface，
   不要求 Integration 运行。根目录不会自动发现组件；必须显式选择并确认组件测试实际运行。

仓库内开发优先复用固定的 build-dev，选择当前组件，不创建产品：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_PRODUCT= -DROBOFRAME_COMPONENT=name
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

该入口编译 SDK 与指定源码组件；组件与产品选择互斥，切换模式时显式清空另一项。
SDK 也可通过 ROBOFRAME_SDK_SOURCE=PACKAGE 和 CMAKE_PREFIX_PATH 选择安装包。
日常切换组件只改 name；不为每次验证新建构建目录，不擅自改开发者的编辑器设置。
若已有不同工具链/生成器的活动目录，按仓库说明使用相应目录，不能混用缓存。
clangd 使用 build-dev/compile_commands.json；版本头由配置阶段生成，不手工补 include 路径。

根目录验证不替代以下独立交付验证；只有安装包而没有根工程时直接使用独立入口。

记录并实际执行独立配置、构建、ctest 和安装；使用所选 SDK 的安装前缀。
命令中的 name、PackageName、路径按实际组件填写，不能留下占位符就声称验证成功。

~~~sh
cmake -S components/name -B build-name -DCMAKE_BUILD_TYPE=Debug -DCMAKE_PREFIX_PATH=/path/to/sdk-prefix
cmake --build build-name -j4
ctest --test-dir build-name --output-on-failure
cmake --install build-name --prefix /path/to/component-prefix
~~~

安装后用临时的最小消费者 find_package 并链接 PackageName::component，
包含公开头并调用一个公开接口；它不创建 Integration，也不放入正式 products 目录。
源码 SDK 需要先配置、构建和安装；安装包用户无需 SDK 源码。
组件内部测试放自己的 tests；除非修改了公共构建机制，不扩大到整仓重构。

交付时简述组件契约、选择的执行方式、已运行验证和仍需真实数据/设备验证的内容。
Skill 不授权额外提交、推送、安装到全局技能目录或改变 SDK 公共接口。
