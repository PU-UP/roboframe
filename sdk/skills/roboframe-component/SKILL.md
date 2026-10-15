---
name: roboframe-component
description: 根据开发者需求创建或修改 Roboframe C++14 组件，复用当前 SDK 接口和执行模板，完成不依赖 Integration 的组件测试及独立构建、安装验证。用于组件开发，不负责产品 BT 编排或 SDK 架构重构。
---

# Roboframe component 开发

把开发者的算法与端口需求落实成独立组件，不替开发者定义算法业务。
只使用当前工作区或选定安装包的 SDK；不从历史 demo 恢复实现，不新增第二套骨架。

## 保留开发反馈

按共用的 [反馈规范](../feedback-format.md) 保留运行背景、验证和遇到的问题，结束时给出记录路径。
开发 agent 只创建未读项，评价由 SDK 维护者填写；无问题也保留验证结果。

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

组件须公开输入、输出及生命周期接口，说明调用线程、数据所有权和错误行为。
使用者可在自己的 main 中，通过 interface、ROS、文件或测试数据调用组件，无需 Integration。
接入和格式转换由外部程序完成，组件库不依赖具体接入方式；不预建适配器或教程。

通过此 skill 创建或修改组件时，必须交付或同步组件 README，方便开发者快速使用：
- 简述功能与输入输出契约，给出基于真实 API 的最小调用示例：构造、绑定输出、激活、输入、停用。
- 提供已验证的编译和测试命令；仓库内日常开发沿用 build-dev，与 clangd 的编译数据库一致。
- 将独立安装/交付验证与日常开发明确区分；验证目录不替代日常索引目录，不修改个人编辑器配置。

## 先独立验证组件

用普通程序调用公开接口并核对输出即可验证组件；通过真实接入方式验证同样无需 Integration。

- 同步组件直接调用并断言；异步组件使用自身执行器，通过公开结果接口或线程安全回调，
  以有截止时间的条件等待结果，不靠长时间 sleep 猜完成。
- Server 组件直接 Submit/GetStatus/TakeResult/RequestCancel，不需要 BT Tick 或 Integration::Poll。
- 按契约覆盖正常结果、非法输入、边界、停用、错误恢复及重启；
  有后台工作时再验证取消、迟到输出和资源释放。
- 地图、TF、传感器输入可由测试构造；外部依赖按需使用窄接口替身，不模拟整个产品。
- 预期结果来自契约、可手算输入或可信基准；精度、实时性需要的数据集/设备验证单独报告。

只有跨组件业务关系和产品任务编排需要相应的集成验证，不能把它们作为组件验证的前置条件。

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
4. 单元测试直接链接组件目标，用 add_test 登记；独立宿主程序按需链接 interface、ROS 等接入依赖。
   根目录须显式选择组件并确认其测试实际运行，不需要 Integration。

仓库内开发优先复用固定的 build-dev，选择当前组件，不创建产品：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_PRODUCT= -DROBOFRAME_COMPONENT=name
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

组件与产品选择互斥，切换时清空另一项；换组件只改 name，复用 build-dev。
SDK 可用 ROBOFRAME_SDK_SOURCE=PACKAGE 和 CMAKE_PREFIX_PATH 选择安装包。
clangd 使用 build-dev/compile_commands.json；生成头路径由 CMake 提供，不手工补 include。
不同工具链/生成器使用相应目录，不能混用缓存；仅用安装包开发时沿用宿主工程的构建与索引约定。

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
