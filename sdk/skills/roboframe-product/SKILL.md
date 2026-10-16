---
name: roboframe-product
description: 使用已有 Roboframe C++14 组件开发与维护产品及 integration（功能迭代、故障修复、接线调整），复用现有生命周期、接线、可选 BT 和产品构建机制，完成集成验证。组件独立接入验证使用 component skill，不为此创建产品。
---

# Roboframe product 开发与维护

将已有组件装配成可运行产品，不恢复历史 demo，不另建框架。
缺少组件能力时说明缺口并记录反馈；组件实现按 component skill 单独处理，不把算法塞进 integration。

## 反馈与真实接口

按共用的 [反馈规范](../feedback-format.md) 先记录背景，过程中追加问题，结束补齐验证并给出记录路径。
开发 agent 只新增未读项，不自行填写维护者评价；无问题也保留运行记录。

先读当前宿主仓库的开发指引、产品需求及所选组件 README/公开头。
本文件向上两级为 SDK 资源目录，源码与安装包均带相邻 templates：
- 按需参考 [数据流接线](../../templates/integration/stream_integration.hpp)、
  [BT 编排](../../templates/integration/bt_integration.hpp) 和 [模板说明](../../templates/README.md)。
- 读取实际选中 SDK 的 core/integration.hpp、core/runtime.hpp；
  使用 BT 时再读 orchestration/sequence.hpp、orchestration/server_action.hpp。
  源码公开头在 SDK 资源目录的 include/roboframe；安装包从 Roboframe::sdk 的 INTERFACE_INCLUDE_DIRECTORIES 定位。
- 产品构建依据宿主根 CMakeLists.txt、cmake/RoboframeComponents.cmake；
  tests/build_isolation_test.py 仅供查构建例子，不复制其模拟产品。
  SDK 安装包不包含宿主根工程，只有安装包时沿用开发者的宿主构建入口，不假定这些文件存在。

## 维护已有产品

先读取现有契约、接线、调用方和测试；修复先复现，功能迭代先明确行为变化。
沿用现有目录与接口，只修改需求涉及的组件选择、接线/任务和对应测试、README；
模板用于核对机制，不覆盖已有产品，不因使用 skill 重建骨架或迁移无关代码。
按影响范围执行回归；改变依赖来源、构建或交付方式时再补相应隔离/包模式验证。

## 最小产品与边界

新产品在仓库内使用 products/<name>，交付 Dependencies.cmake、CMakeLists.txt、main.cpp、
integration 实现、tests 和 README。开始时一个 integration 文件即可，契约、配置及任务文件按需求拆分。

- 组件负责算法、端口和自身执行；integration 持有或引用组件，负责接线、生命周期协调和产品任务。
  不接管组件内部 Server/ExecutionChannel，不使用 UseServer/UseExecution 重复管理其生命周期。
- main.cpp 负责外部接入及运行入口；interface 负责 IPC，宿主适配代码转换为公开契约。
  不把算法或任务策略放进通用 interface；只需独立验证一个组件时不创建产品。
- 按实际需求选择 BT；简单接线无需建树。现有 Sequence 是记忆式顺序执行，
  没有 XML、反应式树或自动恢复。目标准备、结果解释、超时/重试策略属于产品。
  ServerAction 仅适配组件公开的目标执行能力；组件不认识 BT。
- 明确各端口线程、数据所有权及错误行为。重计算留组件，主线程 Poll 只做有界接线/编排。
  需要独立输出时，组件短回调直接入线程安全发送队列；不能经过 Poll 后仍声称独立于主线程。

采用 Integration 时，按真实接口实现 OnStart、CloseInputs、OnStop、OnReset、OnPoll：
生命周期与 Poll 由同一个控制 owner 调用；主循环持续驱动 Poll，阻塞收数不能饿死任务推进。
停止先关闭输入并请求所有组件取消，再等待停用；RequestDeactivate 是组件可选接口，不是基类保证。
BT Halt 只是取消请求；ServerAction 在 Tick/Drained（含 Reset 内部）领取后回调一次完整 Result，包括取消与收尾异常。
Drained 为 true 只表示收尾完成；产品根据 Result.state/error 判定结果，不额外查询终态。
结果回调在同一控制 owner 上同步执行，须不抛异常、不重入动作或生命周期；收尾后才能复用动作。
派生析构在成员销毁前调用 Shutdown；处理部分启动失败，CloseInputs/OnStop 不抛异常。
回调接收者活到组件停止之后，再排空并关闭发送端；旧任务结果不得进入新任务。
Poll 不自动捕获业务异常，产品须明确失败与停止策略，取消不承诺强杀阻塞算法。

## 构建与交付

Dependencies.cmake 设置所选 SDK 的精确 ROBOFRAME_REQUIRED_SDK_VERSION，
并定义 roboframe_load_components()，逐项调用 roboframe_component(directory PackageName)。
不要硬编码 skill 编写时的版本；从所选 SDK 获取版本。

产品 CMake 链接 PackageName::component，使用 BT 才链接 Roboframe::orchestration；
按需复用已有 interface 目标，不直接编译其实现文件。组件附带的独立接入程序按需关闭。
CMake 只组织目标、依赖和测试；回放/结果判断放 tests 脚本，缺少显式启用的依赖明确报错。

仓库内日常开发固定 build-dev，清空组件选择；name 换成实际产品名：

~~~sh
cmake -S . -B build-dev -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON -DROBOFRAME_SDK_SOURCE=SOURCE -DROBOFRAME_COMPONENT= -DROBOFRAME_PRODUCT=name
cmake --build build-dev -j4
ctest --test-dir build-dev --output-on-failure
~~~

组件来源由 ROBOFRAME_<KEY>_SOURCE=SOURCE/PACKAGE 选择，KEY 是组件目录名大写并将连字符换为下划线。
SDK 来源用 ROBOFRAME_SDK_SOURCE；包模式设置 CMAKE_PREFIX_PATH，不重写宿主查找逻辑。
clangd 沿用 build-dev/compile_commands.json，不修改个人编辑器配置。
安装/混合来源验证按实际交付需求放临时目录，不替代日常索引。

## 验证与说明

直接构造 integration、输入数据并断言跨组件输出，不要求先连真实 IPC。
覆盖实际存在的正常流程、错误、取消、停止和重启；异步路径再检查迟到结果与回调寿命。
使用有截止时间的条件等待；要求独立输出时阻塞主线程并验证输出继续。
新增产品或修改外部接入时另做一次最小端到端验证，回放同时检查输入完整性、退出码和结果。
测试通过不等于精度或实机实时性证明，不重复组件内部全部单测。

README 写明组件清单、公开入口、简洁数据流与线程安排，以及实际验证过的构建、运行、测试命令。
交付说明已验证范围、未覆盖项和反馈路径；不要仅以“编译成功”声称业务跑通。
Skill 不授权额外提交、推送或修改 SDK 公共接口。
