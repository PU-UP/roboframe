# 开发设计回顾与重新起步

日期：2026-10-09。本文保留此前讨论的需求和经验，不代表本轮都要实现。
后续已引入可选任务接口、PushSample 与独立 SDK；当前协议、职责和依赖以根目录 README 为准。
组件现已迁入 roboframe/components/sample-accumulator；下文保留的并列目录描述是历史记录。
当前可运行版本的说明见根目录 README；上一轮实现完整保存在 history/2026-10-09-full-prototype/。

## 为什么重新起步

上一轮 roboframe 跑通了 C++14 组件、IPC、BT 导航、建图/重定位演示和真实 odom/scan 建图。
最近一次完整回归为 16/16 通过。但用例通过只能证明那些调用方式有效，
不能证明异步图像分发、位姿匹配、多产品装配和独立模块开发已经形成稳定架构。

扩展速度超过了边界验证速度：为了跑通场景，新增了多个运行入口、组合层和专属组件；
同一模块被拆散在 contracts、components、algorithms，阅读和独立维护成本变高。
本轮恢复最小可理解链路，后续每次扩展只解决一个已复现的问题。

## 保留的需求与原则

1. 三层主线保持 interface → integration → component，结果通过回调反向返回。
   interface 管通讯适配；integration 管组装、数据连接、算法业务流程；
   component 管明确的算法输入输出与自身状态。
2. slam-core 是已经用于产品的参照。保留组件封装、integration 构造/回调接线、
   通讯可替换、模块可独立开发的思路，不把“成熟但有负担”误判成“整体设计失败”。
3. 产品业务与算法流程区分：日程、用户任务、刀盘、电量策略可在外部产品进程；
   算法框架执行估计、感知、导航及其必要协调。两者通过 interface 交互。
4. 不同产品可以有各自 integration，不把所有差异塞进一个产品分支大全。
   一种产品建图并复用，一种直接工作；是否需要全局定位须根据任务坐标判断。
5. 算法作者只需理解所属模块的契约；框架作者依赖公开接口，不读取算法内部状态。
   一个组件的类型、接口、实现、测试尽量放在自己的目录/仓库。
6. 标准能力的语义可以固定，实现可以替换：如 VIO/LIO/DR 的局部位姿输出。
   框架以后应提供标准能力集合，产品装配规则说明哪些必需；不要求所有产品启动所有模块。
   非预定义能力（例如二维码识别）仍可扩展。
7. 输入输出需明确时间、坐标、有效性和所有权。共享大数据以只读为前提；
   图像推理结果保留采集时间，最终应与该时刻位姿关联，不能随手取最新值。
8. BT 的引入意向保留，属于 integration 的任务组织；本轮不实现。
   BT 不承担逐帧传输、线程调度和时间同步。
9. 开发环境可为 x86/ROS，部署可为 ARM/自研 DDS。组件不依赖传输协议。
   当前坚持 C++14；本轮仅在 WSL Ubuntu-22.04 原生验证。
10. 可观测、可复现：演示必须有可读的输入、接线路径、输出和失败示例。

## 上一轮 roboframe 已识别的问题

- 图像没有在产品入口同时分发给 VIO 和模型。on_image 与 camera 是两条独立入口。
- EnvironmentPipeline 同步调用推理，拿入口时的当前 odom，以默认 30 ms 门限校验；
  没有历史位姿缓存、插值或异步等待机制。
- Fusion 用同一个 odom 时间递增检查处理多源输入，同时间的激光/图像可能互相排斥；
  延迟推理结果也没有完善的接入规则。上述机制未经过真实并发验证。
- NavigationRuntime 拥有 Odometry，EnvironmentPipeline 固定拥有 Model/Fusion/MapUpdater，
  数据流和所有权混在几个固定组合中，产品必需能力却主要靠约定。
- 通用 Model 实际只是固定标签分割，名字大于实际契约。
- MapBuilding 是 integration 的会话流程，不是新的一类 SLAM 算法。
  真正 SLAM 建图/定位后端的共享状态方案尚未完成。
- 真实数据入口后期才补齐显式 IPC 开始、结束、保存流程。
  测试只覆盖选定片段，不能表示 SLAM 精度、实机停车或全数据集通过。
- 目录按技术层散开，造成一个组件的契约、实现、测试难以作为一组阅读。

## 本轮从 slam-core 提取什么

参照源码根目录：/home/watermango/work/slam-core/slam_architect_ws/。

| 原代码 | 保留的骨架 | 本轮的简化 |
|---|---|---|
| slam-base/base_interface/base_interface.hpp | interface 持有 integration，接收外部输入并发布结果 | 标准输入输出管道；不复制 ROS/DDS 和插件装载 |
| slam-base/base_integration/base_integration.hpp/.cpp | AddCommand、AddSensorData、构造组件、注册组件回调、输出回调 | 一个明确的组件成员；两个业务状态；不用字符串组件管理器 |
| integration/vision2/algorithm_callback/register_components.cpp | integration 连接组件结果 | 直接类型化回调，不做全局注册工厂 |
| slam-base/base_algorithm/base_algorithm.hpp | AddData、Reset、结果回调；组件封装具体计算 | 使用独立组件的具名输入输出；同步计算，不创建空转工作线程 |
| algorithm/dr-odom/dr_odom_component.hpp | 组件目录拥有其接口与算法实现 | 示例组件独立构建，不接收 integration 配置或引用主框架 |

保留的是职责与调用关系，不是复制企业产品源码、配置、线程和历史兼容层。
本轮不抽万能基类，不为了名字对应保留没有实际用途的 Evaluate/LoadConfiguration。

## slam-core 的核心问题：证据与后续讨论

以下是本轮阅读相关代码的架构判断，不是整个成熟产品的完整缺陷审计。

### 1. 通用 integration 中已有具体产品/组件知识

base_integration.cpp::AddSensorData 中直接访问 ai_model、map_fusion、anomaly_detection，
强制启用输入；ConstructAlgorithms(skip_perception) 也识别具体组件名称。
base_integration.hpp::ClearAllComponentPtr 对 ai_model、anomaly_detection 作特殊保留。

问题不是 integration 不能管理组件，而是产品策略进入了通用基座。
建议后续优先分清“可复用的接线机制”与“某产品如何组装和启停”，不先发明新的调度系统。

### 2. 组件配置与工作模式边界不够清晰

Algorithm 构造接口接收 integration_configure_path；DROdomComponent 也沿用。
ModelComponent 有基于 topic 的工作模式与模型切换命令。

模型切换、重置等操作本身可以是合法算法能力，不能一概说成业务泄漏。
需要逐项区分：integration 决定何时切换，component 实现如何切换；
component 仅接收自己的参数，不自行解释完整产品配置。

### 3. 统一 AddData 外形之下，契约仍依赖 topic 与数据类型约定

AddSensorData 广播同一个 Data 给多个 ComponentHandle；各组件再筛选 topic/类型，
回调也按字符串名称在 integration 中连接。这使接入灵活，但难在边界发现接线和语义错误。

本轮只用一个具名 Sample/Result 表达最小契约。不借此全面批判或迁移 common::Data；
它有历史产品兼容价值。以后在确实需要共享的数据处逐项定义契约。

### 4. 生命周期、线程与对象有效性要先于更多功能明确

部分组件构造时创建工作线程，析构时唤醒并 join；ComponentHandle 同时保存
status、need_construct、enable_data_input、ptr。其 Reset 只检查 need_construct，
没有像 AddData 一样检查 ptr；这说明“可调用”与“已构造”的契约值得收紧，
不等于已经证明产品必然走到该异常路径。

后续应先写清：谁启动/停止、停止后是否还有回调、Reset 如何处理待处理数据。
本轮采用单线程同步调用，避免先造一套尚未验证的异步生命周期。

### 5. 数据共享和时间关联需要针对具体链路验证

slam-core 本身已有时间检查、队列和时间配对。例如 tag_callback.cpp 使用
findClosestOdometry 后再检查 0.1 秒时间差。因此不能说它完全没有时间同步。

需要评估的是各链路是否采用一致、明确的时钟/匹配/超时/重置规则，
以及共享图像是否会被修改或重复拷贝。当前仅阅读相关链路，没有完成全局拷贝量、
线程竞争或延迟分析；不能把这些待核查项写成已证实的性能故障。

## 本轮验收和下一步

只完成两个仓库形态的目录：roboframe 主框架、sample-accumulator 独立组件。
独立组件提供 AddData / Reset / SetResultCallback，主框架只链接其公开 CMake target。
主框架保留 interface、integration、main；Python 是外部调用者，不是第三个代码仓库。

验收：
- 组件不读取、不包含 roboframe；能独立编译、测试。
- 主框架只消费组件安装后的头文件和库；搬迁安装目录仍能构建。
- 实际进程管道走通 START → SAMPLE → 结果回调 → STOP。
- 非法命令、未启动输入、重置、EOF 清理、计算溢出有明确行为。

下一轮先讨论 integration 的职责与 component 的最小契约。
然后再引入第二个真实组件来验证数据分发；有必要时再做异步与时间匹配。
导航、BT、标准能力集合、通用调度器和动态插件都不在本轮实现范围。


## 最小同步骨架阶段的完成记录（后续演进见当前 README）

- 当前主框架只有 main.cpp、interface 的一组 hpp/cpp、integration 的一组 hpp/cpp。
- 独立组件在 /home/watermango/work/sample-accumulator，公开头文件与计算实现各一个。
- 已通过组件独立测试，以及主框架的 5 项 IPC 检查（正常链路、会话门禁、非法输入、
  溢出不改变结果、EOF/残缺输入）。
- 把组件源文件复制到临时目录，脱离主框架构建和测试通过。
- 把主框架源码和组件安装包搬到临时目录，编译命令不引用原工作区，构建和测试通过。
- 缺少组件安装包时配置失败；当前主框架头文件可单独按 C++14 编译。
- 可读执行打印保存在 build/demo-output.txt；隔离验证记录在 build/isolation-check.txt。
- 旧项目归档验证了 2,145 个文件，合计 724,693,307 bytes；目录/符号链接也在清单中。
  文件未删除，旧 CMake 缓存的绝对路径不保证搬迁后仍可直接使用。

以上不包含真实传感器、并发、ARM、ROS/DDS 或 slam-core 全量产品测试。

## SDK 基础版完成记录（2026-10-09）

当前正式结构以根目录 README 为准：组件现位于 roboframe/components/sample-accumulator。
默认构建改为源码连接，不必先安装；安装包消费模式仍保留，并经过搬迁与隔离验证。
SDK 增加最小组件生命周期基类、统一日志、版本检查和可运行组件/integration 开发模板。
独立组件 main 可直接调用组件，无需 IPC 或产品 integration；完整 IPC 示例保持原有协议。
SDK 版本、组件实现版本和组件契约版本分别管理；没有创建 Git 仓库或子模块。
