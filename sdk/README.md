# Roboframe SDK 1.2
interface、integration、component 共用的公开契约和小型基础设施。
SDK 位于主目录的 sdk/，与 interfaces/、products/、components/ 并列；
后两种复数目录是未来多产品布局，当前不为占位而搬动目录或创建空产品。

## 类型归属与依赖边界
**跨 interface 与 integration 边界的自定义数据类型必须由 SDK 定义。**
这包括输入数据、请求参数、对外状态和结果；基础数值、字符串直接使用标准类型，
不为每个字段另造包装类。ROS/DDS 等原生消息属于传输适配层，转换后才进入 integration。
SDK 的 C++ 对象不是线协议，不能直接把对象内存作为 IPC 数据发送。

| 内容 | 所属位置 |
|---|---|
| 时间、坐标、Scan2D、GridSnapshot 等稳定数据 | SDK 对应领域头文件 |
| 建图公开配置、状态 | SDK mapping.hpp；不含建图流程实现 |
| 产品独有且需要跨 interface 的请求/结果 | 确有需要时放 SDK products/<product>/，不污染通用类型 |
| 组件独有配置、局部更新结果、诊断与算法参数 | 具名组件头文件；只有需独立复用时才拆组件契约头 |
| 接线、缓存、同步策略、业务流程和未公开的产品状态 | integration |
| 传输解析、协议编码和传输资源 | interface |

integration 装配具体组件，所以其实现依赖组件的公开接口是正常的；
interface 不应依赖组件数据类型。当前已经统一边界数据，
但 integration 公开头文件仍传递包含具体组件头文件，这项编译依赖隔离在
[统一待办](../TODO.md) 中延期处理，不能把本次迁移称为完全解耦。

## SDK 准入规则
1. 新类型必须有实际的公开边界或独立消费者；不按未来模块清单预建类型。
2. 先判断它是通用数据、领域 API 还是产品专有 API；不能因为进入 SDK 就宣称跨产品通用。
   产品专有契约按产品隔离；私有业务流程不进入 SDK。
3. 优先复用已有语义一致的类型。不能用可选字段不断堆出万能消息，也不引入 any/AddData。
4. 每个小头文件按需引用并能独立编译。基础类型不反向依赖领域或产品类型，
   领域类型不依赖产品类型；整个 SDK 不包含 component、integration、ROS 或 DDS 实现。
5. 契约必须说明单位、坐标系、时钟、所有权和有效性。操作接口还需说明异常、回调和并发要求。
   SDK 检查结构有效性；传输和消费者另外限制容量、频率、范围等资源策略。
6. 新增或修改公开契约需同时核对生产者、消费者、安装导出与兼容性；
   未使用的公开类型先走弃用与版本流程，不随意删改。
   这些规则是代码评审约束，当前没有新增自动架构检查工具。

| 头文件 | 职责 |
|---|---|
| component.hpp / lifecycle.hpp | Activate、Deactivate、生命周期 |
| task.hpp | 可选任务状态和 ID，不管理线程或产品流程 |
| geometry2d.hpp | 微秒时间戳、米/弧度、T_odom_base |
| scan2d.hpp | base_link 下只读二维命中点；不代表所有原始雷达消息 |
| grid.hpp | odom 系二维栅格、只读快照 |
| mapping.hpp | 可选建图公开参数与状态，不规定产品工作流 |
| log.hpp | ROBO_LOG_INFO 等日志，默认 stderr，可替换 sink |
| version.hpp | SDK、实现和契约版本核对，不保证二进制 ABI |

Scan2D 发布后生产者不得通过其他可写别名修改数据；消费者可以保留只读共享指针。
20 米与 2048 点限制属于当前地图更新组件的输入策略，不是所有 SDK 消费者的限制。
现有 GridSpec 校验仍混有资源上限，已记入待办，不在本次扩大重构。

SDK 不强制 Integration 基类或通用 AddData。无真实配置的组件不设空 Config。
调用目前串行；组件头文件声明回调生存期、重入和异常语义。
日志 sink 不重入日志 API；其异常被日志层吞掉。线程安全只覆盖日志。
组件 tests 下的 main 可脱离框架运行，不维护两套相同示例。

## 版本与交付
保持一套 SDK 源码、维护和发布体系，各产品可以固定不同版本，无需同时升级。
产品固定 SDK 与组件包组合；不能在同一个可执行文件中随意混入不兼容的 SDK 头文件。
版本固定清单和目标平台兼容矩阵的自动化见待办。

VERSION 由发布维护者调整，普通源码修改无需改号。
SDK/组件/主框架实现版本分别管理：兼容修复 patch，兼容扩展 minor，破坏兼容 major。
组件 contract_version 在公开头文件中，描述公开接口与 I/O 语义；算法内部改变无需修改它。
CMake 从 sdk/VERSION 生成 sdk_version.hpp；组件 .cpp 返回编译时 SDK 版本。
SDK 更新后重编会更新这个记录，不要求开发者抄写 SDK 版本；预编译组件仍保留原值。

本次 SDK 1.2 增加 Scan2D 和建图公开数据。MapUpdater 的入参类型迁移属于源码接口变化，
其实现版本与契约升为 2.0；当前产品要求 SDK >=1.2、MapUpdater >=2.0，各自同 major。
Odom、MapServer 契约仍为 1.0，独立构建最低支持 SDK 1.1。

integration 在激活前核对库信息：宿主 SDK 必须 >= 组件编译 SDK 且同 major，
组件契约必须 >= 产品声明的最低要求且同 major。不兼容抛异常、拒绝启动。
运行检查不能挽救无法链接或调用的 C++ ABI 错误；目标架构、编译器和标准库仍须一致。
当前组件为静态库。SDK 小版本号本身不保证所有 C++ 布局与编译选项兼容。

~~~sh
cmake -S sdk -B build-sdk
cmake --build build-sdk
ctest --test-dir build-sdk --output-on-failure
cmake --install build-sdk --prefix /tmp/roboframe-sdk
~~~
安装包只有公开头文件、生成版本头和 CMake 导出；业务源码不写进 build。
