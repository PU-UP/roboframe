# Roboframe SDK 4.0
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
| 跨组件接线、缓存与时间匹配、业务流程和未公开的产品状态 | integration；算法内部同步与融合留在 component |
| 传输解析、协议编码和传输资源 | interface |

integration 装配具体组件，所以其实现依赖组件的公开接口是正常的；
interface 不依赖组件数据类型或头文件。integration 公开入口只引用 SDK；
具体组件和 PoseBuffer 在私有 robot_integration_impl.hpp 中，CMake 组件依赖为 PRIVATE。
main 拥有 integration，StdioInterface 只借用引用。interface 仍依赖具体产品的公开操作，
本次没有引入适用于所有产品的抽象接口。

组件封装与组合遵循 [主文档架构原则](../README.md#架构原则能力封装与按需组合)：
SDK 统一必要的输出数据语义，不据此要求所有算法采用同一种输入或统一能力基类。
产品提出频率与延迟要求；满足这些要求所需的内部传播、融合由具体组件负责。

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
| grid.hpp / grid_cells.hpp | odom 系二维栅格、4 KiB 分块写时复制、只读快照 |
| data_flow.hpp | 产品对外的数据共享资源统计 |
| mapping.hpp | 可选建图公开参数与状态，不规定产品工作流 |
| log.hpp | ROBO_LOG_INFO 等日志，默认 stderr，可替换 sink |
| version.hpp | SDK 兼容核对与组件源码标识，不保证二进制 ABI |

Scan2D 发布后生产者不得通过其他可写别名修改数据；消费者可以保留只读共享指针。
20 米与 2048 点限制属于当前地图更新组件的输入策略，不是所有 SDK 消费者的限制。
现有 GridSpec 校验仍混有资源上限，已记入待办，不在本次扩大重构。

SDK 不强制 Integration 基类或通用 AddData。无真实配置的组件不设空 Config。
integration 与地图写入串行，planner 使用受限异步读取；各组件头文件声明并发、回调和异常语义。
日志 sink 不重入日志 API；其异常被日志层吞掉。日志及 planner 的跨线程入口有同步；SDK 类型不自动保证所有可变对象的线程安全。
组件 tests 下的 main 可脱离框架运行，不维护两套相同示例。

## 版本与交付
版本维护只有两个入口：
- SDK 维护者修改 sdk/VERSION，发布并标记兼容性；同 major 的后续版本必须兼容之前版本。
- 产品维护者在 cmake/Dependencies.cmake 选择 SDK 的精确版本和组件清单。

component 开发者不维护 SDK 版本、组件发布数字或独立 contract_version。
独立开发时通过 CMAKE_PREFIX_PATH 或 RoboframeSDK_DIR 使用产品指定的 SDK；
根目录构建会检查源码 SDK 或安装包是否与产品选择一致。

SDK 2.0 已移除独立契约版本；SDK 3.0 将 Grid::cells 改为分块写时复制，
Scan2D 改为只能移动而不能复制。SDK 4.0 增加明确的 TF 数据结构、GridSpec.frame 和 mapper 完整变换入参。
这些是公开接口变化，旧组件开发包必须重建。
Scan2DPtr 和 GridSnapshot 跨模块只读共享；具体所有权与更新约束见
[数据共享说明](../docs/data-sharing.md)。
产品配置阶段会拒绝缺少构建元数据的旧包。

组件身份在 CMake 构建时自动生成：
- 名称：由组件构建目标产生。
- source_id：对组件 src/、include/、cmake/ 与 CMakeLists.txt 内容计算 SHA256；
  包含未提交改动，不依赖 Git，也不受所在绝对路径影响。
- built_with_sdk：实际编译 SDK，由库的 GetComponentInfo() 返回。
- 安装清单额外记录编译器与目标系统/架构，便于追溯，不冒充完整 ABI 检查。

编辑、新增或删除上述源码会自动触发重新生成。源码指纹用于识别内容，
不表达功能大小或兼容性；不包含测试、文档、外部依赖内容或所有编译选项。
当前组件没有范围外算法源码；将来增加其他源码目录时需同步扩大指纹输入。

生成头文件在 build 中，只含元数据，业务源码仍在源目录。
安装包 share/roboframe/<component>/VERSION 是自动生成的记录，不是开发者维护入口；
SDK 的清单位于 share/roboframe/sdk/VERSION。
组件头文件、库、CMake 导出和清单必须一起交付。产品通过源码目录、安装前缀或
各组件的 <Package>_DIR 选择具体实现；不会自动下载或升级为“最新版”。

配置阶段检查产品 SDK 选择和组件编译 SDK；运行前再次检查宿主 SDK
是否与组件的 built_with_sdk 同 major 且不低于它。检查只比较 SDK，
不再要求 integration 手写组件契约版本。
预编译组件保留原来的 SDK 与源码标识，宿主升级不会改写这些事实。
启动日志输出名称、源码标识、编译 SDK；stdout 仍只用于 IPC。

SDK 统一维护，各产品可以固定不同发布版本。SDK 数字与源码指纹都不能自动证明
组件私有接口或 C++ ABI 兼容；产品维护者仍须选择并验证配套组件包。
不兼容 SDK 会明确失败，不自动重编第三方安装包。

~~~sh
cmake -S sdk -B build-sdk
cmake --build build-sdk
ctest --test-dir build-sdk --output-on-failure
cmake --install build-sdk --prefix /tmp/roboframe-sdk
~~~

TF 契约见 [坐标链路](../docs/transforms.md)：TimedPose2D 固定为 T_odom_base，不能塞入全局校正 pose。
