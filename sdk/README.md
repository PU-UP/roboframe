# Roboframe SDK 1.1
integration、component 和 interface 共用的最小基础，不依赖产品或具体组件。

| 头文件 | 职责 |
|---|---|
| component.hpp / lifecycle.hpp | Activate、Deactivate、生命周期 |
| task.hpp | 可选任务状态和 ID，不管理线程或产品流程 |
| geometry2d.hpp | 微秒时间戳、米/弧度、T_odom_base |
| grid.hpp | odom 系二维栅格、只读快照 |
| log.hpp | ROBO_LOG_INFO 等日志，默认 stderr，可替换 sink |
| version.hpp | SDK、实现和契约版本核对，不保证二进制 ABI |

SDK 不强制 Integration 基类或通用 AddData。组件输入输出、配置和语义先集中在
具名组件头文件；只有多实现共享、独立消费者需要时才拆出本组件契约。
已移除数值演示模板，直接参照三个实际组件和 RobotIntegration。
组件 tests 下的 main 可脱离框架运行，不维护两套相同示例。

配置由 integration 提供、组件校验。无真实配置的组件不设空 Config。
调用串行；组件头文件声明回调生存期、重入和异常语义。
日志 sink 不重入日志 API；其异常被日志层吞掉。线程安全只覆盖日志。

VERSION 由发布维护者调整，普通源码修改无需改号。
SDK/组件/主框架实现版本分别管理：兼容修复 patch，兼容扩展 minor，破坏兼容 major。
组件 contract_version 在公开头文件中，描述 I/O 语义；算法内部改变无需修改它。
CMake 从 sdk/VERSION 生成 sdk_version.hpp；组件 .cpp 返回编译时 SDK 版本。
SDK 更新后重编会更新这个记录，不要求开发者抄写 SDK 版本。
预编译组件仍保留它原来使用的版本。

配置/编译要求 SDK >=1.1 且同 major。integration 使用自己声明的最低契约要求，
在激活前核对库信息；宿主 SDK 必须 >= 组件编译 SDK 且同 major，契约同理。
不兼容抛异常、拒绝启动。运行检查不能挽救无法链接或调用的 C++ ABI 错误；
目标架构、编译器和标准库仍须一致。当前组件为静态库。

~~~sh
cmake -S sdk -B build-sdk
cmake --build build-sdk
ctest --test-dir build-sdk --output-on-failure
cmake --install build-sdk --prefix /tmp/roboframe-sdk
~~~
安装包只有公开头文件、生成版本头和 CMake 导出；业务源码不写进 build。
