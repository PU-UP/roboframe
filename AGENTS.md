# roboframe 开发入口

main 是空产品骨架；完整导航/建图实现保存在 demo 分支。
工作前先看 git status，保留用户改动；build* 与 history 不代表当前源码。
先读 README.md、docs/architecture.md、docs/sdk-review.md 和 sdk/README.md。
开发或维护组件时读取 sdk/skills/roboframe-component/SKILL.md；
组件可以直接测试，不需要先创建产品或 Integration。
开发或维护产品/integration 时读取 sdk/skills/roboframe-product/SKILL.md，沿用已有结构和契约。
仓库内组件开发优先固定 build-dev，设置 ROBOFRAME_COMPONENT=<name> 并清空 ROBOFRAME_PRODUCT；
产品开发同用 build-dev，设置 ROBOFRAME_PRODUCT=<name> 并清空 ROBOFRAME_COMPONENT。
不要为每次构建新建目录或改编辑器索引。独立交付检查可放临时目录。
使用或新增 SDK skill 时遵循 sdk/skills/feedback-format.md；
本地记录放 .roboframe/feedback/，开发 agent 只新增未读项，维护者审查时再填写状态评价。

- C++14；组件只依赖 SDK/自身契约，不认识产品、BT 或其他组件。
- 组件按需持有 Server、ExecutionChannel，或管理算法已有线程。
- integration 只做接线、生命周期协调、具体 BT/任务；不重复实现调度。
- interface 只负责 IPC 与公开契约适配；控制输出可短回调入独立发送队列。
- 生命周期只有一个 owner；先关闭接纳/请求全部取消，再等待完成。
- 组件析构前停止回调和后台执行；取消协作式，不承诺强杀。
- SDK 不接纳产品协议/配置；不要预建新抽象或恢复整个 demo。
- 数据共享保持不可变；队列、单帧、历史和外部快照分别设预算。
- 主分支组件/产品目录保持空，直到用户开始其真实接入。

验证：
    cmake -S . -B build-skeleton -DCMAKE_BUILD_TYPE=Debug
    cmake --build build-skeleton -j4
    ctest --test-dir build-skeleton --output-on-failure

修改构建、安装或公开头时另运行：
    python3 tests/build_isolation_test.py

已验证 WSL Ubuntu 22.04 / x86_64；真实算法、实机控制、ARM/Windows 原生未验证。
只报告实际运行结果。不要在没有用户要求时推送或发布。
