# roboframe 开发入口

本仓库是 C++14 机器人算法框架；当前完整示例为 products/demo_robot。
工作前先看 git status，保留已有未提交改动；构建产物和 history 不代表当前实现。

## 先读

1. README.md：目录、构建、回放。
2. docs/architecture.md：运行骨架、产品接线、线程与所有权边界。
3. TODO.md：当前基线、已知限制和建议顺序。
4. sdk/README.md：公开类型准入和交付规则。
5. 改示例产品时再读 docs/mapping.md、docs/ipc.md。

沿 main.cpp → SDK Integration → demo_robot_integration → data_flow → tasks 阅读。
新产品参考 sdk/templates/integration，不复制整个 demo_robot。
协议适配依赖公开产品 contract；组件不认识 integration 或其他组件。

## 修改约束

- C++14，无不必要依赖；保持 typed 接线和不可变共享大数据。
- SDK core/datatype 不依赖产品；产品配置/协议留产品 contract。
- 生命周期、任务、Poll/查询有一个控制 owner；传感器入口可并发。
- 输入短锁不运行耗时算法；按需复用 ExecutionChannel，不强制每组件一线程。
- RUNNING 与任务分开；CANCEL 不清持续数据流；STOP/RESET 必须处理迟到结果。
- 大数据容量包含待处理/计算中/完成未领取；历史和外部引用另设预算。
- 图像/IMU、Brain/BT、导航、子仓库仅在实际任务要求时增加。未来方案不是当前实现。
- SDK 版本由维护者发布；组件构建信息自动生成，不手填组件/契约版本号。

## 验证

常规：
    cmake -S . -B build-product -DROBOFRAME_PRODUCT=demo_robot -DCMAKE_BUILD_TYPE=Release
    cmake --build build-product -j4
    ctest --test-dir build-product --output-on-failure

修改依赖、安装或头文件时运行：
    python3 tests/build_isolation_test.py

修改数据流/坐标/地图时按 README 跑全量 tf 回放；输出目录须为新目录。
数据来源和限制见 data/intel-lab/README.md，不将录制修正轨迹称为在线 SLAM。
慢读/断连/关闭、延迟/过载、共享快照测试已纳入 ctest。
修改 Build Studio 时运行 node tools/build-studio/check.cjs（仅命令预览工具）。

现有验证平台为 WSL Ubuntu 22.04 / x86_64；ARM、真实 VIO/SLAM 与 ROS/DDS 尚未验证。
每次只报告实际运行的检查；更新 TODO 时保留未解决限制，不堆积逐轮日志。
