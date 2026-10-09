// Edit this catalog to extend the demo UI. This does not register components with CMake.
globalThis.BuildCatalog = {
  schemaVersion: 1,
  components: [
    { id: "sample-accumulator", name: "Sample accumulator", description: "现有任务型组件 · 整数累计",
      implemented: true, package: "SampleAccumulator", target: "SampleAccumulator::component",
      sourcePath: "components/sample-accumulator" },
    { id: "planner", name: "Planner", description: "未来示例 · 点到点规划",
      implemented: false, package: "Planner", target: "Planner::component", sourcePath: "components/planner" },
    { id: "vio", name: "VIO", description: "未来示例 · 视觉惯性里程计",
      implemented: false, package: "VIO", target: "VIO::component", sourcePath: "components/vio" },
    { id: "controller", name: "Controller", description: "未来示例 · 路径跟踪",
      implemented: false, package: "Controller", target: "Controller::component", sourcePath: "components/controller" }
  ],
  scenarios: [
    { id: "current", name: "当前骨架", integration: "DemoIntegration",
      buildReady: true, components: ["sample-accumulator"], defaults: ["SOURCE"] },
    { id: "navigation", name: "未来导航示意", integration: "导航 integration（待实现）",
      buildReady: false, components: ["planner", "vio", "controller"], defaults: ["SOURCE", "PACKAGE", "PACKAGE"] }
  ],
  platforms: [
    { id: "linux-x86_64", name: "Linux x86_64 · 本机开发", buildReady: true,
      detail: "命令面向 x86_64 WSL / Linux 环境。网页不检测本机架构。" },
    { id: "linux-aarch64", name: "Linux ARM64 · 交叉编译示意", buildReady: false,
      detail: "需要匹配的交叉编译器、toolchain、sysroot 与 ARM64 依赖包；选择架构不会自动获得它们。" }
  ]
};
