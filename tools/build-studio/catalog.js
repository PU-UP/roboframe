// Capability catalog; the UI does not inspect implementation internals.
globalThis.BuildCatalog = {
  schemaVersion: 1,
  components: [
    { id: "odom", name: "Odom", description: "PassthroughOdomComponent · 现有里程计输入与输出",
      implemented: true, package: "Odom", target: "Odom::component", sourcePath: "components/odom" },
    { id: "slam", name: "SLAM", description: "RecordedSlamComponent · 录制参考位姿校正",
      implemented: true, package: "Slam", target: "Slam::component", sourcePath: "components/slam" },
    { id: "posegraph", name: "PoseGraph", description: "PoseGraphComponent · 坐标关系与子图锚点",
      implemented: true, package: "PoseGraph", target: "PoseGraph::component", sourcePath: "components/posegraph" },
    { id: "localmapbuilder", name: "LocalMapBuilder", description: "LocalMapComponent · 局部子图",
      implemented: true, package: "LocalMapBuilder", target: "LocalMapBuilder::component", sourcePath: "components/localmapbuilder" },
    { id: "map-updater", name: "Map updater", description: "OccupancyMapComponent · 占据地图更新",
      implemented: true, package: "MapUpdater", target: "MapUpdater::component", sourcePath: "components/map-updater" },
    { id: "map-server", name: "Map server", description: "FileMapComponent · 地图保存与加载",
      implemented: true, package: "MapServer", target: "MapServer::component", sourcePath: "components/map-server" },
    { id: "planner", name: "Planner", description: "PlannerComponent · 地图 A* 路径规划",
      implemented: true, package: "Planner", target: "Planner::component", sourcePath: "components/planner" },
    { id: "controller", name: "Controller", description: "ControllerComponent · TF 跟踪与速度输出",
      implemented: true, package: "Controller", target: "Controller::component", sourcePath: "components/controller" },
  ],
  scenarios: [
    { id: "current", product: "demo_robot", name: "Demo robot", integration: "DemoRobotIntegration", buildReady: true,
      components: ["odom", "slam", "posegraph", "localmapbuilder", "map-updater", "map-server", "planner", "controller"], defaults: ["SOURCE", "SOURCE", "SOURCE", "SOURCE", "SOURCE", "SOURCE", "SOURCE", "SOURCE"] }
  ],
  platforms: [
    { id: "linux-x86_64", name: "Linux x86_64 · 本机开发", buildReady: true,
      detail: "面向 x86_64 WSL / Linux；页面不检测本机架构，不执行构建。" },
    { id: "linux-aarch64", name: "Linux ARM64 · 交叉编译预览", buildReady: false,
      detail: "需要匹配的交叉编译器、toolchain、sysroot 与 ARM64 依赖包；尚未做交叉编译验证。" }
  ]
};
