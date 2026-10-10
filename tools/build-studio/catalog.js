// Capability catalog; the UI does not inspect implementation internals.
globalThis.BuildCatalog = {
  schemaVersion: 1,
  components: [
    { id: "odom", name: "Odom", description: "PassthroughOdomComponent · 现有里程计输入与输出",
      implemented: true, package: "Odom", target: "Odom::component", sourcePath: "components/odom" },
    { id: "map-updater", name: "Map updater", description: "OccupancyMapComponent · 占据地图更新",
      implemented: true, package: "MapUpdater", target: "MapUpdater::component", sourcePath: "components/map-updater" },
    { id: "map-server", name: "Map server", description: "FileMapComponent · 地图保存与加载",
      implemented: true, package: "MapServer", target: "MapServer::component", sourcePath: "components/map-server" }
  ],
  scenarios: [
    { id: "current", name: "当前产品", integration: "RobotIntegration", buildReady: true,
      components: ["odom", "map-updater", "map-server"], defaults: ["SOURCE", "SOURCE", "SOURCE"] }
  ],
  platforms: [
    { id: "linux-x86_64", name: "Linux x86_64 · 本机开发", buildReady: true,
      detail: "面向 x86_64 WSL / Linux；页面不检测本机架构，不执行构建。" },
    { id: "linux-aarch64", name: "Linux ARM64 · 交叉编译预览", buildReady: false,
      detail: "需要匹配的交叉编译器、toolchain、sysroot 与 ARM64 依赖包；尚未做交叉编译验证。" }
  ]
};
