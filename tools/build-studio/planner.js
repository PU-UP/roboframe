/* Pure configuration preview. No command execution, filesystem writes or package discovery. */
(function () {
  "use strict";
  function initialState(catalog, scenarioId = "current") {
    const scenario = catalog.scenarios.find(item => item.id === scenarioId);
    if (!scenario) throw new Error("未知场景");
    return {
      scenario: scenario.id, platform: catalog.platforms[0].id,
      sources: Object.fromEntries(scenario.components.map((id, index) => [id, scenario.defaults[index]])),
      packagePrefix: "/opt/roboframe-deps/linux-x86_64",
      tests: true, demo: true
    };
  }
  function shellQuote(value) {
    return "'" + String(value).replace(/'/g, "'\"'\"'") + "'";
  }
  function buildPlan(catalog, state) {
    const scenario = catalog.scenarios.find(item => item.id === state.scenario);
    const platform = catalog.platforms.find(item => item.id === state.platform);
    if (!scenario || !platform) throw new Error("未知场景或目标平台");
    const components = scenario.components.map(id => {
      const definition = catalog.components.find(item => item.id === id);
      const source = state.sources[id];
      if (!definition || !["SOURCE", "PACKAGE"].includes(source)) throw new Error("组件或来源配置无效");
      return { ...definition, source };
    });
    const allSource = components.every(item => item.source === "SOURCE");
    const allPackage = components.every(item => item.source === "PACKAGE");
    const prefix = state.packagePrefix.trim();
    const reasons = [];
    if (!scenario.buildReady || components.some(item => !item.implemented))
      reasons.push("所选 integration 或组件仅为示意，尚未接入当前 CMake。");
    if (!allSource && !allPackage)
      reasons.push("此原型尚未生成逐组件命令；本分支 CMake 已支持真实组件的混合构建，详见建图文档。");
    if (!platform.buildReady)
      reasons.push("ARM64 工具链、sysroot 和依赖包尚未配置，也未做交叉编译验证。");
    if (!allSource && (!prefix.startsWith("/") || /[\r\n\0;]/.test(prefix)))
      reasons.push("请填写一个绝对安装前缀，不包含换行或分号；demo 只演示一个前缀。");

    const buildDirectory = "build/studio/" + platform.id + "/" +
      (allSource ? "source" : allPackage ? "package" : "mixed");
    const commands = [];
    if (reasons.length === 0) {
      const args = ["cmake -S . -B " + shellQuote(buildDirectory),
        "-DROBOFRAME_USE_INSTALLED_COMPONENTS=" + (allPackage ? "ON" : "OFF"),
        "-DBUILD_TESTING=" + (state.tests ? "ON" : "OFF")];
      if (allSource) args.push("-DSAMPLE_ACCUMULATOR_BUILD_DEMO=" + (state.demo ? "ON" : "OFF"));
      if (allPackage) args.push(shellQuote("-DCMAKE_PREFIX_PATH=" + prefix));
      commands.push(args.join(" \\\n  "));
      commands.push("cmake --build " + shellQuote(buildDirectory) + " -j2");
      if (state.tests) commands.push("ctest --test-dir " + shellQuote(buildDirectory) + " --output-on-failure");
    }
    return {
      components, reasons, commands, buildDirectory,
      hasPackages: !allSource, allPackage,
      sdkSource: allSource ? "SOURCE" : "PACKAGE",
      intent: {
        schemaVersion: catalog.schemaVersion, kind: "roboframe-build-intent-demo",
        scenario: scenario.id, platform: platform.id,
        sdkSource: allSource ? "SOURCE" : "PACKAGE",
        components: components.map(item => ({ id: item.id, source: item.source })),
        packagePrefix: allSource ? null : prefix,
        tests: Boolean(state.tests), componentDemo: allPackage ? false : Boolean(state.demo)
      }
    };
  }
  globalThis.BuildPlanner = { initialState, buildPlan, shellQuote };
})();
