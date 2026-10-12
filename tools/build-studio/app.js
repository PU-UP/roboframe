"use strict";
const catalog = BuildCatalog;
const { initialState, buildPlan } = BuildPlanner;
const byId = id => document.getElementById(id);
let state = initialState(catalog);
let plan;

function option(value, text) {
  const item = document.createElement("option");
  item.value = value;
  item.textContent = text;
  return item;
}
for (const item of catalog.scenarios) byId("scenario").append(option(item.id, item.name));
for (const item of catalog.platforms) byId("platform").append(option(item.id, item.name));

function renderComponents() {
  byId("components").replaceChildren();
  const scenario = catalog.scenarios.find(item => item.id === state.scenario);
  for (const id of scenario.components) {
    const item = catalog.components.find(component => component.id === id);
    const row = document.createElement("article");
    row.className = "component";
    row.dataset.component = id;
    const description = document.createElement("div");
    const title = document.createElement("h3");
    title.textContent = item.name;
    if (!item.implemented) {
      const badge = document.createElement("span");
      badge.className = "badge";
      badge.textContent = "未实现";
      title.append(badge);
    }
    const note = document.createElement("p");
    note.textContent = item.description;
    const target = document.createElement("code");
    target.textContent = item.target;
    description.append(title, note, target);
    const label = document.createElement("label");
    label.textContent = "来源";
    const select = document.createElement("select");
    select.setAttribute("aria-label", item.name + " 来源");
    select.append(option("SOURCE", "源码"), option("PACKAGE", "安装包"));
    select.value = state.sources[id];
    select.addEventListener("change", () => { state.sources[id] = select.value; update(); });
    label.append(select);
    row.append(description, label);
    byId("components").append(row);
  }
}
function update() {
  plan = buildPlan(catalog, state);
  const scenario = catalog.scenarios.find(item => item.id === state.scenario);
  const platform = catalog.platforms.find(item => item.id === state.platform);
  const previewOnly = plan.reasons.length > 0;
  byId("scenario").value = state.scenario;
  byId("platform").value = state.platform;
  byId("platform-note").textContent = platform.detail;
  byId("integration").textContent = scenario.integration;
  byId("prefix-field").hidden = !plan.hasPackages;
  byId("prefix").value = state.packagePrefix;
  byId("tests").checked = state.tests;
  byId("sdk-source").textContent = plan.sdkSource === "SOURCE" ? "本地源码" : "同一套 SDK 安装包";
  byId("status").textContent = previewOnly ? "仅设计预览" : "现有 CMake 命令";
  byId("status").classList.toggle("future", previewOnly);
  byId("notice").classList.toggle("future", previewOnly);
  byId("notice").replaceChildren();
  if (previewOnly) {
    const list = document.createElement("ul");
    for (const reason of plan.reasons) {
      const item = document.createElement("li");
      item.textContent = reason;
      list.append(item);
    }
    byId("notice").append(list);
  } else {
    byId("notice").textContent = plan.allPackage ?
      "请提供 SDK 5.0 与五个配套组件开发包；页面不检查包是否存在或兼容。" :
      "命令按各组件来源生成。在 WSL / Linux 的 roboframe 根目录执行；页面不执行构建，混合模式需要 SDK 安装包。";
  }
  byId("preview-label").textContent = previewOnly ? "待框架支持后再接入执行" : "在 roboframe 根目录运行";
  byId("commands").textContent = previewOnly ?
    "已记录所选来源与目标平台。\n\n当前不生成尚未验证的 ARM 构建命令。\n可导出配置意图，用于后续设计讨论。" : plan.commands.join("\n\n");
  byId("copy").disabled = previewOnly;
  byId("intent").textContent = JSON.stringify(plan.intent, null, 2);
  byId("feedback").textContent = "";
  for (const row of byId("components").children) row.dataset.source = state.sources[row.dataset.component];
}
byId("scenario").addEventListener("change", event => {
  state = initialState(catalog, event.target.value);
  renderComponents();
  update();
});
byId("platform").addEventListener("change", event => {
  const previousDefault = "/opt/roboframe-deps/" + state.platform;
  state.platform = event.target.value;
  if (state.packagePrefix === previousDefault)
    state.packagePrefix = "/opt/roboframe-deps/" + state.platform;
  update();
});
byId("prefix").addEventListener("input", event => { state.packagePrefix = event.target.value; update(); });
byId("tests").addEventListener("change", event => { state.tests = event.target.checked; update(); });
byId("reset").addEventListener("click", () => {
  state = initialState(catalog);
  renderComponents();
  update();
});
byId("copy").addEventListener("click", async () => {
  try {
    await navigator.clipboard.writeText(plan.commands.join("\n\n"));
    byId("feedback").textContent = "已复制构建命令。尚未执行。";
  } catch {
    const selection = window.getSelection();
    const range = document.createRange();
    range.selectNodeContents(byId("commands"));
    selection.removeAllRanges();
    selection.addRange(range);
    byId("feedback").textContent = "浏览器未允许自动复制，已选中命令，请手动复制。";
  }
});
byId("download").addEventListener("click", () => {
  const blob = new Blob([JSON.stringify(plan.intent, null, 2) + "\n"], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = "roboframe-build-intent.json";
  link.click();
  setTimeout(() => URL.revokeObjectURL(url), 1000);
  byId("feedback").textContent = "已请求下载配置意图。它不是可直接交给 CMake 的配置文件。";
});
renderComponents();
update();
