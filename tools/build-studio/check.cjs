// Optional developer check: node tools/build-studio/check.cjs (no npm dependencies).
const assert = require("node:assert/strict");
require("./catalog.js");
require("./planner.js");
const catalog = globalThis.BuildCatalog;
const { initialState, buildPlan, shellQuote } = globalThis.BuildPlanner;
let state = initialState(catalog);
let plan = buildPlan(catalog, state);
assert.equal(plan.reasons.length, 0);
assert.match(plan.commands[0], /USE_INSTALLED_COMPONENTS=OFF/);
assert.equal(plan.commands.length, 3);

state.tests = false;
state.demo = false;
plan = buildPlan(catalog, state);
assert.equal(plan.commands.length, 2);
assert.match(plan.commands[0], /BUILD_TESTING=OFF/);
assert.match(plan.commands[0], /BUILD_DEMO=OFF/);

state.sources["sample-accumulator"] = "PACKAGE";
plan = buildPlan(catalog, state);
assert.equal(plan.reasons.length, 0);
assert.match(plan.commands[0], /USE_INSTALLED_COMPONENTS=ON/);
assert.doesNotMatch(plan.commands[0], /BUILD_DEMO/);
assert.equal(plan.intent.componentDemo, false);
assert.equal(plan.sdkSource, "PACKAGE");
assert.equal(shellQuote("a'b"), "'a'\"'\"'b'");

for (const prefix of ["relative/path", "/one;/two", "/one\n/two"]) {
  state.packagePrefix = prefix;
  assert.equal(buildPlan(catalog, state).commands.length, 0);
}
state = initialState(catalog, "navigation");
plan = buildPlan(catalog, state);
assert.equal(plan.commands.length, 0);
assert(plan.reasons.some(reason => reason.includes("混合")));
assert.deepEqual(plan.intent.components.map(item => item.source), ["SOURCE", "PACKAGE", "PACKAGE"]);
state.platform = "linux-aarch64";
assert(buildPlan(catalog, state).reasons.some(reason => reason.includes("ARM64")));
state = initialState(catalog);
state.platform = "linux-aarch64";
assert.equal(buildPlan(catalog, state).commands.length, 0);
state = initialState(catalog);
state.sources["sample-accumulator"] = "AUTO";
assert.throws(() => buildPlan(catalog, state), /无效/);

// A new catalog entry can appear in a scenario without editing UI or planner code.
const extended = JSON.parse(JSON.stringify(catalog));
extended.components.push({ id: "qr-detector", name: "QR", implemented: false });
extended.scenarios.push({
  id: "qr", components: ["qr-detector"], defaults: ["PACKAGE"], buildReady: false
});
plan = buildPlan(extended, initialState(extended, "qr"));
assert.equal(plan.components[0].id, "qr-detector");
assert.equal(plan.commands.length, 0);
console.log("PASS: source/package commands, test switches, quoting, preview guards and catalog extension");
