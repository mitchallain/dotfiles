import assert from "node:assert/strict";
import { mkdtempSync, mkdirSync, readFileSync, rmSync, symlinkSync, writeFileSync } from "node:fs";
import { tmpdir } from "node:os";
import { dirname, join } from "node:path";
import { test } from "node:test";
import claudeSkillsExtension from "./index.ts";
import { discoverPluginSkills } from "./plugins.ts";
import { mirrorPluginSkills } from "./plugin-mirror.ts";

function fixture() {
	const root = mkdtempSync(join(tmpdir(), "pi-plugin-skills-test-"));
	const home = join(root, "home");
	const project = join(root, "project");
	const other = join(root, "other-project");
	mkdirSync(project, { recursive: true });
	const plugins: Record<string, unknown[]> = {};
	const enabled: Record<string, boolean> = {};
	function text(file: string, value: string) {
		mkdirSync(dirname(file), { recursive: true });
		writeFileSync(file, value);
	}
	function json(file: string, value: unknown) { text(file, JSON.stringify(value)); }
	function plugin(id: string, scope = "user", projectPath?: string, version = "current") {
		const installPath = join(home, ".claude", "plugins", "cache", id, version);
		const entry = { installPath, scope, projectPath };
		plugins[id] = [...(plugins[id] ?? []), entry];
		enabled[id] = true;
		json(join(installPath, ".claude-plugin", "plugin.json"), { name: id });
		text(join(installPath, "skills", "basic", "SKILL.md"), "---\nname: basic\ndescription: A test skill.\n---\nBody\n");
		return installPath;
	}
	function save() {
		json(join(home, ".claude", "settings.json"), { enabledPlugins: enabled });
		json(join(home, ".claude", "plugins", "installed_plugins.json"), { version: 2, plugins });
	}
	function discover(projectTrusted = false) {
		return discoverPluginSkills({ cwd: project, projectTrusted, homeDir: home, projectDirs: [project] });
	}
	return { root, home, project, other, enabled, plugins, plugin, save, discover, text, json,
		cleanup() { rmSync(root, { recursive: true, force: true }); } };
}

test("imports enabled registry installs only; does not scan stale/uninstalled caches", () => {
	const f = fixture();
	try {
		const current = f.plugin("enabled@market");
		f.plugin("disabled@market"); f.enabled["disabled@market"] = false;
		f.plugin("not-enabled@market"); delete f.enabled["not-enabled@market"];
		f.text(join(dirname(current), "stale", "skills", "obsolete", "SKILL.md"), "uninstalled");
		f.save();
		const result = f.discover();
		assert.equal(result.sources.length, 1);
		assert.equal(result.sources[0].dir, join(current, "skills"));
		assert.deepEqual(result.diagnostics, []);
	} finally { f.cleanup(); }
});

test("excludes the learning-protocol plugin only in Pi without changing Claude settings", () => {
	const f = fixture();
	try {
		f.plugin("learning-protocol@market");
		f.plugin("other@market");
		f.save();
		assert.deepEqual(f.discover().sources.map((s) => s.pluginId), ["other@market"]);
		const settings = JSON.parse(readFileSync(join(f.home, ".claude", "settings.json"), "utf8"));
		assert.equal(settings.enabledPlugins["learning-protocol@market"], true);
	} finally { f.cleanup(); }
});

test("scope precedence, project disables, and project settings gated on trust", () => {
	const f = fixture();
	try {
		const user = f.plugin("shared@market");
		f.plugin("shared@market", "project", f.project, "project");
		const local = f.plugin("shared@market", "local", f.project, "local");
		f.plugin("foreign@market", "project", f.other);
		f.plugin("unknown@market", "unknown");
		f.save();
		assert.deepEqual(f.discover().sources.map((s) => s.pluginRoot), [user]);
		assert.deepEqual(f.discover(true).sources.map((s) => s.pluginRoot), [local]);
		f.json(join(f.project, ".claude", "settings.local.json"), { enabledPlugins: { "shared@market": false } });
		assert.equal(f.discover(true).sources.length, 0);
		assert.equal(f.discover().sources.length, 1);
	} finally { f.cleanup(); }
});

test("ancestor project settings merge from root to cwd; local settings override shared", () => {
	const f = fixture();
	try {
		f.plugin("shared@market");
		f.save();
		const inner = join(f.project, "subdir");
		f.json(join(f.project, ".claude", "settings.json"), { enabledPlugins: { "shared@market": false } });
		f.json(join(inner, ".claude", "settings.local.json"), { enabledPlugins: { "shared@market": true } });
		const result = discoverPluginSkills({ cwd: inner, homeDir: f.home, projectTrusted: true, projectDirs: [f.project, inner] });
		assert.equal(result.sources.length, 1);
	} finally { f.cleanup(); }
});

test("manifest paths are additive, deduplicated, confined to the plugin; direct skills supported", () => {
	const f = fixture();
	try {
		const plugin = f.plugin("paths@market");
		f.text(join(plugin, "extra", "direct", "SKILL.md"), "---\nname: custom\ndescription: Custom.\n---\n");
		f.text(join(f.root, "outside", "SKILL.md"), "outside");
		symlinkSync(join(f.root, "outside"), join(plugin, "outside-link"));
		f.json(join(plugin, ".claude-plugin", "plugin.json"), {
			skills: ["./skills", "./skills/basic", "${CLAUDE_PLUGIN_ROOT}/extra/direct", "../escape", "./outside-link", "$UNSET/skills"],
		});
		f.save();
		const result = f.discover();
		assert.deepEqual(result.sources.map((s) => s.dir), [join(plugin, "skills"), join(plugin, "extra", "direct")]);
		assert.equal(result.diagnostics.length, 3);
	} finally { f.cleanup(); }
});

test("missing installs, corrupt JSON, unsupported registry versions, and invalid enable values", () => {
	const f = fixture();
	try {
		const plugin = f.plugin("missing@market");
		f.save();
		rmSync(plugin, { recursive: true });
		assert.equal(f.discover().sources.length, 0);
		assert.match(f.discover().diagnostics[0], /unavailable/);
		f.text(join(f.home, ".claude", "settings.json"), '{"sensitive": NOT_JSON}');
		assert.equal(f.discover().sources.length, 0);
		assert.ok(!f.discover().diagnostics.join().includes("sensitive"));
		f.json(join(f.home, ".claude", "plugins", "installed_plugins.json"), { version: 99, plugins: {} });
		assert.match(f.discover().diagnostics[0], /Unsupported/);
		f.plugin("invalid@market"); f.save();
		f.json(join(f.home, ".claude", "settings.json"), { enabledPlugins: { "invalid@market": "true" } });
		assert.equal(f.discover().sources.length, 0);
		assert.match(f.discover().diagnostics[0], /non-boolean/);
	} finally { f.cleanup(); }
});

test("mirror preserves multiline YAML, plugin-relative assets, and original files; expands plugin root", () => {
	const f = fixture();
	try {
		const plugin = f.plugin("mirror@market");
		const original = '---\nname: basic\ndescription: >\n  A multiline description.\nmetadata:\n  category: documentation\n---\nRun ${CLAUDE_PLUGIN_ROOT}/scripts/helper.sh\nRead ../../references/shared.md\n';
		const skill = join(plugin, "skills", "basic", "SKILL.md");
		f.text(skill, original);
		f.text(join(plugin, "references", "shared.md"), "shared reference");
		f.text(join(plugin, "scripts", "helper.sh"), "echo helper");
		f.text(join(plugin, "skills", "basic", "asset.txt"), "local asset");
		f.save();
		const result = mirrorPluginSkills(f.discover().sources, join(f.root, "cache"));
		assert.equal(result.count, 1);
		assert.deepEqual(result.diagnostics, []);
		const dir = join(result.skillPaths[0], "basic");
		const mirrored = readFileSync(join(dir, "SKILL.md"), "utf8");
		assert.equal(mirrored, original.replaceAll("${CLAUDE_PLUGIN_ROOT}", plugin));
		assert.equal(readFileSync(skill, "utf8"), original);
		assert.equal(readFileSync(join(dir, "../../references/shared.md"), "utf8"), "shared reference");
		assert.equal(readFileSync(join(dir, "asset.txt"), "utf8"), "local asset");
		assert.equal(mirrorPluginSkills(f.discover().sources, join(f.root, "cache")).count, 1);
	} finally { f.cleanup(); }
});

test("repairs simple unquoted colon descriptions; invalid skills do not block valid siblings", () => {
	const f = fixture();
	try {
		const plugin = f.plugin("yaml@market");
		f.text(join(plugin, "skills", "repaired", "SKILL.md"), "---\nname: repaired\ndescription: Triggers on: help.\n---\nBody\n");
		f.text(join(plugin, "skills", "invalid", "SKILL.md"), "---\nname: broken\ndescription: [\n---\nBody\n");
		f.text(join(plugin, "skills", "nested", "deep", "SKILL.md"), "---\nname: deep\ndescription: Deep.\n---\n");
		f.save();
		const result = mirrorPluginSkills(f.discover().sources, join(f.root, "cache"));
		assert.equal(result.count, 3);
		assert.equal(result.diagnostics.length, 1);
		assert.match(readFileSync(join(result.skillPaths[0], "repaired", "SKILL.md"), "utf8"), /description: 'Triggers on: help\.'/);
		assert.equal(readFileSync(join(result.skillPaths[0], "invalid", "SKILL.md"), "utf8"), "");
	} finally { f.cleanup(); }
});

test("discovery follows registry updates and plugin disables on the next reload", () => {
	const f = fixture();
	try {
		const original = f.plugin("updated@market");
		f.save();
		assert.equal(f.discover().sources[0].pluginRoot, original);
		const updated = f.plugin("updated@market", "user", undefined, "updated");
		f.plugins["updated@market"] = f.plugins["updated@market"].slice(-1);
		f.save();
		assert.equal(f.discover().sources[0].pluginRoot, updated);
		f.enabled["updated@market"] = false;
		f.save();
		assert.equal(f.discover().sources.length, 0);
	} finally { f.cleanup(); }
});

test("broken skill symlinks do not block valid siblings", () => {
	const f = fixture();
	try {
		const plugin = f.plugin("symlinks@market");
		symlinkSync(join(f.root, "missing"), join(plugin, "skills", "broken"));
		f.save();
		const result = mirrorPluginSkills(f.discover().sources, join(f.root, "cache"));
		assert.equal(result.count, 1);
		assert.equal(result.diagnostics.length, 1);
	} finally { f.cleanup(); }
});

test("resource handler registers only discovery; plugin opt-out yields no project paths without trust", async () => {
	const previous = process.env.CLAUDE_SKILLS_INCLUDE_PLUGINS;
	process.env.CLAUDE_SKILLS_INCLUDE_PLUGINS = "0";
	try {
		const handlers = new Map();
		claudeSkillsExtension({ on(event, handler) { handlers.set(event, handler); } } as any);
		assert.deepEqual([...handlers.keys()], ["resources_discover"]);
		const result = await handlers.get("resources_discover")({ cwd: "/untrusted", reason: "reload" }, {
			isProjectTrusted() { return false; }, hasUI: false,
		});
		assert.deepEqual(result.skillPaths, []);
	} finally {
		if (previous === undefined) delete process.env.CLAUDE_SKILLS_INCLUDE_PLUGINS;
		else process.env.CLAUDE_SKILLS_INCLUDE_PLUGINS = previous;
	}
});
