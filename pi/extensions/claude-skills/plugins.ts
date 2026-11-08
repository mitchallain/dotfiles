/** Read enabled Claude plugin skill paths. Never load hooks, MCPs, or plugin code. */
import * as fs from "node:fs";
import { homedir } from "node:os";
import * as path from "node:path";

type JsonObject = Record<string, unknown>;

export interface PluginSkillSource {
	dir: string;
	pluginRoot: string;
	pluginId: string;
}

export interface PluginDiscovery {
	sources: PluginSkillSource[];
	diagnostics: string[];
}

export interface PluginDiscoveryOptions {
	cwd: string;
	projectTrusted: boolean;
	homeDir?: string;
	/** Project config directories, ordered from repo root to cwd. */
	projectDirs?: string[];
}

function object(value: unknown): value is JsonObject {
	return value !== null && typeof value === "object" && !Array.isArray(value);
}

function within(root: string, target: string): boolean {
	const relative = path.relative(root, target);
	return relative === "" || (!relative.startsWith(`..${path.sep}`) && relative !== ".." && !path.isAbsolute(relative));
}

function readJson(file: string, diagnostics: string[]): JsonObject | undefined {
	try {
		const value: unknown = JSON.parse(fs.readFileSync(file, "utf8"));
		if (!object(value)) throw new Error("Expected a JSON object");
		return value;
	} catch (error) {
		if ((error as NodeJS.ErrnoException).code !== "ENOENT") {
			// Don't include parser messages: they may echo sensitive config values.
			diagnostics.push(`Cannot read JSON object: ${file}`);
		}
		return undefined;
	}
}

export function discoverPluginSkills(opts: PluginDiscoveryOptions): PluginDiscovery {
	const home = opts.homeDir ?? homedir();
	const diagnostics: string[] = [];
	const sources: PluginSkillSource[] = [];
	const result = { sources, diagnostics };
	const configDir = path.join(home, ".claude");
	const registryFile = path.join(configDir, "plugins", "installed_plugins.json");
	const registry = readJson(registryFile, diagnostics);
	if (!registry) return result;
	if (registry.version !== 2 || !object(registry.plugins)) {
		diagnostics.push(`Unsupported Claude plugin registry format: ${registryFile}`);
		return result;
	}

	// A malformed settings layer must not accidentally re-enable its plugins.
	const enabled = new Map<string, boolean>();
	const projectDirs = opts.projectTrusted ? (opts.projectDirs ?? [opts.cwd]).map((dir) => path.resolve(dir)) : [];
	const settingsFiles = [
		path.join(configDir, "settings.json"),
		...projectDirs.flatMap((dir) => [path.join(dir, ".claude", "settings.json"), path.join(dir, ".claude", "settings.local.json")]),
	];
	for (const file of [...new Set(settingsFiles)]) {
		const before = diagnostics.length;
		const settings = readJson(file, diagnostics);
		if (diagnostics.length !== before) return result;
		if (!settings || settings.enabledPlugins === undefined) continue;
		if (!object(settings.enabledPlugins)) {
			diagnostics.push(`Invalid enabledPlugins mapping: ${file}`);
			return result;
		}
		for (const [id, value] of Object.entries(settings.enabledPlugins)) {
			if (typeof value !== "boolean") diagnostics.push(`Ignoring non-boolean enabledPlugins entry: ${id}`);
			enabled.set(id, value === true);
		}
	}

	for (const [pluginId, entries] of Object.entries(registry.plugins).sort(([a], [b]) => a.localeCompare(b))) {
		// Pi already loads the personal learning-protocol skill via ~/.claude/skills.
		if (pluginId.startsWith("learning-protocol@")) continue;
		if (enabled.get(pluginId) !== true) continue;
		if (!Array.isArray(entries)) {
			diagnostics.push(`Invalid install records: ${pluginId}`);
			continue;
		}
		const installs = entries.filter((entry): entry is JsonObject => {
			if (!object(entry)) return false;
			if (entry.scope === "user") return true;
			return (entry.scope === "project" || entry.scope === "local") && opts.projectTrusted &&
				typeof entry.projectPath === "string" && projectDirs.includes(path.resolve(entry.projectPath));
		}).sort((a, b) => {
			const rank = (entry: JsonObject) => entry.scope === "local" ? 2 : entry.scope === "project" ? 1 : 0;
			return rank(b) - rank(a);
		});
		const install = installs[0];
		if (!install) continue;
		if (typeof install.installPath !== "string" || !path.isAbsolute(install.installPath)) {
			diagnostics.push(`Missing absolute installPath: ${pluginId}`);
			continue;
		}
		let pluginRoot: string;
		try {
			pluginRoot = fs.realpathSync(install.installPath);
			if (!fs.statSync(pluginRoot).isDirectory()) throw new Error("Not a directory");
		} catch {
			diagnostics.push(`Installed plugin directory is unavailable: ${pluginId}`);
			continue;
		}

		const before = diagnostics.length;
		const manifest = readJson(path.join(pluginRoot, ".claude-plugin", "plugin.json"), diagnostics);
		if (diagnostics.length !== before) continue;
		const custom = manifest?.skills;
		const candidates = ["./skills"];
		if (typeof custom === "string") candidates.push(custom);
		else if (Array.isArray(custom) && custom.every((value) => typeof value === "string")) candidates.push(...custom);
		else if (custom !== undefined) diagnostics.push(`Invalid skills paths in manifest: ${pluginId}`);

		for (const configured of candidates) {
			const expanded = configured.replace(/\$\{CLAUDE_PLUGIN_ROOT\}/g, pluginRoot);
			const candidate = path.resolve(pluginRoot, expanded);
			if (!within(pluginRoot, candidate) || /\$/.test(expanded)) {
				diagnostics.push(`Ignoring skill path outside plugin or with unresolved variables: ${pluginId}`);
				continue;
			}
			try {
				const dir = fs.realpathSync(candidate);
				if (!within(pluginRoot, dir)) {
					diagnostics.push(`Ignoring skill symlink outside plugin: ${pluginId}`);
					continue;
				}
				if (!fs.statSync(dir).isDirectory()) {
					diagnostics.push(`Skill path is not a directory: ${pluginId}`);
					continue;
				}
				// A collection covers any nested custom path; avoid duplicate imports.
				if (sources.some((source) => within(source.dir, dir))) continue;
				for (let i = sources.length - 1; i >= 0; i--) {
					if (within(dir, sources[i].dir)) sources.splice(i, 1);
				}
				sources.push({ dir, pluginRoot, pluginId });
			} catch (error) {
				if (configured !== "./skills" || (error as NodeJS.ErrnoException).code !== "ENOENT") {
					diagnostics.push(`Skill directory is unavailable: ${pluginId} (${configured})`);
				}
			}
		}
	}
	return result;
}
