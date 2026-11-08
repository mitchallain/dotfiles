/** Mirror plugin skills without altering installed plugins or losing relative assets. */
import { createHash } from "node:crypto";
import * as fs from "node:fs";
import * as path from "node:path";
import { parseDocument } from "yaml";
import { repairSkillContent, scanFrontmatter } from "./normalize.ts";
import type { PluginSkillSource } from "./plugins.ts";

export interface MirroredPluginSkills {
	skillPaths: string[];
	count: number;
	diagnostics: string[];
}

function within(root: string, target: string): boolean {
	const relative = path.relative(root, target);
	return relative === "" || (!relative.startsWith(`..${path.sep}`) && relative !== ".." && !path.isAbsolute(relative));
}

function validFrontmatter(content: string): boolean {
	const match = /^---\r?\n([\s\S]*?)\r?\n---(?:\r?\n|$)/.exec(content);
	if (!match) return false;
	const document = parseDocument(match[1]);
	if (document.errors.length) return false;
	try {
		const value = document.toJS();
		return value !== null && typeof value === "object" && !Array.isArray(value) &&
			typeof value.description === "string" && value.description.trim().length > 0;
	} catch {
		return false;
	}
}

export function mirrorPluginSkills(sources: PluginSkillSource[], cacheRoot: string): MirroredPluginSkills {
	const result: MirroredPluginSkills = { skillPaths: [], count: 0, diagnostics: [] };
	const groups = new Map<string, PluginSkillSource[]>();
	for (const source of sources) {
		const group = groups.get(source.pluginRoot) ?? [];
		group.push(source);
		groups.set(source.pluginRoot, group);
	}
	for (const [root, pluginSources] of groups) {
		const edited = new Map<string, string>();
		const seen = new Set<string>();
		for (const source of pluginSources) {
			function scan(dir: string): void {
				const real = fs.realpathSync(dir);
				if (!within(root, real) || seen.has(real)) return;
				seen.add(real);
				const skill = path.join(dir, "SKILL.md");
				if (fs.existsSync(skill)) {
					if (!within(root, fs.realpathSync(skill))) {
						result.diagnostics.push(`Skipping skill symlink outside plugin: ${source.pluginId}`);
						return;
					}
					let content = fs.readFileSync(skill, "utf8");
					if (!validFrontmatter(content)) {
						const yaml = /^---\r?\n([\s\S]*?)\r?\n---(?:\r?\n|$)/.exec(content)?.[1];
						if (yaml && scanFrontmatter(yaml).kind === "needs-repair") content = repairSkillContent(content);
					}
					if (!validFrontmatter(content)) {
						result.diagnostics.push(`Skipping invalid skill frontmatter: ${source.pluginId} (${path.relative(root, skill)})`);
						// Blank the invalid skill in the mirror; Pi won't advertise it.
						edited.set(skill, "");
						return;
					}
					// Resolve per-plugin variables in text, not in the process environment.
					content = content.replace(/\$\{CLAUDE_PLUGIN_ROOT\}|\$CLAUDE_PLUGIN_ROOT\b/g, root);
					edited.set(skill, content);
					result.count++;
					return;
				}
				for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
					if (entry.name.startsWith(".") || entry.name === "node_modules") continue;
					const child = path.join(dir, entry.name);
					if (entry.isDirectory() || entry.isSymbolicLink()) {
						try {
							if (fs.statSync(child).isDirectory()) scan(child);
						} catch {
							result.diagnostics.push(`Cannot scan skill directory: ${source.pluginId} (${entry.name})`);
						}
					}
				}
			}
			try { scan(source.dir); }
			catch { result.diagnostics.push(`Cannot scan plugin skills: ${source.pluginId}`); }
		}
		if (!edited.size) continue;
		const hash = createHash("sha256").update(root).digest("hex").slice(0, 16);
		const mirror = path.join(cacheRoot, "plugins", hash);
		fs.rmSync(mirror, { recursive: true, force: true });
		// Rebuild only ancestors of SKILL.md files. Symlink the rest of the plugin,
		// so ../references and ../../scripts retain their original layout.
		function copy(dir: string, target: string): void {
			fs.mkdirSync(target, { recursive: true });
			for (const entry of fs.readdirSync(dir, { withFileTypes: true })) {
				const source = path.join(dir, entry.name);
				const dest = path.join(target, entry.name);
				if (edited.has(source)) {
					fs.writeFileSync(dest, edited.get(source)!, { mode: 0o600 });
				} else if ([...edited.keys()].some((skill) => within(source, skill))) {
					copy(source, dest);
				} else {
					fs.symlinkSync(source, dest);
				}
			}
		}
		copy(root, mirror);
		for (const source of pluginSources) {
			result.skillPaths.push(path.join(mirror, path.relative(root, source.dir)));
		}
	}
	return result;
}
