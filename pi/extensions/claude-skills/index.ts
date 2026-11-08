/**
 * Auto-discover `.claude/skills/` and enabled installed Claude plugin skills for pi.
 *
 * Mirrors Claude Code's project-skills discovery described at
 * <https://code.claude.com/docs/en/skills>:
 *
 *   | Personal | ~/.claude/skills/<skill>/SKILL.md | All your projects |
 *   | Project  | .claude/skills/<skill>/SKILL.md   | This project only |
 *
 * Claude Code auto-discovers `.claude/skills/` from "parent and nested
 * directories". Pi's built-in skill discovery covers `.pi/skills/` and
 * `.agents/skills/` (walking up to the git repo root) but not
 * `.claude/skills/`. This extension fills that gap by emitting
 * `skillPaths` from `resources_discover` so pi loads them on startup
 * and `/reload`.
 *
 * Discovery
 *   1. Ancestor walk: from `cwd` up to the git repo root (or filesystem
 *      root if not in a repo), each `.claude/skills/` directory is
 *      added.
 *   2. Nested walk: descend up to NESTED_DEPTH levels under `cwd`,
 *      skipping common non-source directories (node_modules, .git,
 *      build, dist, etc.). Each `.claude/skills/` found is added.
 *      Gated on `cwd` being inside a git repo, since Claude Code's
 *      upstream nested discovery is itself a project/monorepo feature
 *      and running outside any repo (e.g. from `~/`) tends to slurp
 *      in unrelated worktrees and sibling repos.
 *   3. User-level (`~/.claude/skills/`) is intentionally skipped by
 *      default. Most setups wire it via pi's `settings.json` `skills`
 *      array; doubling up triggers name-collision warnings. Set
 *      `CLAUDE_SKILLS_INCLUDE_USER=1` to opt in.
 *
 *   4. Installed plugin skills: read the v2 Claude install registry and enabled
 *      plugin settings, following current install paths instead of scanning caches.
 *      Project skills/settings/installs require Pi project trust. Only skill text
 *      is imported: no hooks, agents, MCP servers, or executable plugin code.
 *
 * Environment overrides
 *   - `CLAUDE_SKILLS_INCLUDE_PLUGINS=0` — disable installed plugin skill imports.
 *   - `CLAUDE_SKILLS_INCLUDE_USER=1` — also include `~/.claude/skills/`.
 *   - `CLAUDE_SKILLS_NESTED_DEPTH=<n>` — override the nested-walk depth.
 *     Set to `0` to disable the nested walk entirely. Default: 4.
 *   - `CLAUDE_SKILLS_FORCE_NESTED=1` — run the nested walk even when
 *     `cwd` is not inside a git repo. Use sparingly.
 *
 * Pi handles loading: each emitted path is a directory, and pi walks
 * it for subdirectories containing `SKILL.md`.
 */

import * as fs from "node:fs";
import { homedir } from "node:os";
import * as path from "node:path";

import type { ExtensionAPI, ExtensionContext } from "@earendil-works/pi-coding-agent";

import { defaultCacheRoot, type NormalizedDir, normalizeSkillsDir } from "./normalize.ts";
import { discoverPluginSkills } from "./plugins.ts";
import { mirrorPluginSkills } from "./plugin-mirror.ts";

// ---------- Constants ----------

/**
 * Directory names skipped during the nested walk. Mirrors common
 * "don't descend here" conventions across language ecosystems.
 */
export const NESTED_SKIP = new Set<string>([
	".git",
	".hg",
	".svn",
	".direnv",
	".cache",
	".idea",
	".vscode",
	".next",
	".nuxt",
	".turbo",
	".gradle",
	".terraform",
	"node_modules",
	"bower_components",
	"vendor",
	"venv",
	".venv",
	"__pycache__",
	"dist",
	"build",
	"out",
	"target",
	"coverage",
]);

/** Default maximum depth (relative to cwd) for the nested-skills walk. */
export const NESTED_DEPTH = 4;

// ---------- Types ----------

export interface FoundSkillsDir {
	/** Absolute, real (symlink-resolved) path to the `.claude/skills` dir. */
	dir: string;
	/** Where this directory was found relative to cwd. */
	origin: "cwd" | "ancestor" | "nested" | "user";
	/** Number of immediate subdirs containing a SKILL.md. */
	skillCount: number;
}

export interface DiscoverOptions {
	cwd: string;
	/** When true, also include `~/.claude/skills/`. Default: false. */
	includeUser?: boolean;
	/**
	 * Maximum depth for the nested walk. `0` disables the nested walk
	 * entirely (overriding `forceNested`). Defaults to `NESTED_DEPTH`.
	 */
	nestedDepth?: number;
	/**
	 * When true, run the nested walk even when `cwd` is not inside a
	 * git repo. Defaults to false; the gate exists because nested
	 * discovery from e.g. `~/` tends to find unrelated worktrees and
	 * sibling repos. Ignored when `nestedDepth === 0`.
	 */
	forceNested?: boolean;
	/** Override `os.homedir()` (test hook). */
	homeDir?: string;
	/** Override the ancestor-walk stop directory (test hook). */
	findRoot?: (start: string) => string | null;
}

// ---------- Filesystem helpers ----------

function realpath(p: string): string {
	try {
		return fs.realpathSync(p);
	} catch {
		return path.resolve(p);
	}
}

function isDir(p: string): boolean {
	try {
		return fs.statSync(p).isDirectory();
	} catch {
		return false;
	}
}

function isFile(p: string): boolean {
	try {
		return fs.statSync(p).isFile();
	} catch {
		return false;
	}
}

/**
 * Walk upward from `start` looking for a `.git` entry. Returns the
 * directory containing it, or `null` if none is found before the
 * filesystem root.
 */
export function findGitRoot(start: string): string | null {
	let cur = path.resolve(start);
	while (true) {
		if (fs.existsSync(path.join(cur, ".git"))) return cur;
		const parent = path.dirname(cur);
		if (parent === cur) return null;
		cur = parent;
	}
}

/**
 * Count immediate child directories of `dir` that contain a top-level
 * `SKILL.md`. Used only for the friendly startup notification.
 */
export function countSkills(dir: string): number {
	let entries: fs.Dirent[];
	try {
		entries = fs.readdirSync(dir, { withFileTypes: true });
	} catch {
		return 0;
	}
	let count = 0;
	for (const entry of entries) {
		if (!entry.isDirectory()) continue;
		if (isFile(path.join(dir, entry.name, "SKILL.md"))) count += 1;
	}
	return count;
}

// ---------- Walks ----------

/**
 * Collect `.claude/skills` directories from `cwd` walking upward to
 * the stop directory (git root, or filesystem root when not in a
 * repo). The starting `cwd` is included in the walk.
 */
export function walkAncestors(cwd: string, stop: string | null): string[] {
	const found: string[] = [];
	let cur = path.resolve(cwd);
	while (true) {
		const candidate = path.join(cur, ".claude", "skills");
		if (isDir(candidate)) found.push(realpath(candidate));
		if (stop !== null && cur === stop) break;
		const parent = path.dirname(cur);
		if (parent === cur) break;
		cur = parent;
	}
	return found;
}

/**
 * Collect `.claude/skills` directories strictly below `cwd`. Recurses
 * into non-skipped, non-hidden subdirectories up to `maxDepth` levels
 * deep (defaults to `NESTED_DEPTH`). Symlink cycles are guarded via
 * realpath dedup. `maxDepth <= 0` returns immediately.
 */
export function walkNested(cwd: string, maxDepth: number = NESTED_DEPTH): string[] {
	const found: string[] = [];
	if (maxDepth <= 0) return found;
	const seen = new Set<string>();

	function walk(current: string, depth: number): void {
		if (depth > maxDepth) return;
		const real = realpath(current);
		if (seen.has(real)) return;
		seen.add(real);

		// Check this directory for a `.claude/skills` child.
		const candidate = path.join(real, ".claude", "skills");
		if (isDir(candidate)) found.push(realpath(candidate));

		let entries: fs.Dirent[];
		try {
			entries = fs.readdirSync(real, { withFileTypes: true });
		} catch {
			return;
		}
		for (const entry of entries) {
			if (!entry.isDirectory()) continue;
			if (NESTED_SKIP.has(entry.name)) continue;
			if (entry.name.startsWith(".")) continue; // skip hidden dirs (incl. `.claude`)
			walk(path.join(real, entry.name), depth + 1);
		}
	}

	// Start at cwd's children — cwd itself is covered by `walkAncestors`.
	let entries: fs.Dirent[];
	try {
		entries = fs.readdirSync(cwd, { withFileTypes: true });
	} catch {
		return found;
	}
	for (const entry of entries) {
		if (!entry.isDirectory()) continue;
		if (NESTED_SKIP.has(entry.name)) continue;
		if (entry.name.startsWith(".")) continue;
		walk(path.join(cwd, entry.name), 1);
	}
	return found;
}

// ---------- Discovery ----------

export function discoverSkillsDirs(opts: DiscoverOptions): FoundSkillsDir[] {
	const home = opts.homeDir ?? homedir();
	const homeSkills = realpath(path.join(home, ".claude", "skills"));
	const findRoot = opts.findRoot ?? findGitRoot;
	const stop = findRoot(opts.cwd);
	const nestedDepth = opts.nestedDepth ?? NESTED_DEPTH;
	const forceNested = opts.forceNested ?? false;

	const out: FoundSkillsDir[] = [];
	const seen = new Set<string>();

	function add(dir: string, origin: FoundSkillsDir["origin"]): void {
		const real = realpath(dir);
		if (seen.has(real)) return;
		// Skip user-home `.claude/skills` unless the caller opts in.
		if (real === homeSkills && origin !== "user") return;
		if (!isDir(real)) return;
		seen.add(real);
		out.push({ dir: real, origin, skillCount: countSkills(real) });
	}

	const startReal = realpath(opts.cwd);
	const ancestors = walkAncestors(opts.cwd, stop);
	for (const a of ancestors) {
		// First entry is cwd's own `.claude/skills` (if it exists).
		add(a, a === realpath(path.join(startReal, ".claude", "skills")) ? "cwd" : "ancestor");
	}

	// Gate the nested walk: only run when we're inside a git repo, so
	// running pi from `~/` (or any non-project dir) doesn't slurp in
	// every worktree and sibling repo below it. `forceNested` opts back
	// into the old eager behavior; `nestedDepth === 0` disables it
	// outright.
	const nestedAllowed = nestedDepth > 0 && (stop !== null || forceNested);
	if (nestedAllowed) {
		for (const n of walkNested(opts.cwd, nestedDepth)) {
			add(n, "nested");
		}
	}

	if (opts.includeUser && isDir(homeSkills)) {
		add(homeSkills, "user");
	}

	return out;
}

// ---------- Extension entry ----------

/**
 * Parse a non-negative integer env var. Returns `fallback` for unset,
 * empty, or invalid values. Negatives are clamped to 0.
 */
export function parseDepthEnv(raw: string | undefined, fallback: number): number {
	if (raw === undefined || raw === "") return fallback;
	const n = Number.parseInt(raw, 10);
	if (!Number.isFinite(n)) return fallback;
	return Math.max(0, n);
}

export default function claudeSkillsExtension(pi: ExtensionAPI) {
	const includeUser = process.env.CLAUDE_SKILLS_INCLUDE_USER === "1";
	const nestedDepth = parseDepthEnv(process.env.CLAUDE_SKILLS_NESTED_DEPTH, NESTED_DEPTH);
	const forceNested = process.env.CLAUDE_SKILLS_FORCE_NESTED === "1";
	const includePlugins = process.env.CLAUDE_SKILLS_INCLUDE_PLUGINS !== "0";

	pi.on("resources_discover", async (event, ctx: ExtensionContext) => {
		const projectTrusted = ctx.isProjectTrusted();
		const found = projectTrusted ? discoverSkillsDirs({
			cwd: event.cwd,
			includeUser,
			nestedDepth,
			forceNested,
		}) : includeUser ? [{ dir: path.join(homedir(), ".claude", "skills") }] : [];
		const projectDirs: string[] = [];
		if (projectTrusted) {
			const root = findGitRoot(event.cwd);
			let dir = path.resolve(event.cwd);
			while (dir !== homedir()) {
				projectDirs.unshift(dir);
				if (dir === root || dir === path.dirname(dir)) break;
				dir = path.dirname(dir);
			}
		}
		const plugins = includePlugins ? discoverPluginSkills({
			cwd: event.cwd, projectTrusted, projectDirs,
		}) : { sources: [], diagnostics: [] };

		// Mirror each found dir into a normalized cache so Claude-style
		// frontmatter (which often has unquoted `: ` in descriptions)
		// doesn't trip pi's strict YAML parser. See ./normalize.ts.
		const cacheRoot = defaultCacheRoot();
		const normalized: NormalizedDir[] = found.map((f) => normalizeSkillsDir(f.dir, cacheRoot));
		const mirrored = mirrorPluginSkills(plugins.sources, cacheRoot);

		if (ctx.hasUI) {
			for (const message of [...plugins.diagnostics, ...mirrored.diagnostics]) {
				ctx.ui.notify(`Claude skills: ${message}`, "warning");
			}
		}
		if (ctx.hasUI && event.reason === "startup" && (found.length || mirrored.count)) {
			const dirs = found.length + plugins.sources.length;
			const total = mirrored.count + normalized.reduce((acc, n) => acc + n.cleanCount + n.repairedCount, 0);
			const repaired = normalized.reduce((acc, n) => acc + n.repairedCount, 0);
			const dirWord = dirs === 1 ? "dir" : "dirs";
			const skillWord = total === 1 ? "skill" : "skills";
			let summary = `Imported ${total} Claude ${skillWord} from ${dirs} ${dirWord} (${mirrored.count} plugin skills)`;
			if (repaired > 0) {
				summary += ` (repaired YAML in ${repaired})`;
			}
			ctx.ui.notify(summary, "info");

			for (const n of normalized) {
				for (const u of n.unrepairable) {
					ctx.ui.notify(
						`Skipped Claude skill (frontmatter ${u.reason}): ${u.skillPath}`,
						"warning",
					);
				}
			}
		}

		return { skillPaths: [...normalized.map((n) => n.mirror), ...mirrored.skillPaths] };
	});
}
