/**
 * claude-local — load repo-local `CLAUDE.local.md` files into the prompt
 *
 * Mirrors Claude Code's `CLAUDE.local.md` behavior: a project-local,
 * usually-gitignored file holding personal overrides that layer on top of
 * the committed `CLAUDE.md`. Pi only auto-loads `AGENTS.md` / `CLAUDE.md`,
 * so without this the local file never reaches the model.
 *
 * Discovery
 *   - Walk from `cwd` up to (and including) the git repo root, collecting
 *     every `CLAUDE.local.md` found along the way.
 *   - Ordered outermost-first (repo root) → cwd last, so the most-specific
 *     file appears last and wins, matching how pi layers `CLAUDE.md`.
 *   - If `cwd` is not inside a git repo, only `cwd` itself is checked (we
 *     never walk up to `~` and slurp an unrelated personal file).
 *
 * Imports
 *   - `@path` imports inside a `CLAUDE.local.md` are expanded inline via
 *     the sibling `at-imports` extension's `expandImports`. We do this
 *     ourselves rather than relying on `at-imports`' own pass: it runs
 *     before this extension (alphabetical discovery order) and chains off
 *     `event.systemPrompt`, so it never sees text appended here.
 *
 * Runtime behavior
 *   - Files are discovered on `session_start` (covers startup and
 *     `/reload`, which re-fires `session_start` with `reason: "reload"`).
 *   - On every `before_agent_start` the discovered files are (re)read with
 *     an mtime cache and appended under a `## Local Instructions` heading,
 *     so mid-session edits are picked up on the next turn.
 */

import * as fs from "node:fs";
import * as path from "node:path";

import type { ExtensionAPI, ExtensionContext } from "@earendil-works/pi-coding-agent";

import { expandImports } from "../at-imports/index.ts";

export const FILENAME = "CLAUDE.local.md";

// ---------- Types ----------

export interface LocalSource {
	/** Absolute path to the `CLAUDE.local.md` file. */
	sourcePath: string;
	/** Human-friendly label, e.g. "CLAUDE.local.md" or "sub/CLAUDE.local.md". */
	label: string;
}

export interface LocalFile extends LocalSource {
	/** File contents with `@imports` expanded, trimmed. */
	body: string;
}

// ---------- Filesystem helpers ----------

function isFile(p: string): boolean {
	try {
		return fs.statSync(p).isFile();
	} catch {
		return false;
	}
}

function realpath(p: string): string {
	try {
		return fs.realpathSync(p);
	} catch {
		return path.resolve(p);
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

// ---------- mtime cache for the top-level files ----------

// Cache: absolute path -> { mtimeMs, raw }. We cache the *raw* file read;
// `@import` expansion runs every turn so changes to imported files are
// reflected too (at-imports keeps its own mtime cache for those).
const rawCache = new Map<string, { mtimeMs: number; raw: string }>();

/** Clear the raw-file cache. Useful in tests. */
export function clearCache(): void {
	rawCache.clear();
}

function readRaw(absPath: string): string | null {
	let stat: fs.Stats;
	try {
		stat = fs.statSync(absPath);
	} catch {
		return null;
	}
	if (!stat.isFile()) return null;

	const cached = rawCache.get(absPath);
	if (cached && cached.mtimeMs === stat.mtimeMs) return cached.raw;

	let raw: string;
	try {
		raw = fs.readFileSync(absPath, "utf8");
	} catch {
		return null;
	}
	rawCache.set(absPath, { mtimeMs: stat.mtimeMs, raw });
	return raw;
}

// ---------- Discovery ----------

export interface DiscoverOptions {
	cwd: string;
	/** Override the git-root lookup (test hook). */
	findRoot?: (start: string) => string | null;
}

/**
 * Discover `CLAUDE.local.md` files from `cwd` up to the git repo root.
 * Returns sources ordered outermost-first (root) → cwd last, deduped by
 * real path. When `cwd` is not in a git repo, only `cwd` is checked.
 */
export function discoverLocalFiles(opts: DiscoverOptions): LocalSource[] {
	const findRoot = opts.findRoot ?? findGitRoot;
	const cwd = path.resolve(opts.cwd);
	const root = findRoot(cwd);

	// Collect candidate directories from cwd upward to the root.
	const dirs: string[] = [];
	let cur = cwd;
	while (true) {
		dirs.push(cur);
		if (root === null) break; // not in a repo: only check cwd
		if (cur === root) break;
		const parent = path.dirname(cur);
		if (parent === cur) break;
		cur = parent;
	}
	// Outermost-first so the most-specific (cwd) file lands last.
	dirs.reverse();

	const labelBase = root ?? cwd;
	const out: LocalSource[] = [];
	const seen = new Set<string>();
	for (const dir of dirs) {
		const candidate = path.join(dir, FILENAME);
		if (!isFile(candidate)) continue;
		const real = realpath(candidate);
		if (seen.has(real)) continue;
		seen.add(real);
		const rel = path.relative(labelBase, candidate);
		const label = rel && !rel.startsWith("..") ? rel.split(path.sep).join("/") : FILENAME;
		out.push({ sourcePath: candidate, label });
	}
	return out;
}

// ---------- Reading + import expansion ----------

/**
 * Read a single `CLAUDE.local.md` and expand its `@imports`. Returns
 * `null` if the file is missing/unreadable or its (expanded) body is empty.
 */
export function loadLocalFile(source: LocalSource): LocalFile | null {
	const raw = readRaw(source.sourcePath);
	if (raw === null) return null;
	const body = expandImports(raw).trim();
	if (body === "") return null;
	return { ...source, body };
}

// ---------- System prompt assembly ----------

/** Build the `## Local Instructions` section appended to the system prompt. */
export function buildLocalSection(files: LocalFile[]): string {
	if (files.length === 0) return "";
	const sections: string[] = [
		"## Local Instructions",
		"The following project-local instructions were loaded from `CLAUDE.local.md` " +
			"files (personal, usually-gitignored overrides; outermost first, most " +
			"specific last):",
	];
	for (const file of files) {
		sections.push(`### ${file.label}\n\n${file.body}`);
	}
	return sections.join("\n\n");
}

// ---------- Extension entry point ----------

export default function claudeLocalExtension(pi: ExtensionAPI) {
	let sources: LocalSource[] = [];

	pi.on("session_start", async (_event, ctx: ExtensionContext) => {
		sources = discoverLocalFiles({ cwd: ctx.cwd });
		if (sources.length > 0 && ctx.hasUI) {
			const word = sources.length === 1 ? "file" : "files";
			ctx.ui.notify(`Loaded ${sources.length} CLAUDE.local.md ${word}`, "info");
		}
	});

	pi.on("before_agent_start", async (event) => {
		if (sources.length === 0) return;
		const files = sources
			.map((s) => loadLocalFile(s))
			.filter((f): f is LocalFile => f !== null);
		const section = buildLocalSection(files);
		if (!section) return;
		return { systemPrompt: `${event.systemPrompt}\n\n${section}\n` };
	});
}
