/**
 * Normalize Claude-formatted SKILL.md frontmatter for pi.
 *
 * Pi parses SKILL.md frontmatter strictly via the `yaml` package. Skills
 * that Claude generates frequently embed `: ` (colon-space) inside
 * unquoted plain scalars — most often in `description:` lines like
 *
 *     description: Author a new ... test. Triggers on: 'add', 'fix'.
 *
 * YAML's plain-scalar rules forbid `: ` (it would otherwise be
 * ambiguous with a nested mapping), so the parser raises
 * "Nested mappings are not allowed in compact mappings" and pi drops
 * the skill with a `[Skill conflicts]` warning.
 *
 * Rather than ask users to babysit Claude's YAML, we mirror each
 * `.claude/skills/` directory under a per-process cache. Skills with
 * valid frontmatter become whole-subdir symlinks (so relative file
 * refs Just Work). Skills with simple `key: value` frontmatter are
 * normalized by single-quoting unquoted scalar values and writing the
 * fixed SKILL.md alongside per-entry symlinks for the rest of the
 * skill. Skills whose frontmatter is too structurally complex for the
 * line-based repair (nested mappings, anchors, block scalars, list
 * items) are dropped from the mirror with a diagnostic.
 *
 * Pi never sees the originals, so the strict YAML parse failure that
 * triggered all this never fires.
 */

import * as crypto from "node:crypto";
import * as fs from "node:fs";
import { tmpdir } from "node:os";
import * as path from "node:path";

// ---------- Types ----------

export interface NormalizedDir {
	/** Original `.claude/skills/` directory. */
	source: string;
	/** Mirror directory to emit to pi instead of `source`. */
	mirror: string;
	/** Skills that parsed cleanly and were whole-subdir symlinked. */
	cleanCount: number;
	/** Skills whose SKILL.md frontmatter was rewritten. */
	repairedCount: number;
	/** Skills dropped from the mirror (frontmatter too complex to auto-fix). */
	unrepairable: Array<{ skillPath: string; reason: string }>;
}

interface SkillSubdir {
	name: string;
	/** Absolute path to the skill directory inside `source`. */
	path: string;
	/** Absolute path to its SKILL.md file. */
	skillMd: string;
}

type ScanResult =
	| { kind: "clean" }
	| { kind: "needs-repair" }
	| { kind: "unrepairable"; reason: string };

// ---------- Frontmatter scan + repair ----------

/**
 * Classify a frontmatter YAML string as one of:
 * - `clean`: line-based scan finds no problems; pi will parse it as-is.
 * - `needs-repair`: simple `key: value` shape with at least one
 *   unquoted scalar that contains `: ` (colon-space). Quoting fixes it.
 * - `unrepairable`: indented continuations, nested mappings, list
 *   items, anchors, or block scalars. Out of scope for the line-based
 *   repair.
 *
 * The detector errs toward `unrepairable` when in doubt — false
 * negatives only mean we leave a real skill alone (pi still warns,
 * same as before this extension).
 */
export function scanFrontmatter(yamlStr: string): ScanResult {
	let needsRepair = false;

	for (const raw of yamlStr.split("\n")) {
		const line = raw.replace(/\s+$/, "");
		if (line === "") continue;
		if (line.startsWith("#")) continue;
		if (/^\s+\S/.test(line)) {
			return { kind: "unrepairable", reason: "indented continuation/nested mapping" };
		}
		if (line.startsWith("- ") || line === "-") {
			return { kind: "unrepairable", reason: "top-level list item" };
		}
		const m = line.match(/^([A-Za-z_][\w-]*):\s*(.*)$/);
		if (!m) {
			return { kind: "unrepairable", reason: `cannot parse line: ${truncate(line, 60)}` };
		}
		const value = m[2];
		if (value === "") continue;
		// Already-typed YAML scalars: leave alone, won't trip the parser.
		if (/^(true|false|null|~)$/i.test(value)) continue;
		if (/^-?\d+(\.\d+)?$/.test(value)) continue;
		// Quoted: trust it. (Unterminated quotes will still fail pi's
		// parser, but that's a true syntax error we can't auto-repair.)
		if (/^['"]/.test(value)) continue;
		// Flow scalars / anchors / block scalars: out of scope.
		if (/^[[{>|&*]/.test(value)) {
			return { kind: "unrepairable", reason: `complex scalar: ${truncate(value, 40)}` };
		}
		// Plain scalar. The parser-breaking case is `: ` (or `:\t`)
		// embedded in the value. Other plain-scalar restrictions
		// (leading `?`, `!`, etc.) are rare in skill frontmatter.
		if (/:[ \t]/.test(value)) {
			needsRepair = true;
		}
	}

	return needsRepair ? { kind: "needs-repair" } : { kind: "clean" };
}

/**
 * Re-emit a SKILL.md with each top-level unquoted scalar value
 * single-quoted. Embedded single quotes are escaped by doubling per
 * YAML's single-quoted-scalar rules. The body (everything after the
 * closing `---`) is preserved verbatim.
 *
 * Caller is responsible for passing only frontmatters classified as
 * `needs-repair` (or `clean`, if you want a no-op pass). The function
 * does not validate structure beyond what the line-based repair can
 * express, so feeding it `unrepairable` content may produce broken
 * output.
 */
export function repairSkillContent(content: string): string {
	const norm = content.replace(/\r\n/g, "\n").replace(/\r/g, "\n");
	if (!norm.startsWith("---\n")) return content;
	const end = norm.indexOf("\n---", 3);
	if (end === -1) return content;

	const yamlStr = norm.slice(4, end);
	const rest = norm.slice(end); // includes leading "\n---"

	const repaired: string[] = [];
	for (const raw of yamlStr.split("\n")) {
		const trimmedRight = raw.replace(/\s+$/, "");
		const m = trimmedRight.match(/^([A-Za-z_][\w-]*):\s*(.*)$/);
		if (!m) {
			repaired.push(raw);
			continue;
		}
		const key = m[1];
		const rawValue = m[2];
		if (
			rawValue === "" ||
			/^['"]/.test(rawValue) ||
			/^(true|false|null|~)$/i.test(rawValue) ||
			/^-?\d+(\.\d+)?$/.test(rawValue) ||
			/^[[{>|&*]/.test(rawValue)
		) {
			repaired.push(raw);
			continue;
		}
		const escaped = rawValue.replace(/'/g, "''");
		repaired.push(`${key}: '${escaped}'`);
	}

	return `---\n${repaired.join("\n")}${rest}`;
}

function truncate(s: string, n: number): string {
	return s.length <= n ? s : `${s.slice(0, n - 1)}…`;
}

// ---------- Filesystem helpers ----------

function listSkillSubdirs(dir: string): SkillSubdir[] {
	let entries: fs.Dirent[];
	try {
		entries = fs.readdirSync(dir, { withFileTypes: true });
	} catch {
		return [];
	}
	const out: SkillSubdir[] = [];
	for (const entry of entries) {
		if (!entry.isDirectory() && !entry.isSymbolicLink()) continue;
		const sub = path.join(dir, entry.name);
		const md = path.join(sub, "SKILL.md");
		try {
			if (fs.statSync(md).isFile()) {
				out.push({ name: entry.name, path: sub, skillMd: md });
			}
		} catch {
			// No SKILL.md — not a skill from our perspective.
		}
	}
	return out;
}

function ensureFreshDir(dir: string): void {
	try {
		fs.rmSync(dir, { recursive: true, force: true });
	} catch {
		// Best effort.
	}
	fs.mkdirSync(dir, { recursive: true });
}

function symlinkAbs(target: string, link: string): void {
	try {
		fs.symlinkSync(target, link);
	} catch (err) {
		const e = err as NodeJS.ErrnoException;
		if (e.code === "EEXIST") return;
		throw err;
	}
}

/**
 * Default cache root for mirrors. Scoped by pid so concurrent pi
 * processes (e.g. multiple worktrees) don't stomp on each other. Each
 * `resources_discover` call wipes and rebuilds the relevant subtrees,
 * so leftover dirs from crashed pis are bounded and cleared on next
 * reboot via the OS's tmp cleanup.
 */
export function defaultCacheRoot(): string {
	return path.join(tmpdir(), `pi-claude-skills-${process.pid}`);
}

// ---------- Main entry ----------

/**
 * Build a normalized mirror of `source`. The returned `mirror` is what
 * the extension should hand to pi via `skillPaths` — pi walks it the
 * same way it would walk `source`, but only sees skills that will
 * actually parse.
 */
export function normalizeSkillsDir(
	source: string,
	cacheRoot: string = defaultCacheRoot(),
): NormalizedDir {
	const hash = crypto.createHash("sha1").update(source).digest("hex").slice(0, 16);
	const mirror = path.join(cacheRoot, hash, "skills");
	ensureFreshDir(mirror);

	let cleanCount = 0;
	let repairedCount = 0;
	const unrepairable: Array<{ skillPath: string; reason: string }> = [];

	for (const skill of listSkillSubdirs(source)) {
		let content: string;
		try {
			content = fs.readFileSync(skill.skillMd, "utf-8");
		} catch {
			// Read failure: pass through so pi reports it via its own
			// `read_failed` diagnostic, instead of silently dropping.
			symlinkAbs(skill.path, path.join(mirror, skill.name));
			cleanCount += 1;
			continue;
		}

		const yamlStr = extractFrontmatter(content);
		if (yamlStr === null) {
			// No (or malformed-edge) frontmatter — pi's loader handles
			// the no-frontmatter case by treating it as empty, which
			// means "no description" and pi drops it. Same as today.
			symlinkAbs(skill.path, path.join(mirror, skill.name));
			cleanCount += 1;
			continue;
		}

		const scan = scanFrontmatter(yamlStr);
		if (scan.kind === "clean") {
			symlinkAbs(skill.path, path.join(mirror, skill.name));
			cleanCount += 1;
			continue;
		}
		if (scan.kind === "unrepairable") {
			unrepairable.push({ skillPath: skill.path, reason: scan.reason });
			continue;
		}

		// needs-repair → write a fixed SKILL.md and symlink the rest
		// of the skill so relative paths inside it still resolve.
		const fixed = repairSkillContent(content);
		const subMirror = path.join(mirror, skill.name);
		fs.mkdirSync(subMirror, { recursive: true });
		fs.writeFileSync(path.join(subMirror, "SKILL.md"), fixed);

		let entries: fs.Dirent[] = [];
		try {
			entries = fs.readdirSync(skill.path, { withFileTypes: true });
		} catch {
			// We have at least SKILL.md; that's enough for pi to load
			// the skill, even if other entries become inaccessible.
		}
		for (const entry of entries) {
			if (entry.name === "SKILL.md") continue;
			symlinkAbs(path.join(skill.path, entry.name), path.join(subMirror, entry.name));
		}
		repairedCount += 1;
	}

	return { source, mirror, cleanCount, repairedCount, unrepairable };
}

/**
 * Pull the YAML between leading `---\n` and the next `\n---` line.
 * Returns `null` when the document has no frontmatter (in which case
 * pi treats it as empty — we mirror that behavior unchanged).
 */
function extractFrontmatter(content: string): string | null {
	const norm = content.replace(/\r\n/g, "\n").replace(/\r/g, "\n");
	if (!norm.startsWith("---\n")) return null;
	const end = norm.indexOf("\n---", 3);
	if (end === -1) return null;
	return norm.slice(4, end);
}
