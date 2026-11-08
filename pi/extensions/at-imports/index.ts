/**
 * @file imports for pi context files
 *
 * Mimics Claude Code's `@path/to/file` import behavior for pi's
 * AGENTS.md / CLAUDE.md context files. Pi ships context files verbatim,
 * so lines like `@~/synced-data/CLAUDE.md` are treated as plain text.
 * This extension expands them inline in the system prompt at the start
 * of every agent turn.
 *
 * Rules (kept close to Claude Code's memory imports):
 *   - `@` must be at start-of-line or preceded by whitespace (avoids
 *     matching email addresses, npm scopes like @foo/bar in prose, etc.)
 *   - The referenced path must start with `/`, `~/`, `./`, or `../` — a
 *     reasonable proxy for "this is really a filesystem path, not an
 *     @-mention".
 *   - If the file exists and is readable, the `@path` token is replaced
 *     with the file's contents.
 *   - If it doesn't exist, the token is left untouched.
 *   - Imports are expanded recursively, with cycle detection and a max
 *     depth of 5 (matches Claude Code).
 *   - Fenced code blocks (``` … ```) are skipped so code examples stay
 *     literal.
 *   - Results are cached by absolute-path + mtime, so we don't re-read
 *     files on every turn.
 */

import { readFileSync, statSync } from "node:fs";
import { homedir } from "node:os";
import { isAbsolute, resolve } from "node:path";
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

export const MAX_DEPTH = 5;

// Matches `@<path>` where `@` is at SOL or preceded by whitespace, and the
// path begins with `/`, `~/`, `./`, or `../` and continues with non-whitespace.
// The path chars exclude trailing punctuation we never want to capture.
export const IMPORT_RE = /(^|\s)@((?:~\/|\.\.?\/|\/)[^\s)`'"]+)/gm;

// Cache: absolute path -> { mtimeMs, content }
const fileCache = new Map<string, { mtimeMs: number; content: string }>();

/** Clear the file cache. Useful in tests. */
export function clearCache(): void {
	fileCache.clear();
}

function expandTilde(p: string): string {
	if (p === "~") return homedir();
	if (p.startsWith("~/")) return resolve(homedir(), p.slice(2));
	return p;
}

function resolvePath(p: string): string {
	const expanded = expandTilde(p);
	return isAbsolute(expanded) ? expanded : resolve(process.cwd(), expanded);
}

function readCached(absPath: string): string | null {
	let stat;
	try {
		stat = statSync(absPath);
	} catch {
		return null;
	}
	if (!stat.isFile()) return null;

	const cached = fileCache.get(absPath);
	if (cached && cached.mtimeMs === stat.mtimeMs) return cached.content;

	let content: string;
	try {
		content = readFileSync(absPath, "utf8");
	} catch {
		return null;
	}
	fileCache.set(absPath, { mtimeMs: stat.mtimeMs, content });
	return content;
}

/**
 * Expand `@path` imports in `text`. Splits on fenced code blocks (``` … ```)
 * so code examples are left literal. Recursive with cycle detection.
 *
 * Exported for testing; callers typically don't need to use this directly.
 */
export function expandImports(text: string, seen: Set<string> = new Set(), depth = 0): string {
	if (depth > MAX_DEPTH) return text;

	// Split on fenced code blocks, preserving fences.
	const parts = text.split(/(```[\s\S]*?```)/g);
	return parts
		.map((part) => {
			if (part.startsWith("```")) return part;
			return part.replace(IMPORT_RE, (match, lead: string, rawPath: string) => {
				const abs = resolvePath(rawPath);
				if (seen.has(abs)) {
					// Cycle — leave the original reference verbatim.
					return match;
				}
				const content = readCached(abs);
				if (content === null) return match;
				seen.add(abs);
				const expanded = expandImports(content, seen, depth + 1);
				seen.delete(abs);
				// Preserve the leading whitespace/anchor so surrounding layout stays intact.
				return `${lead}${expanded}`;
			});
		})
		.join("");
}

export default function (pi: ExtensionAPI) {
	pi.on("before_agent_start", (event) => {
		const current = event.systemPrompt;
		if (!current.includes("@")) return;
		const next = expandImports(current);
		if (next === current) return;
		return { systemPrompt: next };
	});
}
