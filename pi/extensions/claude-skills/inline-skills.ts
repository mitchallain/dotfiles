/**
 * Mid-prompt `/skill` autocomplete and inline skill expansion.
 *
 * Pi's built-in editor and `CombinedAutocompleteProvider` only trigger `/`
 * command completion at the very start of line 0, and `AgentSession` only
 * expands `/skill:<name>` when it is the first token of the prompt.
 *
 * This module brings Claude Code-style mid-prompt skill invocation to Pi:
 *   1. Typing `/` after whitespace (or at column 0 of line 2+) opens an
 *      autocomplete menu filtered to loaded skills.
 *   2. Skills match by exact name, prefix, hyphen-segment prefix (e.g.
 *      `/report` or `/dev-docs` for `acme-developer-docs`), or
 *      3+-char substring — avoiding false-positive subsequence matches
 *      on common root paths like `/tmp` or `/etc`.
 *   3. Submitting a prompt with mid-prompt `/skill:<name>` (or bare
 *      `/<skill-name>`) rewrites the prompt so Pi's native
 *      `_expandSkillCommand` injects the `<skill>` block and renders
 *      the collapsible `[skill]` badge in the transcript.
 */

import * as fs from "node:fs";
import * as path from "node:path";

import type { AutocompleteItem, AutocompleteProvider } from "@earendil-works/pi-tui";

// ---------- Types ----------

export interface SkillCommandSourceInfo {
	path?: string;
	source?: string;
	scope?: string;
	baseDir?: string;
}

export interface SkillCommandInfo {
	/** Command name from `pi.getCommands()`, e.g. `"skill:grill-me"` or `"grill-me"`. */
	name: string;
	description?: string;
	sourceInfo?: SkillCommandSourceInfo;
}

export interface TransformInlineSkillsOptions {
	/** Command names (without leading `/`) that belong to non-skill commands. */
	nonSkillCommands?: ReadonlySet<string>;
	/** Optional reader override for multi-skill expansion in tests. */
	readSkillFile?: (filePath: string) => string;
}

// ---------- Cursor context helpers ----------

const START_OF_MESSAGE_SLASH_RE = /^\s*\/[^\s]*$/;
const MID_PROMPT_SLASH_RE = /(?:^|\s)(\/[^\s/]*)$/;

/**
 * True when the cursor is inside a slash token at the very start of the
 * prompt (line 0, with only optional whitespace before `/`).
 */
export function isStartOfMessageSlash(
	lines: readonly string[],
	cursorLine: number,
	cursorCol: number,
): boolean {
	if (cursorLine !== 0) return false;
	const beforeCursor = (lines[0] ?? "").slice(0, cursorCol);
	return START_OF_MESSAGE_SLASH_RE.test(beforeCursor);
}

/**
 * Extract the leading-slash token (`"/"`, `"/grill"`, `"/skill:grill"`) before
 * the cursor when it is in a mid-prompt position (preceded by whitespace on
 * line 0 after other text, or at start-of-line / after whitespace on line 2+).
 *
 * Returns `null` for start-of-message slash commands (which Pi's built-in
 * provider handles) and for tokens containing a second `/` (e.g. `/tmp/foo`,
 * `a/b`, URLs).
 */
export function extractMidPromptSlashPrefix(
	lines: readonly string[],
	cursorLine: number,
	cursorCol: number,
): string | null {
	if (isStartOfMessageSlash(lines, cursorLine, cursorCol)) {
		return null;
	}
	const currentLine = lines[cursorLine] ?? "";
	const beforeCursor = currentLine.slice(0, cursorCol);
	const match = MID_PROMPT_SLASH_RE.exec(beforeCursor);
	return match?.[1] ?? null;
}

// ---------- Skill matching ----------

function bareSkillName(name: string): string {
	return name.startsWith("skill:") ? name.slice("skill:".length) : name;
}

function fullSkillCommandName(name: string): string {
	return name.startsWith("skill:") ? name : `skill:${name}`;
}

function formatSourceTag(sourceInfo?: SkillCommandSourceInfo): string | undefined {
	if (!sourceInfo || !sourceInfo.source || sourceInfo.source === "builtin") {
		return undefined;
	}
	const scopePrefix =
		sourceInfo.scope === "user" ? "u" : sourceInfo.scope === "project" ? "p" : "t";
	const source = sourceInfo.source.trim();
	if (source === "auto" || source === "local" || source === "cli") {
		return scopePrefix;
	}
	if (source.startsWith("npm:")) {
		return `${scopePrefix}:${source}`;
	}
	return scopePrefix;
}

function formatSkillDescription(
	description?: string,
	sourceInfo?: SkillCommandSourceInfo,
): string | undefined {
	const tag = formatSourceTag(sourceInfo);
	if (!tag) return description;
	return description ? `[${tag}] ${description}` : `[${tag}]`;
}

/**
 * Check whether `queryParts` match prefixes of ordered hyphen-separated
 * `segments` (e.g. `["dev", "docs"]` matches `["acme", "developer", "docs"]`).
 * Returns the index of the first matched segment, or `-1` if no match.
 */
function matchSegmentPrefixes(segments: readonly string[], queryParts: readonly string[]): number {
	if (queryParts.length === 0 || queryParts.length > segments.length) return -1;
	let segIdx = 0;
	let firstMatched = -1;
	for (const part of queryParts) {
		let found = false;
		while (segIdx < segments.length) {
			if (segments[segIdx].startsWith(part)) {
				if (firstMatched === -1) firstMatched = segIdx;
				segIdx += 1;
				found = true;
				break;
			}
			segIdx += 1;
		}
		if (!found) return -1;
	}
	return firstMatched;
}

/**
 * Score a skill's bare name against `query` (lower is better; `null` = no match).
 *
 * Uses prefix, hyphen-segment prefix, and 3+-char contiguous substring matching
 * rather than arbitrary single-letter subsequence matching so short root paths
 * like `/tmp` or `/etc` do not match random letters inside skill names.
 */
export function scoreSkillMatch(
	bareName: string,
	query: string,
	rawAfterSlash: string,
): number | null {
	const name = bareName.toLowerCase();
	if (query === "") return 0;

	// 1. Exact bare name match (`/notes` -> `notes`).
	if (name === query) return 0;

	// 2. Prefix of bare name (`/grill` -> `grill-me`).
	if (name.startsWith(query)) return 10 + name.length;

	// 3. Prefix of a hyphen-separated tail (`/report` or `/html-report` -> `acme-html-report`).
	const segments = name.split("-");
	for (let i = 1; i < segments.length; i++) {
		const tail = segments.slice(i).join("-");
		if (tail === query) return 15 + i * 2 + name.length;
		if (tail.startsWith(query)) return 20 + i * 2 + name.length;
	}

	// 4. Multi-segment prefix match (`/dev-docs` -> `acme-developer-docs`).
	if (query.includes("-")) {
		const queryParts = query.split("-").filter((p) => p.length > 0);
		if (queryParts.length > 1) {
			const firstIdx = matchSegmentPrefixes(segments, queryParts);
			if (firstIdx !== -1) {
				return 30 + firstIdx * 2 + name.length;
			}
		}
	}

	// 5. Contiguous substring match for queries of at least 3 chars.
	if (query.length >= 3) {
		const idx = name.indexOf(query);
		if (idx !== -1) {
			return 50 + idx * 2 + name.length;
		}
	}

	// 6. User is typing `/sk`, `/ski`, `/skil`, `/skill` on the way to `/skill:`.
	if (!rawAfterSlash.startsWith("skill:") && "skill:".startsWith(rawAfterSlash)) {
		return 100;
	}

	return null;
}

/**
 * Filter and rank `skills` for a mid-prompt `slashPrefix` (e.g. `"/"`, `"/grill"`,
 * `"/skill:grill"`).
 */
export function matchSkills(
	skills: readonly SkillCommandInfo[],
	slashPrefix: string,
): AutocompleteItem[] {
	if (!slashPrefix.startsWith("/")) return [];
	const rawAfterSlash = slashPrefix.slice(1).toLowerCase();
	const query = rawAfterSlash.startsWith("skill:")
		? rawAfterSlash.slice("skill:".length)
		: rawAfterSlash;

	// Unknown namespace prefix like `/foo:bar`.
	if (query.includes(":")) return [];

	const scored: Array<{ item: AutocompleteItem; score: number; index: number }> = [];
	for (let index = 0; index < skills.length; index++) {
		const skill = skills[index];
		const bare = bareSkillName(skill.name);
		const commandName = fullSkillCommandName(skill.name);
		const score = scoreSkillMatch(bare, query, rawAfterSlash);
		if (score === null) continue;
		const description = formatSkillDescription(skill.description, skill.sourceInfo);
		scored.push({
			item: {
				value: commandName,
				label: commandName,
				...(description ? { description } : {}),
			},
			score,
			index,
		});
	}

	scored.sort((a, b) => a.score - b.score || a.index - b.index);
	return scored.map((entry) => entry.item);
}

// ---------- AutocompleteProvider wrapper ----------

export function createInlineSkillAutocompleteProvider(
	current: AutocompleteProvider,
	getSkills: () => readonly SkillCommandInfo[],
): AutocompleteProvider {
	return {
		async getSuggestions(lines, cursorLine, cursorCol, options) {
			if (!options.force) {
				const prefix = extractMidPromptSlashPrefix(lines, cursorLine, cursorCol);
				if (prefix !== null) {
					if (options.signal.aborted) return null;
					const items = matchSkills(getSkills(), prefix);
					return options.signal.aborted || items.length === 0 ? null : { items, prefix };
				}
			}
			return current.getSuggestions(lines, cursorLine, cursorCol, options);
		},

		applyCompletion(lines, cursorLine, cursorCol, item, prefix) {
			const isMidPrompt =
				prefix.startsWith("/") &&
				!prefix.slice(1).includes("/") &&
				!isStartOfMessageSlash(lines, cursorLine, cursorCol);

			if (isMidPrompt) {
				const currentLine = lines[cursorLine] ?? "";
				const beforePrefix = currentLine.slice(0, cursorCol - prefix.length);
				const afterCursor = currentLine.slice(cursorCol);
				const suffix = afterCursor.startsWith(" ") ? "" : " ";
				const replacement = `/${item.value}${suffix}`;
				const newLine = `${beforePrefix}${replacement}${afterCursor}`;
				const newLines = [...lines];
				newLines[cursorLine] = newLine;
				return {
					lines: newLines,
					cursorLine,
					cursorCol: beforePrefix.length + replacement.length,
				};
			}

			return current.applyCompletion(lines, cursorLine, cursorCol, item, prefix);
		},

		shouldTriggerFileCompletion(lines, cursorLine, cursorCol) {
			return current.shouldTriggerFileCompletion?.(lines, cursorLine, cursorCol) ?? true;
		},
	};
}

// ---------- Input prompt expansion ----------

function stripYamlFrontmatter(content: string): string {
	const normalized = content.replace(/\r\n/g, "\n").replace(/\r/g, "\n");
	if (!normalized.startsWith("---\n")) return normalized.trim();
	const endIndex = normalized.indexOf("\n---", 3);
	if (endIndex === -1) return normalized.trim();
	return normalized.slice(endIndex + 4).trim();
}

function buildSkillBlock(skill: SkillCommandInfo, readSkillFile: (p: string) => string): string | null {
	const filePath = skill.sourceInfo?.path;
	if (!filePath) return null;
	const baseDir = path.dirname(filePath);
	const bare = bareSkillName(skill.name);
	try {
		const raw = readSkillFile(filePath);
		const body = stripYamlFrontmatter(raw);
		return `<skill name="${bare}" location="${filePath}">\nReferences are relative to ${baseDir}.\n\n${body}\n</skill>`;
	} catch {
		return null;
	}
}

const CODE_SPAN_OR_FENCE_RE = /(```[\s\S]*?```|`[^`\n]+`)/g;
const INLINE_SKILL_TOKEN_RE = /(^|\s)\/(skill:)?([a-z0-9-]+)(?=\s|$)/g;

/**
 * Transform a user prompt containing inline `/skill:<name>` or `/<skill-name>`
 * references into a leading `/skill:<first-skill> ...` prompt that Pi's native
 * `_expandSkillCommand` expands into a `<skill>` block.
 *
 * Returns `null` when no transformation is needed (e.g. no skill tokens, or a
 * single leading `/skill:<name>` that Pi already handles natively).
 */
export function transformPromptWithInlineSkills(
	text: string,
	skills: readonly SkillCommandInfo[],
	options: TransformInlineSkillsOptions = {},
): string | null {
	if (!text.includes("/") || text.startsWith("<skill ")) return null;

	const skillMap = new Map<string, SkillCommandInfo>();
	for (const skill of skills) {
		skillMap.set(bareSkillName(skill.name), skill);
	}
	if (skillMap.size === 0) return null;

	const nonSkillCommands = options.nonSkillCommands;
	const readSkillFile = options.readSkillFile ?? ((p: string) => fs.readFileSync(p, "utf8"));

	const matchedSkillNames: string[] = [];
	const recordSkill = (name: string) => {
		if (!matchedSkillNames.includes(name)) {
			matchedSkillNames.push(name);
		}
	};

	let sawMidPromptToken = false;
	let sawLeadingBareSkill = false;
	let sawLeadingNewlineAfterSkill = false;

	// Split on fenced code blocks and inline code spans so backticked `/skill:...`
	// or `/notes` references are left untouched.
	const parts = text.split(CODE_SPAN_OR_FENCE_RE);
	let charOffset = 0;

	const transformedParts = parts.map((part) => {
		if (part.startsWith("```") || part.startsWith("`")) {
			charOffset += part.length;
			return part;
		}
		const partStart = charOffset;
		charOffset += part.length;

		return part.replace(
			INLINE_SKILL_TOKEN_RE,
			(fullMatch, lead: string, skillPrefix: string | undefined, candidate: string, matchOffset: number) => {
				const skill = skillMap.get(candidate);
				if (!skill) return fullMatch;

				const tokenStartInText = partStart + matchOffset + lead.length;
				const isAtStartOfPrompt = text.slice(0, tokenStartInText).trim() === "";

				if (!skillPrefix && isAtStartOfPrompt && nonSkillCommands?.has(candidate)) {
					return fullMatch;
				}

				recordSkill(candidate);

				if (isAtStartOfPrompt) {
					if (!skillPrefix) sawLeadingBareSkill = true;
					const afterTokenIndex = tokenStartInText + 1 + (skillPrefix?.length ?? 0) + candidate.length;
					if (text[afterTokenIndex] === "\n" || text[afterTokenIndex] === "\r") {
						sawLeadingNewlineAfterSkill = true;
					}
					// Strip the leading token from the body; we'll re-prepend `/skill:<first>` below.
					return lead;
				}

				sawMidPromptToken = true;
				// Keep a clean `/<skill>` reference in place so the sentence reads naturally.
				return `${lead}/${candidate}`;
			},
		);
	});

	if (matchedSkillNames.length === 0) return null;

	// If the prompt already started with `/skill:<name> ` and had no other skill
	// tokens and wasn't followed directly by a newline, Pi's built-in
	// `_expandSkillCommand` handles it without transformation.
	if (
		!sawMidPromptToken &&
		!sawLeadingBareSkill &&
		!sawLeadingNewlineAfterSkill &&
		matchedSkillNames.length === 1
	) {
		return null;
	}

	const firstSkill = matchedSkillNames[0];
	let body = transformedParts.join("").trim();

	// If multiple distinct skills were referenced, Pi's `_expandSkillCommand`
	// expands the first one (`firstSkill`); expand any additional skills here.
	if (matchedSkillNames.length > 1) {
		const extraBlocks: string[] = [];
		for (const extraName of matchedSkillNames.slice(1)) {
			const extraSkill = skillMap.get(extraName);
			if (!extraSkill) continue;
			const block = buildSkillBlock(extraSkill, readSkillFile);
			if (block) extraBlocks.push(block);
		}
		if (extraBlocks.length > 0) {
			body = body ? `${extraBlocks.join("\n\n")}\n\n${body}` : extraBlocks.join("\n\n");
		}
	}

	return body ? `/skill:${firstSkill} ${body}` : `/skill:${firstSkill}`;
}
