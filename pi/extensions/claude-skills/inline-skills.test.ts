import assert from "node:assert/strict";
import { test } from "node:test";

import {
	createInlineSkillAutocompleteProvider,
	extractMidPromptSlashPrefix,
	isStartOfMessageSlash,
	matchSkills,
	type SkillCommandInfo,
	transformPromptWithInlineSkills,
} from "./inline-skills.ts";

const SAMPLE_SKILLS: SkillCommandInfo[] = [
	{
		name: "skill:grill-me",
		description: "Interview the user relentlessly about a plan",
		sourceInfo: { source: "local", scope: "user", path: "/skills/grill-me/SKILL.md" },
	},
	{
		name: "skill:acme-html-report",
		description: "Generate a polished HTML report",
		sourceInfo: { source: "local", scope: "user", path: "/skills/acme-html-report/SKILL.md" },
	},
	{
		name: "skill:acme-developer-docs",
		description: "Fetch developer documentation",
		sourceInfo: { source: "extension:claude-skills", scope: "temporary", path: "/skills/acme-developer-docs/SKILL.md" },
	},
	{
		name: "skill:query-datalake",
		description: "Query data lake",
		sourceInfo: { source: "local", scope: "user", path: "/skills/query-datalake/SKILL.md" },
	},
	{
		name: "skill:notes",
		description: "Save and retrieve notes",
		sourceInfo: { source: "local", scope: "user", path: "/skills/notes/SKILL.md" },
	},
];

test("extractMidPromptSlashPrefix distinguishes start-of-message from mid-prompt slash tokens", () => {
	// Start of message on line 0 -> null (handled by Pi's built-in provider)
	assert.equal(extractMidPromptSlashPrefix(["/"], 0, 1), null);
	assert.equal(extractMidPromptSlashPrefix(["/grill"], 0, 6), null);
	assert.equal(extractMidPromptSlashPrefix(["  /grill"], 0, 8), null);
	assert.equal(isStartOfMessageSlash(["/grill"], 0, 6), true);

	// Mid-prompt after whitespace on line 0
	assert.equal(extractMidPromptSlashPrefix(["please /"], 0, 8), "/");
	assert.equal(extractMidPromptSlashPrefix(["please /grill"], 0, 13), "/grill");
	assert.equal(extractMidPromptSlashPrefix(["please /skill:grill"], 0, 19), "/skill:grill");
	assert.equal(extractMidPromptSlashPrefix(["please /grill on this"], 0, 13), "/grill");

	// Line 2+ at start of line or after whitespace
	assert.equal(extractMidPromptSlashPrefix(["first line", "/"], 1, 1), "/");
	assert.equal(extractMidPromptSlashPrefix(["first line", "/grill"], 1, 6), "/grill");
	assert.equal(extractMidPromptSlashPrefix(["first line", "  /grill"], 1, 8), "/grill");

	// Paths containing a second slash or not preceded by whitespace -> null
	assert.equal(extractMidPromptSlashPrefix(["check /tmp/foo"], 0, 14), null);
	assert.equal(extractMidPromptSlashPrefix(["check src/index.ts"], 0, 18), null);
	assert.equal(extractMidPromptSlashPrefix(["https://example.com"], 0, 19), null);
});

test("matchSkills matches bare prefixes, segment prefixes, multi-segment prefixes, and /skill:", () => {
	// Bare "/" lists all skills
	const all = matchSkills(SAMPLE_SKILLS, "/");
	assert.equal(all.length, SAMPLE_SKILLS.length);
	assert.equal(all[0].value, "skill:grill-me");
	assert.equal(all[0].description, "[u] Interview the user relentlessly about a plan");

	// "/skill:" lists all skills
	const allSkillNs = matchSkills(SAMPLE_SKILLS, "/skill:");
	assert.equal(allSkillNs.length, SAMPLE_SKILLS.length);

	// Prefix match
	const grill = matchSkills(SAMPLE_SKILLS, "/grill");
	assert.deepEqual(grill.map((i) => i.value), ["skill:grill-me"]);

	// Hyphen-segment prefix match
	const report = matchSkills(SAMPLE_SKILLS, "/report");
	assert.deepEqual(report.map((i) => i.value), ["skill:acme-html-report"]);

	// Multi-segment prefix match
	const devDocs = matchSkills(SAMPLE_SKILLS, "/dev-docs");
	assert.deepEqual(devDocs.map((i) => i.value), ["skill:acme-developer-docs"]);

	// Typing "/ski" on the way to "/skill:" keeps skills visible
	const ski = matchSkills(SAMPLE_SKILLS, "/ski");
	assert.equal(ski.length, SAMPLE_SKILLS.length);

	// Common single-segment root paths do not false-positive via subsequence matching
	assert.deepEqual(matchSkills(SAMPLE_SKILLS, "/etc"), []);
	assert.deepEqual(matchSkills(SAMPLE_SKILLS, "/tmp"), []);
	assert.deepEqual(matchSkills(SAMPLE_SKILLS, "/usr"), []);
	assert.deepEqual(matchSkills(SAMPLE_SKILLS, "/var"), []);
});

test("createInlineSkillAutocompleteProvider intercepts mid-prompt slash and delegates start-of-prompt", async () => {
	let delegatedSuggestions = 0;
	let delegatedApply = 0;
	const baseProvider = {
		async getSuggestions() {
			delegatedSuggestions += 1;
			return { items: [{ value: "model", label: "model" }], prefix: "/m" };
		},
		applyCompletion(lines: string[], cursorLine: number, cursorCol: number) {
			delegatedApply += 1;
			return { lines, cursorLine, cursorCol };
		},
	};

	const provider = createInlineSkillAutocompleteProvider(baseProvider, () => SAMPLE_SKILLS);
	const signal = new AbortController().signal;

	// Start-of-prompt delegates to baseProvider
	const startRes = await provider.getSuggestions(["/m"], 0, 2, { signal });
	assert.equal(delegatedSuggestions, 1);
	assert.equal(startRes?.items[0].value, "model");

	// Mid-prompt intercepts and returns matching skills
	const midRes = await provider.getSuggestions(["please /grill"], 0, 13, { signal });
	assert.equal(delegatedSuggestions, 1);
	assert.equal(midRes?.prefix, "/grill");
	assert.deepEqual(midRes?.items.map((i) => i.value), ["skill:grill-me"]);

	// Mid-prompt applyCompletion replaces the slash prefix and adds a trailing space
	const applied = provider.applyCompletion(
		["please /grill"],
		0,
		13,
		{ value: "skill:grill-me", label: "skill:grill-me" },
		"/grill",
	);
	assert.equal(delegatedApply, 0);
	assert.deepEqual(applied.lines, ["please /skill:grill-me "]);
	assert.equal(applied.cursorCol, "please /skill:grill-me ".length);

	// Mid-prompt applyCompletion does not double-space when text after cursor starts with a space
	const appliedInMiddle = provider.applyCompletion(
		["please /grill on this plan"],
		0,
		13,
		{ value: "skill:grill-me", label: "skill:grill-me" },
		"/grill",
	);
	assert.deepEqual(appliedInMiddle.lines, ["please /skill:grill-me on this plan"]);
	assert.equal(appliedInMiddle.cursorCol, "please /skill:grill-me".length);
});

test("transformPromptWithInlineSkills expands mid-prompt and bare skill references", () => {
	// Already leading `/skill:<name> ...` with no other skills -> null (Pi handles natively)
	assert.equal(
		transformPromptWithInlineSkills("/skill:grill-me check my plan", SAMPLE_SKILLS),
		null,
	);

	// Leading bare `/grill-me ...` -> rewritten to `/skill:grill-me ...`
	assert.equal(
		transformPromptWithInlineSkills("/grill-me check my plan", SAMPLE_SKILLS),
		"/skill:grill-me check my plan",
	);

	// Leading `/skill:grill-me\ncheck my plan` (newline directly after skill) -> space inserted
	assert.equal(
		transformPromptWithInlineSkills("/skill:grill-me\ncheck my plan", SAMPLE_SKILLS),
		"/skill:grill-me check my plan",
	);

	// Mid-prompt `/skill:grill-me` -> prepends `/skill:grill-me` and keeps `/grill-me` in sentence
	assert.equal(
		transformPromptWithInlineSkills(
			"here is my plan, please /skill:grill-me on the edge cases",
			SAMPLE_SKILLS,
		),
		"/skill:grill-me here is my plan, please /grill-me on the edge cases",
	);

	// Mid-prompt bare `/grill-me` -> prepends `/skill:grill-me`
	assert.equal(
		transformPromptWithInlineSkills(
			"here is my plan, please /grill-me on the edge cases",
			SAMPLE_SKILLS,
		),
		"/skill:grill-me here is my plan, please /grill-me on the edge cases",
	);

	// Backticked code spans and fenced code blocks are ignored
	assert.equal(
		transformPromptWithInlineSkills(
			"why does `/skill:grill-me` or `/grill-me` work?",
			SAMPLE_SKILLS,
		),
		null,
	);
	assert.equal(
		transformPromptWithInlineSkills("```\n/skill:grill-me\n```", SAMPLE_SKILLS),
		null,
	);

	// Non-skill commands at start of prompt are left untouched
	assert.equal(
		transformPromptWithInlineSkills("/model claude", SAMPLE_SKILLS, {
			nonSkillCommands: new Set(["model"]),
		}),
		null,
	);

	// Multi-skill prompt expands the second skill inline and puts the first at the start
	const multi = transformPromptWithInlineSkills(
		"use /skill:grill-me and /skill:notes here",
		SAMPLE_SKILLS,
		{
			readSkillFile: (p) => `---\nname: notes\ndescription: test\n---\nNotes body from ${p}\n`,
		},
	);
	assert.equal(
		multi,
		`/skill:grill-me <skill name="notes" location="/skills/notes/SKILL.md">\nReferences are relative to /skills/notes.\n\nNotes body from /skills/notes/SKILL.md\n</skill>\n\nuse /grill-me and /notes here`,
	);
});
