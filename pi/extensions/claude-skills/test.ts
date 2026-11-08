/**
 * Tests for the .claude/skills/ extension.
 *
 * Run with:
 *     cd ~/dotfiles/pi/extensions/claude-skills
 *     npx tsx test.ts
 * or:
 *     nix shell nixpkgs#nodejs_22 -c npx -y tsx test.ts
 *
 * Exercises the pure helpers (ancestor walk, nested walk, depth limit,
 * skip list, dedup, user-home gating) against fixtures in
 * `./test-fixtures/`. Runs without pi or any of its deps.
 */
import { dirname, resolve } from "node:path";
import { existsSync, lstatSync, readFileSync, rmSync } from "node:fs";
import { tmpdir } from "node:os";
import { fileURLToPath } from "node:url";

import {
	NESTED_DEPTH,
	NESTED_SKIP,
	countSkills,
	discoverSkillsDirs,
	findGitRoot,
	parseDepthEnv,
	walkAncestors,
	walkNested,
} from "./index.ts";
import { normalizeSkillsDir, repairSkillContent, scanFrontmatter } from "./normalize.ts";

const here = dirname(fileURLToPath(import.meta.url));
const fixtures = resolve(here, "test-fixtures");
const projectRoot = resolve(fixtures, "project");
const homeDir = resolve(fixtures, "home");
const homeSkills = resolve(homeDir, ".claude", "skills");
const inner = resolve(projectRoot, "src", "inner");

let failed = 0;
function check(cond: boolean, msg: string) {
	if (cond) {
		console.log("  PASS:", msg);
	} else {
		console.error("  FAIL:", msg);
		failed += 1;
	}
}

// All discovery calls in tests stop the ancestor walk at projectRoot, so
// they don't escape the fixture tree.
const findRoot = () => projectRoot;

// ---------------- countSkills ----------------
console.log("countSkills:");
{
	const n = countSkills(resolve(projectRoot, ".claude", "skills"));
	check(n === 2, `top-level .claude/skills has 2 SKILL.md children (got ${n})`);
}
{
	// `not-a-skill/` exists as a child but has no SKILL.md → not counted.
	const n = countSkills(resolve(projectRoot, ".claude", "skills"));
	check(n === 2, "directories without SKILL.md are not counted");
}
{
	const n = countSkills(homeSkills);
	check(n === 1, `home .claude/skills has 1 skill (got ${n})`);
}
{
	const n = countSkills("/this/does/not/exist");
	check(n === 0, "missing dir returns 0");
}

// ---------------- findGitRoot ----------------
console.log("\nfindGitRoot:");
{
	// fixtures/project/.git-marker is just a marker — findGitRoot looks
	// for `.git`, which doesn't exist in fixtures. So it should walk all
	// the way up. We're mostly verifying it doesn't throw.
	const r = findGitRoot(projectRoot);
	check(r === null || typeof r === "string", "returns null or a string");
}

// ---------------- walkAncestors ----------------
console.log("\nwalkAncestors:");
{
	const ancestors = walkAncestors(projectRoot, projectRoot);
	check(ancestors.length === 1, `from project root: 1 hit (got ${ancestors.length})`);
	check(ancestors[0].endsWith("/project/.claude/skills"), "found project root .claude/skills");
}
{
	const ancestors = walkAncestors(inner, projectRoot);
	check(
		ancestors.length === 1,
		`from project/src/inner up to project root: 1 hit (got ${ancestors.length})`,
	);
	check(
		ancestors[0].endsWith("/project/.claude/skills"),
		"climbed up to project's .claude/skills",
	);
}
{
	// Stop directly at inner — only checks inner itself. inner has no .claude/skills.
	const ancestors = walkAncestors(inner, inner);
	check(ancestors.length === 0, "stop=cwd with no skills here returns 0");
}

// ---------------- walkNested ----------------
console.log("\nwalkNested:");
{
	const nested = walkNested(projectRoot);
	const set = new Set(nested);
	check(
		set.has(resolve(projectRoot, "packages/frontend/.claude/skills")),
		"found packages/frontend/.claude/skills",
	);
	check(
		set.has(resolve(projectRoot, "a/b/c/d/.claude/skills")),
		`found depth-${NESTED_DEPTH} dir at a/b/c/d/.claude/skills`,
	);
	check(
		!set.has(resolve(projectRoot, "a/b/c/d/e/.claude/skills")),
		`skipped depth-${NESTED_DEPTH + 1} dir at a/b/c/d/e/.claude/skills`,
	);
	check(
		!set.has(resolve(projectRoot, "node_modules/.claude/skills")),
		"skipped node_modules/.claude/skills",
	);
	check(NESTED_SKIP.has("node_modules"), "node_modules in skip list");
	check(NESTED_SKIP.has(".git"), ".git in skip list");
}

// ---------------- discoverSkillsDirs ----------------
console.log("\ndiscoverSkillsDirs:");
{
	const found = discoverSkillsDirs({ cwd: projectRoot, homeDir, findRoot });
	const dirs = found.map((f) => f.dir);
	check(
		dirs.includes(resolve(projectRoot, ".claude/skills")),
		"includes top-level .claude/skills",
	);
	check(
		dirs.includes(resolve(projectRoot, "packages/frontend/.claude/skills")),
		"includes nested packages/frontend/.claude/skills",
	);
	check(
		dirs.includes(resolve(projectRoot, "a/b/c/d/.claude/skills")),
		"includes within-depth-limit a/b/c/d/.claude/skills",
	);
	check(
		!dirs.includes(resolve(projectRoot, "node_modules/.claude/skills")),
		"excludes node_modules/.claude/skills",
	);
	check(!dirs.includes(homeSkills), "excludes ~/.claude/skills by default");

	// Exactly one entry is tagged "cwd"; the rest are nested.
	const origins = found.map((f) => f.origin);
	check(origins.filter((o) => o === "cwd").length === 1, "one cwd-origin entry");
	check(origins.filter((o) => o === "nested").length >= 2, "two-plus nested-origin entries");

	// skillCount sanity check on the top-level dir.
	const top = found.find((f) => f.origin === "cwd");
	check(top !== undefined && top.skillCount === 2, "cwd entry has skillCount 2");
}

console.log("\ndiscoverSkillsDirs from sub-cwd:");
{
	const found = discoverSkillsDirs({ cwd: inner, homeDir, findRoot });
	const dirs = found.map((f) => f.dir);
	// Ancestor walk should still find the project's top-level skills,
	// even though cwd is `project/src/inner`.
	check(
		dirs.includes(resolve(projectRoot, ".claude/skills")),
		"ancestor walk reaches project/.claude/skills",
	);
	// Nested walk from src/inner should NOT pick up the project-root or
	// packages dirs — those aren't below `inner`.
	check(
		!dirs.includes(resolve(projectRoot, "packages/frontend/.claude/skills")),
		"sibling packages/frontend/.claude/skills not found from sub-cwd",
	);
	const origins = found.map((f) => f.origin);
	check(origins.includes("ancestor"), "ancestor-origin tagged correctly");
}

console.log("\ndiscoverSkillsDirs with includeUser:");
{
	const found = discoverSkillsDirs({ cwd: projectRoot, homeDir, findRoot, includeUser: true });
	const dirs = found.map((f) => f.dir);
	check(dirs.includes(homeSkills), "includes ~/.claude/skills when includeUser=true");
	const userEntry = found.find((f) => f.origin === "user");
	check(
		userEntry !== undefined && userEntry.skillCount === 1,
		"user-origin entry has skillCount 1",
	);
}

console.log("\ndedup:");
{
	// Calling twice should still only return each path once. (The
	// internal seen-set is per call, but realpath dedup means symlinked
	// equivalents collapse too.)
	const found = discoverSkillsDirs({ cwd: projectRoot, homeDir, findRoot });
	const dirs = found.map((f) => f.dir);
	const unique = new Set(dirs);
	check(dirs.length === unique.size, "no duplicate paths in single call");
}

console.log("\nnested-walk gating (no repo):");
{
	// findRoot returns null → cwd is not inside a git repo. Nested walk
	// must be skipped, ancestor walk still runs.
	const found = discoverSkillsDirs({
		cwd: projectRoot,
		homeDir,
		findRoot: () => null,
	});
	const origins = found.map((f) => f.origin);
	check(
		origins.every((o) => o !== "nested"),
		"no nested-origin entries when findRoot returns null",
	);
	// The cwd's own .claude/skills is still picked up via walkAncestors.
	check(
		origins.includes("cwd"),
		"cwd-origin entry still found via ancestor walk when not in a repo",
	);
}

console.log("\nnested-walk gating (forceNested overrides no-repo):");
{
	const found = discoverSkillsDirs({
		cwd: projectRoot,
		homeDir,
		findRoot: () => null,
		forceNested: true,
	});
	const origins = found.map((f) => f.origin);
	check(
		origins.includes("nested"),
		"nested entries returned when forceNested=true even without a repo",
	);
}

console.log("\nnestedDepth=0 disables nested walk inside a repo:");
{
	const found = discoverSkillsDirs({
		cwd: projectRoot,
		homeDir,
		findRoot,
		nestedDepth: 0,
	});
	const origins = found.map((f) => f.origin);
	check(
		origins.every((o) => o !== "nested"),
		"nestedDepth=0 produces no nested entries",
	);
}

console.log("\nnestedDepth=0 wins over forceNested:");
{
	const found = discoverSkillsDirs({
		cwd: projectRoot,
		homeDir,
		findRoot: () => null,
		forceNested: true,
		nestedDepth: 0,
	});
	const origins = found.map((f) => f.origin);
	check(
		origins.every((o) => o !== "nested"),
		"nestedDepth=0 still wins when forceNested=true",
	);
}

console.log("\nnestedDepth shallower than max blocks deeper hits:");
{
	const found = discoverSkillsDirs({
		cwd: projectRoot,
		homeDir,
		findRoot,
		nestedDepth: 2,
	});
	const dirs = found.map((f) => f.dir);
	// packages/frontend is 2 levels deep → still found.
	check(
		dirs.includes(resolve(projectRoot, "packages/frontend/.claude/skills")),
		"nestedDepth=2 still finds packages/frontend (2 levels)",
	);
	// a/b/c/d is 4 levels deep → must be excluded with nestedDepth=2.
	check(
		!dirs.includes(resolve(projectRoot, "a/b/c/d/.claude/skills")),
		"nestedDepth=2 excludes a/b/c/d (4 levels)",
	);
}

console.log("\nwalkNested respects maxDepth arg directly:");
{
	const shallow = walkNested(projectRoot, 1);
	const set = new Set(shallow);
	check(
		!set.has(resolve(projectRoot, "packages/frontend/.claude/skills")),
		"walkNested(cwd, 1) does not reach packages/frontend (depth 2)",
	);
	const none = walkNested(projectRoot, 0);
	check(none.length === 0, "walkNested(cwd, 0) returns empty array");
	const neg = walkNested(projectRoot, -3);
	check(neg.length === 0, "walkNested(cwd, -3) returns empty array");
}

console.log("\nparseDepthEnv:");
{
	check(parseDepthEnv(undefined, 4) === 4, "undefined → fallback");
	check(parseDepthEnv("", 4) === 4, "empty → fallback");
	check(parseDepthEnv("abc", 4) === 4, "non-numeric → fallback");
	check(parseDepthEnv("0", 4) === 0, "'0' parses to 0");
	check(parseDepthEnv("7", 4) === 7, "'7' parses to 7");
	check(parseDepthEnv("-2", 4) === 0, "negative clamps to 0");
	check(parseDepthEnv("3.9", 4) === 3, "float parses leading int");
}

// ---------------- scanFrontmatter ----------------
console.log("\nscanFrontmatter:");
{
	const r = scanFrontmatter("name: foo\ndescription: a plain string.");
	check(r.kind === "clean", "plain key/value frontmatter is clean");
}
{
	const r = scanFrontmatter(
		"name: foo\ndescription: Use when X. Triggers on: 'add', 'fix'.",
	);
	check(r.kind === "needs-repair", "unquoted `: ` in description → needs-repair");
}
{
	const r = scanFrontmatter(
		"name: foo\ndescription: 'Use when X. Triggers on: a, b'",
	);
	check(r.kind === "clean", "already-quoted scalar with `: ` is clean");
}
{
	const r = scanFrontmatter("name: foo\nmeta:\n  key: value");
	check(r.kind === "unrepairable", "nested mapping → unrepairable");
}
{
	const r = scanFrontmatter("- one\n- two");
	check(r.kind === "unrepairable", "top-level sequence → unrepairable");
}
{
	const r = scanFrontmatter("name: foo\ndescription: |\n  block");
	check(r.kind === "unrepairable", "block scalar (`|`) → unrepairable");
}
{
	const r = scanFrontmatter("# comment line\nname: foo");
	check(r.kind === "clean", "comment lines are tolerated");
}
{
	const r = scanFrontmatter("name: foo\ndisable-model-invocation: true\ncount: 5");
	check(r.kind === "clean", "booleans and numbers are recognized as typed scalars");
}

// ---------------- repairSkillContent ----------------
console.log("\nrepairSkillContent:");
{
	const input = `---
name: foo
description: Triggers on: 'a', 'b'.
---
body
`;
	const out = repairSkillContent(input);
	check(
		out.includes("description: 'Triggers on: ''a'', ''b''.'"),
		"description is single-quoted with embedded single quotes doubled",
	);
	check(out.endsWith("body\n"), "body is preserved verbatim");
	check(out.includes("name: 'foo'"), "plain key values are also single-quoted");
}
{
	// Already-quoted values must not be wrapped a second time.
	const input = `---
name: 'foo'
description: "already quoted"
---
body
`;
	const out = repairSkillContent(input);
	check(out === input, "no-op when all values are already quoted");
}

// ---------------- normalizeSkillsDir ----------------
console.log("\nnormalizeSkillsDir:");
{
	const source = resolve(here, "test-fixtures", "normalize");
	const cacheRoot = resolve(tmpdir(), `claude-skills-test-${process.pid}`);
	rmSync(cacheRoot, { recursive: true, force: true });

	const result = normalizeSkillsDir(source, cacheRoot);
	check(result.cleanCount === 1, `clean skills mirrored (got ${result.cleanCount})`);
	check(result.repairedCount === 1, `repaired skills (got ${result.repairedCount})`);
	check(
		result.unrepairable.length === 2,
		`unrepairable skills (got ${result.unrepairable.length})`,
	);

	const cleanLink = resolve(result.mirror, "clean");
	check(
		lstatSync(cleanLink).isSymbolicLink(),
		"clean skill subdir is a whole-subdir symlink",
	);
	check(
		existsSync(resolve(cleanLink, "helper.sh")),
		"clean skill: extra files visible through the symlink",
	);

	const claudeDir = resolve(result.mirror, "claude-style");
	check(
		lstatSync(claudeDir).isDirectory() && !lstatSync(claudeDir).isSymbolicLink(),
		"repaired skill: subdir is a real directory",
	);
	const fixedSkillMd = resolve(claudeDir, "SKILL.md");
	check(
		lstatSync(fixedSkillMd).isFile() && !lstatSync(fixedSkillMd).isSymbolicLink(),
		"repaired skill: SKILL.md is a real (rewritten) file",
	);
	const fixedContent = readFileSync(fixedSkillMd, "utf-8");
	check(
		fixedContent.includes("description: '"),
		"repaired skill: description is now single-quoted",
	);
	check(
		fixedContent.includes("Triggers on: ''add a test''"),
		"repaired skill: embedded single quotes are doubled",
	);
	const assetLink = resolve(claudeDir, "asset.txt");
	check(
		lstatSync(assetLink).isSymbolicLink(),
		"repaired skill: non-SKILL.md entries are symlinked back to source",
	);
	check(
		readFileSync(assetLink, "utf-8") === "extra asset\n",
		"repaired skill: symlinked asset content matches source",
	);

	check(
		!existsSync(resolve(result.mirror, "unrepairable-nested")),
		"unrepairable skill (nested mapping) dropped from mirror",
	);
	check(
		!existsSync(resolve(result.mirror, "unrepairable-list")),
		"unrepairable skill (top-level list) dropped from mirror",
	);

	const reasons = result.unrepairable.map((u) => u.reason).join(" | ");
	check(
		result.unrepairable.every((u) => u.skillPath.startsWith(source)),
		"unrepairable diagnostics report original (source-tree) paths",
	);
	check(reasons.length > 0, "unrepairable diagnostics include a reason string");

	rmSync(cacheRoot, { recursive: true, force: true });
}

console.log("\nnormalizeSkillsDir is idempotent across calls:");
{
	const source = resolve(here, "test-fixtures", "normalize");
	const cacheRoot = resolve(tmpdir(), `claude-skills-test-idem-${process.pid}`);
	rmSync(cacheRoot, { recursive: true, force: true });

	const a = normalizeSkillsDir(source, cacheRoot);
	const b = normalizeSkillsDir(source, cacheRoot);
	check(a.mirror === b.mirror, "mirror path is stable across calls");
	check(a.cleanCount === b.cleanCount, "cleanCount is stable");
	check(a.repairedCount === b.repairedCount, "repairedCount is stable");
	check(
		a.unrepairable.length === b.unrepairable.length,
		"unrepairable count is stable across calls",
	);
	rmSync(cacheRoot, { recursive: true, force: true });
}

console.log(`\n${failed === 0 ? "All tests passed." : `${failed} test(s) failed.`}`);
process.exit(failed === 0 ? 0 : 1);
