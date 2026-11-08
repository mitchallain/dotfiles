/**
 * Tests for the claude-local extension's pure logic.
 *
 * Run with:
 *     cd ~/dotfiles/pi/extensions/claude-local
 *     npx tsx test.ts
 * or:
 *     nix shell nixpkgs#nodejs_22 -c npx -y tsx test.ts
 *
 * Exercises discovery, ordering, dedup, the not-in-a-repo path, @import
 * expansion, and section assembly against fixtures in `./test-fixtures/`.
 * Runs without pi or any of its deps.
 */
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import {
	buildLocalSection,
	clearCache,
	discoverLocalFiles,
	FILENAME,
	loadLocalFile,
	type LocalFile,
} from "./index.ts";

const here = dirname(fileURLToPath(import.meta.url));
const fixtures = resolve(here, "test-fixtures");
const repo = resolve(fixtures, "repo");

let failed = 0;
function check(cond: boolean, msg: string) {
	if (cond) {
		console.log("  PASS:", msg);
	} else {
		console.error("  FAIL:", msg);
		failed++;
	}
}

// A findRoot that pretends `repo/` is a git root, independent of any real
// `.git` directory on disk.
const fakeRoot = (start: string): string | null => {
	const norm = resolve(start);
	if (norm === repo || norm.startsWith(repo + "/")) return repo;
	return null;
};

// --- discovery: ancestor walk, ordering, dedup ---
console.log("discovery (cwd deep in repo):");
{
	const cwd = resolve(repo, "sub", "deep");
	const found = discoverLocalFiles({ cwd, findRoot: fakeRoot });
	check(found.length === 2, `found 2 files (got ${found.length})`);
	check(found[0]?.label === FILENAME, `outermost first: ${found[0]?.label}`);
	check(found[1]?.label === "sub/CLAUDE.local.md", `cwd-side last: ${found[1]?.label}`);
}

console.log("discovery (cwd at repo root):");
{
	const found = discoverLocalFiles({ cwd: repo, findRoot: fakeRoot });
	check(found.length === 1, `found 1 file (got ${found.length})`);
	check(found[0]?.label === FILENAME, `label is ${FILENAME}`);
}

// --- discovery: not in a repo only checks cwd ---
console.log("discovery (no repo):");
{
	const standalone = resolve(fixtures, "standalone");
	const found = discoverLocalFiles({ cwd: standalone, findRoot: () => null });
	check(found.length === 1, `found 1 file (got ${found.length})`);
	check(found[0]?.label === FILENAME, `label is ${FILENAME}`);

	// A repo file is not picked up when we're not in a repo and cwd has none.
	const empty = discoverLocalFiles({ cwd: resolve(repo, "sub", "deep"), findRoot: () => null });
	check(empty.length === 0, `no files when cwd has none and not in a repo (got ${empty.length})`);
}

// --- reading + @import expansion ---
console.log("loadLocalFile expands @imports:");
{
	clearCache();
	// expandImports resolves `@./` relative to process.cwd(), so run from repo.
	const orig = process.cwd();
	process.chdir(repo);
	const sources = discoverLocalFiles({ cwd: repo, findRoot: fakeRoot });
	const file = loadLocalFile(sources[0]);
	process.chdir(orig);
	check(file !== null, "root file loaded");
	check(file?.body.includes("Root rule one."), "body retains original text");
	check(
		file?.body.includes("Imported content from imported.md."),
		"@./imported.md was expanded inline",
	);
	check(!file?.body.includes("@./imported.md"), "no literal @import token remains");
}

// --- section assembly ---
console.log("buildLocalSection:");
{
	const files: LocalFile[] = [
		{ sourcePath: "/x/CLAUDE.local.md", label: "CLAUDE.local.md", body: "Root body." },
		{ sourcePath: "/x/sub/CLAUDE.local.md", label: "sub/CLAUDE.local.md", body: "Sub body." },
	];
	const section = buildLocalSection(files);
	check(section.startsWith("## Local Instructions"), "has heading");
	check(section.includes("### CLAUDE.local.md\n\nRoot body."), "renders root file");
	check(section.includes("### sub/CLAUDE.local.md\n\nSub body."), "renders sub file");
	check(
		section.indexOf("Root body.") < section.indexOf("Sub body."),
		"preserves outermost-first order",
	);
	check(buildLocalSection([]) === "", "empty input → empty section");
}

console.log("");
if (failed > 0) {
	console.error(`${failed} check(s) failed.`);
	process.exit(1);
}
console.log("All checks passed.");
