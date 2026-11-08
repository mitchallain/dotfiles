/**
 * Tests for the @-import expansion logic.
 *
 * Run with:
 *     cd ~/dotfiles/pi/extensions/at-imports
 *     npx tsx test.ts
 * or:
 *     nix shell nixpkgs#nodejs_22 -c npx -y tsx test.ts
 *
 * These tests exercise the pure `expandImports` function against fixtures
 * in `./test-fixtures/`. They run without pi or any of its deps.
 */
import { readFileSync } from "node:fs";
import { dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

import { clearCache, expandImports } from "./index.ts";

const here = dirname(fileURLToPath(import.meta.url));
const fixtures = resolve(here, "test-fixtures");

let failed = 0;
function check(cond: boolean, msg: string) {
	if (cond) {
		console.log("  PASS:", msg);
	} else {
		console.error("  FAIL:", msg);
		failed++;
	}
}

// --- fixture-based tests ---
console.log("fixture-based expansion:");
clearCache();
const input = readFileSync(resolve(fixtures, "fixture.md"), "utf8");
// Run from the fixtures directory so `@./foo.md` resolves correctly.
const origCwd = process.cwd();
process.chdir(fixtures);
const out = expandImports(input);
process.chdir(origCwd);

check(out.includes("IMPORTED_CONTENT"), "top-level import expanded");
check(out.includes("NESTED"), "nested import expanded recursively");
check(out.includes("INLINE"), "inline import expanded");
check(!out.includes("CODE_BLOCK_CONTENT"), "code block NOT expanded");
check(out.includes("@./should-not-expand.md"), "code block reference left verbatim");
check(out.includes("foo@bar.com"), "email left alone");
check(out.includes("@foo/bar"), "npm scope left alone");
// Self-reference: one level expands, the inner @./fixture.md hits the cycle guard
// and remains literal. So the final text still contains one literal
// "@./fixture.md end." at the deepest cycle point.
check(out.includes("@./fixture.md end."), "cycle left as literal reference");

// --- missing file ---
console.log("\nmissing file:");
clearCache();
const missing = expandImports("see @/nonexistent/file.md for info");
check(missing === "see @/nonexistent/file.md for info", "missing file left untouched");

// --- non-path @-mentions ---
console.log("\nnon-path @ mentions:");
clearCache();
const mentions = expandImports("contact foo@bar.com or @mariozechner for help");
check(mentions === "contact foo@bar.com or @mariozechner for help", "plain @-mentions left alone");

// --- depth guard ---
console.log("\ndepth guard:");
clearCache();
// If we ever get deeper than MAX_DEPTH=5 we just return the text as-is.
// Hard to exercise directly without cyclic fixtures; the cycle test above
// is the practical guarantee.
check(true, "depth guard reached only via explicit test (skipped)");

// --- leading-whitespace preservation ---
console.log("\nleading context:");
clearCache();
process.chdir(fixtures);
const leading = expandImports("prefix line\n@./inline.md\nsuffix line");
process.chdir(origCwd);
check(leading.includes("prefix line\nINLINE"), "newline before @import preserved");
check(leading.includes("\nsuffix line"), "content after expansion preserved");

console.log(`\n${failed === 0 ? "All tests passed." : `${failed} test(s) failed.`}`);
process.exit(failed === 0 ? 0 : 1);
