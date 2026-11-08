/**
 * nix-store-scan-guard — block recursive traversals rooted at the entire
 * `/nix/store`, or at the whole filesystem root `/`.
 *
 * Problem
 * -------
 * `/nix/store` on a Nix/NixOS box is enormous — hundreds of thousands of
 * derivation outputs, often hundreds of gigabytes. A command like
 * `find /nix/store`, `rg pat /nix/store`, `fd foo /nix/store`, or
 * `grep -r pat /nix/store` walks that entire tree. It pins I/O for minutes,
 * blows the tool-output limit, and burns a turn for nothing. Agents fall into
 * this out of habit (e.g. "search the pi install") when they should scope the
 * search to a SPECIFIC store path like `/nix/store/<hash>-pi-0.80.3/...`.
 *
 * The same failure mode shows up as `find /`, `fd . /`, `rg pat /`, etc. — a
 * scan rooted at the filesystem root, which walks `/nix/store` and every
 * other mount too. That case is blocked the same way.
 *
 * A CLAUDE.md note is advisory; this makes it deterministic. The `tool_call`
 * event fires before bash runs, so we inspect the command and block a
 * whole-store scan with a self-correcting message the model can act on.
 *
 * Scope
 * -----
 * Only recursive-search tools are considered: `find`, `fd`/`fdfind`,
 * `rg`/`ripgrep`, `ag`, and the grep family (`grep`/`egrep`/`fgrep` **only**
 * when given a recursive flag like `-r`/`-R`/`-rn`, plus always-recursive
 * `rgrep`). A command is blocked only when one of its arguments points at the
 * WHOLE store:
 *   - `/nix/store`, `/nix/store/` (trailing slashes), `/nix/store/.`
 *   - a top-level glob: `/nix/store/*`, `/nix/store/*-foo`, or `/nix/store/*` + `/bin`
 *
 * A concrete store path is always allowed, even with globs deeper inside it:
 *   - `/nix/store/dsdk...-pi-0.80.3`
 *   - `/nix/store/dsdk...-pi-0.80.3/lib/.../doc.md` (globs deeper inside are fine)
 *   - `/nix/store/abc-*`  (literal-prefixed glob — the shell narrows it)
 *
 * Or at the WHOLE filesystem root:
 *   - `/`, `//` (trailing slashes), `/.`, `/..`
 *   - a top-level glob: `/*`
 *
 * A concrete top-level directory (`/etc`, `/home`, `/usr/local/...`) is
 * always allowed.
 *
 * Detection is scoped per shell segment and per command word (leading
 * `VAR=val`, `sudo`, `env`, `xargs`, ... wrappers are peeled off first), so an
 * `echo "find /nix/store"` or an unrelated tool in a pipeline is not flagged.
 *
 * Override
 * --------
 * Set `NIX_STORE_SCAN_OK=1` in the environment, or include the marker
 * `nix-store-scan-ok` anywhere in the command, to bypass the guard for a
 * genuinely intentional whole-store scan.
 */

import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { isToolCallEventType } from "@earendil-works/pi-coding-agent";

const BYPASS_MARKER = "nix-store-scan-ok";

/** Tools that recurse into a directory tree by default. */
const RECURSIVE_SEARCH_TOOLS = new Set(["find", "fd", "fdfind", "rg", "ripgrep", "ag"]);
/** grep family — only dangerous with a recursive flag (except always-recursive rgrep). */
const GREP_TOOLS = new Set(["grep", "egrep", "fgrep", "rgrep"]);
/** Wrapper/prefix commands to peel off before finding the real command word. */
const WRAPPER_COMMANDS = new Set([
	"sudo",
	"doas",
	"env",
	"time",
	"nice",
	"ionice",
	"nohup",
	"stdbuf",
	"command",
	"builtin",
	"xargs",
]);

/** Strip a single layer of surrounding quotes from a token. */
function stripQuotes(token: string): string {
	return token.replace(/^['"]/, "").replace(/['"]$/, "");
}

/** Strip a leading path so `/nix/store/.../bin/find` compares equal to `find`. */
function basename(token: string): string {
	const cleaned = stripQuotes(token);
	return cleaned.split("/").pop() ?? cleaned;
}

/**
 * True when a token points at the WHOLE filesystem root `/` rather than some
 * specific subtree. Accepted forms: `/`, `/.`, `/..`, any of those with
 * extra trailing slashes, and a top-level glob (`/*`).
 */
export function isWholeRootPath(token: string): boolean {
	const t = stripQuotes(token);
	if (!t.startsWith("/")) return false;

	const afterSlashes = t.replace(/^\/+/, "");
	if (afterSlashes === "") return true; // "/", "//", ...
	if (afterSlashes === "." || afterSlashes === "..") return true; // "/.", "/.."

	const firstComponent = afterSlashes.split("/")[0];
	// A glob that begins the first component spans the whole filesystem.
	if (/^[*?\[]/.test(firstComponent)) return true;

	return false;
}

/**
 * True when a token points at the WHOLE `/nix/store` rather than a specific
 * derivation output. See the module header for the exact accepted forms.
 */
export function isWholeNixStorePath(token: string): boolean {
	const t = stripQuotes(token);
	if (!t.startsWith("/nix/store")) return false;

	const rest = t.slice("/nix/store".length);
	if (rest === "") return true; // exactly /nix/store
	if (!rest.startsWith("/")) return false; // /nix/storefoo — not the store

	const afterSlashes = rest.replace(/^\/+/, "");
	if (afterSlashes === "") return true; // /nix/store/ (trailing slashes only)

	const firstComponent = afterSlashes.split("/")[0];
	if (firstComponent === "." || firstComponent === "..") return true; // /nix/store/.
	// A glob that begins the first component spans the whole store
	// (`*`, `**`, `*-foo`). A literal-prefixed glob (`abc-*`) is narrowed by the
	// shell to a handful of concrete outputs, so it is left alone.
	if (/^[*?\[]/.test(firstComponent)) return true;

	return false; // concrete derivation directory → specific path, allowed
}

/**
 * Split a compound command into top-level segments, respecting quotes so a
 * `|`/`;`/`&` inside a quoted pattern doesn't create a spurious boundary.
 */
function splitSegments(command: string): string[] {
	const segments: string[] = [];
	let current = "";
	let quote: '"' | "'" | null = null;
	const flush = () => {
		segments.push(current);
		current = "";
	};
	for (let i = 0; i < command.length; i++) {
		const ch = command[i];
		if (quote) {
			if (ch === quote) quote = null;
			current += ch;
			continue;
		}
		if (ch === '"' || ch === "'") {
			quote = ch;
			current += ch;
			continue;
		}
		if (ch === ";" || ch === "|" || ch === "&" || ch === "\n" || ch === "(" || ch === ")") {
			flush();
			continue;
		}
		current += ch;
	}
	flush();
	return segments;
}

/**
 * Peel leading `VAR=val` assignments and wrapper commands (`sudo`, `env`,
 * `xargs`, ..., plus any flags they carry) off a token list and return the real
 * command word and its arguments.
 */
function resolveCommand(tokens: string[]): { tool: string | null; args: string[] } {
	let i = 0;
	while (i < tokens.length) {
		const token = tokens[i];
		if (/^[A-Za-z_][A-Za-z0-9_]*=/.test(token)) {
			i++; // environment assignment prefix
			continue;
		}
		if (WRAPPER_COMMANDS.has(basename(token))) {
			i++;
			while (i < tokens.length && tokens[i].startsWith("-")) i++; // skip wrapper flags
			continue;
		}
		return { tool: token, args: tokens.slice(i + 1) };
	}
	return { tool: null, args: [] };
}

/** True when a grep argument list requests recursion (`-r`, `-R`, `-rn`, ...). */
function hasGrepRecursiveFlag(args: string[]): boolean {
	for (const arg of args) {
		if (arg === "--recursive" || arg === "--dereference-recursive") return true;
		// Short-flag cluster containing r/R (`-r`, `-R`, `-rn`, `-nr`, `-Rl`, ...).
		if (/^-[A-Za-z]*[rR][A-Za-z]*$/.test(arg)) return true;
	}
	return false;
}

/**
 * Return the offending `{ tool, path, kind }` for the first recursive-search
 * command in `command` that is rooted at the whole `/nix/store` or the whole
 * filesystem root `/`, or null if none.
 */
export function findWholeStoreScan(
	command: string,
): { tool: string; path: string; kind: "store" | "root" } | null {
	for (const segment of splitSegments(command)) {
		const tokens = segment.trim().split(/\s+/).filter(Boolean);
		if (tokens.length === 0) continue;

		const { tool, args } = resolveCommand(tokens);
		if (!tool) continue;

		const base = basename(tool);
		const isRecursiveTool = RECURSIVE_SEARCH_TOOLS.has(base);
		const isGrep = GREP_TOOLS.has(base);
		if (!isRecursiveTool && !isGrep) continue;
		// Non-recursive grep on a directory just errors out cheaply — only flag
		// grep once recursion is requested (rgrep is always recursive).
		if (isGrep && base !== "rgrep" && !hasGrepRecursiveFlag(args)) continue;

		for (const arg of args) {
			if (isWholeNixStorePath(arg)) return { tool: base, path: stripQuotes(arg), kind: "store" };
			if (isWholeRootPath(arg)) return { tool: base, path: stripQuotes(arg), kind: "root" };
		}
	}
	return null;
}

export default function nixStoreScanGuard(pi: ExtensionAPI) {
	pi.on("tool_call", (event) => {
		if (!isToolCallEventType("bash", event)) return;

		const command = event.input.command;
		if (!command) return;

		if (process.env.NIX_STORE_SCAN_OK || command.includes(BYPASS_MARKER)) return;

		const hit = findWholeStoreScan(command);
		if (!hit) return;

		if (hit.kind === "root") {
			return {
				block: true,
				reason:
					`Blocked \`${hit.tool}\` recursing over the entire filesystem root \`${hit.path}\`. ` +
					`A whole-root scan walks every mount (including \`/nix/store\`, \`/proc\`, network ` +
					`mounts, etc.), pinning I/O for minutes and flooding the output. Scope the search to a ` +
					`SPECIFIC directory instead (e.g. your repo, \`~\`, or a known install prefix). If you ` +
					`really must scan the whole filesystem, add the marker \`# ${BYPASS_MARKER}\` or set ` +
					`NIX_STORE_SCAN_OK=1 to bypass this guard.`,
			};
		}

		return {
			block: true,
			reason:
				`Blocked \`${hit.tool}\` recursing over the entire \`${hit.path}\`. The Nix ` +
				`store holds hundreds of thousands of derivation outputs (often hundreds of GB); ` +
				`scanning all of it pins I/O for minutes and floods the output. ` +
				`Scope the search to a SPECIFIC store path instead — e.g. ` +
				`\`${hit.tool} ... /nix/store/<hash>-<name>/...\` (a concrete package dir), which is ` +
				`allowed. To find which package provides a file, use \`nix-locate\`/\`nix-index\`. ` +
				`If you really must scan the whole store, add the marker \`# ${BYPASS_MARKER}\` ` +
				`or set NIX_STORE_SCAN_OK=1 to bypass this guard.`,
		};
	});
}

