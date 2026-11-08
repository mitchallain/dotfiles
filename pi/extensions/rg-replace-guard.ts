/**
 * rg-replace-guard — deterministically block the ripgrep `-r` footgun.
 *
 * Problem
 * -------
 * Out of `grep` muscle memory it's easy to write `rg -rn 'pat'` intending
 * `-r` = "recursive". In ripgrep `-r` is `--replace`, so `-rn` parses as
 * "replace every match with the literal `n`". ripgrep already recurses by
 * default, so the `-r` does nothing useful — instead every match in the
 * OUTPUT is rewritten to the bundled letters (`n`, `l`, `i`, ...). It does
 * NOT modify files on disk, but it silently corrupts your search results and
 * wastes a turn. Same trap with `-rl`, `-ri`, `-rc`, `-nr`, etc.
 *
 * A CLAUDE.md note asks the model to avoid this, but that's advisory. This
 * extension makes it deterministic: the `tool_call` event fires before bash
 * runs, so we can inspect the command and block it with an explanatory reason
 * the model sees and can self-correct from — no human in the loop.
 *
 * Scope
 * -----
 * Only bundled short-flag clusters that pair `r` (=--replace) with other
 * short flags are blocked, e.g. `-rn`, `-rl`, `-ri`, `-rc`, `-nr`, `-rin`.
 * A lone `-r <value>` and the explicit long form `--replace=...` are treated
 * as intentional and pass through untouched. Detection is scoped per shell
 * segment so a legitimate `sed -rn ...` piped after an `rg` is not flagged.
 *
 * Override
 * --------
 * Set `RG_REPLACE_OK=1` in the environment, or include the marker
 * `rg-replace-ok` anywhere in the command, to bypass the guard for a genuine
 * bundled-replace invocation.
 *
 * Autofix (opt-in)
 * ----------------
 * Set `RG_AUTOFIX=1` to have the guard rewrite the command instead of
 * blocking it: the accidental `r` is stripped from the bundled cluster
 * (`-rn`→`-n`, `-nr`→`-n`, `-rl`→`-l`, `-rin`→`-in`) and the corrected
 * command runs, with a `ctx.ui.notify` so the correction is visible. The
 * rewrite is whole-token and whitespace-preserving, and only touches flag
 * tokens after an `rg`/`ripgrep` in the same shell segment. A full bypass
 * (`RG_REPLACE_OK` / `rg-replace-ok`) still runs the command exactly as typed.
 */

import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { isToolCallEventType } from "@earendil-works/pi-coding-agent";

const RG_NAMES = new Set(["rg", "ripgrep"]);
const BYPASS_MARKER = "rg-replace-ok";

/** Strip every lowercase `r` (=--replace) from a bundled short-flag cluster. */
function stripReplaceFlag(cluster: string): string {
	return "-" + cluster.slice(1).replace(/r/g, "");
}

/** Strip a leading path so `/usr/bin/rg` compares equal to `rg`. */
function basename(token: string): string {
	const cleaned = token.replace(/^['"]|['"]$/g, "");
	return cleaned.split("/").pop() ?? cleaned;
}

/**
 * A single-dash short-flag cluster (2+ letters) that bundles a lowercase `r`
 * with other flags — the accidental `--replace` case. Lone `-r` (its own
 * token) and long `--replace` never match.
 */
function isDangerousCluster(token: string): boolean {
	if (!/^-[A-Za-z]{2,}$/.test(token)) return false; // short cluster, not `--long`
	return token.slice(1).includes("r"); // lowercase r == --replace
}

/**
 * Split a compound command into top-level segments, respecting quotes so a
 * `|` or `;` inside a quoted pattern doesn't create a spurious boundary.
 * Breaks on ; & | newline and grouping parens (covers && and ||).
 */
function splitSegments(command: string): string[] {
	return splitSegmentsWithSeparators(command)
		.filter((piece) => !piece.isSeparator)
		.map((piece) => piece.text);
}

type CommandPiece = { text: string; isSeparator: boolean };

/**
 * Like {@link splitSegments} but preserves the separator characters as their
 * own pieces so a command can be rebuilt exactly. Quotes are respected so a
 * separator inside a quoted pattern doesn't create a spurious boundary.
 */
function splitSegmentsWithSeparators(command: string): CommandPiece[] {
	const pieces: CommandPiece[] = [];
	let current = "";
	let quote: '"' | "'" | null = null;
	const flush = () => {
		pieces.push({ text: current, isSeparator: false });
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
			pieces.push({ text: ch, isSeparator: true });
			continue;
		}
		current += ch;
	}
	flush();
	return pieces;
}

/**
 * Return the first offending flag token in a command, or null. A cluster is
 * only flagged once we've seen an `rg`/`ripgrep` token earlier in the same
 * segment, so unrelated tools (e.g. `sed -rn`) are never blocked.
 */
export function findRgReplaceFootgun(command: string): string | null {
	for (const segment of splitSegments(command)) {
		const tokens = segment.trim().split(/\s+/).filter(Boolean);
		let sawRg = false;
		for (const token of tokens) {
			if (RG_NAMES.has(basename(token))) {
				sawRg = true;
				continue;
			}
			if (sawRg && isDangerousCluster(token)) return token;
		}
	}
	return null;
}

/**
 * Rewrite a command that trips the footgun by stripping the accidental `r`
 * from every offending bundled cluster after an `rg`/`ripgrep` in each shell
 * segment. Returns the fixed command plus the list of `original -> fixed`
 * flag rewrites, or null if nothing was rewritten.
 *
 * Reconstruction preserves original whitespace: within a segment we split on
 * whitespace-capturing groups and only replace whole flag tokens, so quoted
 * patterns (which never match the cluster regex) are left untouched.
 */
export function fixRgReplaceFootgun(
	command: string,
): { command: string; rewrites: Array<{ from: string; to: string }> } | null {
	const rewrites: Array<{ from: string; to: string }> = [];
	// Split into segments while keeping the separators so we can rejoin exactly.
	const pieces = splitSegmentsWithSeparators(command);
	const fixedPieces = pieces.map((piece) => {
		if (piece.isSeparator) return piece.text;
		// Whitespace-preserving token split: alternating token / space chunks.
		const chunks = piece.text.split(/(\s+)/);
		let sawRg = false;
		for (let i = 0; i < chunks.length; i++) {
			const chunk = chunks[i];
			if (!chunk || /^\s+$/.test(chunk)) continue;
			if (RG_NAMES.has(basename(chunk))) {
				sawRg = true;
				continue;
			}
			if (sawRg && isDangerousCluster(chunk)) {
				const fixed = stripReplaceFlag(chunk);
				rewrites.push({ from: chunk, to: fixed });
				chunks[i] = fixed;
			}
		}
		return chunks.join("");
	});
	if (rewrites.length === 0) return null;
	return { command: fixedPieces.join(""), rewrites };
}

export default function rgReplaceGuard(pi: ExtensionAPI) {
	pi.on("tool_call", (event, ctx) => {
		if (!isToolCallEventType("bash", event)) return;

		const command = event.input.command;
		if (!command) return;

		if (process.env.RG_REPLACE_OK || command.includes(BYPASS_MARKER)) return;

		const offending = findRgReplaceFootgun(command);
		if (!offending) return;

		// Opt-in autofix: rewrite the command in place instead of blocking.
		if (process.env.RG_AUTOFIX) {
			const fix = fixRgReplaceFootgun(command);
			if (fix) {
				event.input.command = fix.command;
				const summary = fix.rewrites.map((r) => `${r.from} \u2192 ${r.to}`).join(", ");
				ctx?.ui?.notify?.(`rg-replace-guard: autofixed ${summary}`, "warning");
				return; // let the corrected command run
			}
		}

		const suffix = offending.slice(1).replace(/^.*r/, ""); // letters after `r`
		const replacementNote = suffix
			? `would replace every match with "${suffix}" in the output`
			: "replaces every match with the bundled letters in the output";

		return {
			block: true,
			reason:
				`Blocked \`${offending}\`: in ripgrep \`-r\` is \`--replace\`, not "recursive" ` +
				`(that's grep muscle memory). \`${offending}\` ${replacementNote}; ripgrep already ` +
				`recurses by default, so this silently corrupts your search results. ` +
				`Fix: drop the \`-r\` (e.g. \`rg -n 'pat'\`). If you truly want replacement, ` +
				`use the explicit long form \`rg --replace='...' 'pat'\`, or add the marker ` +
				`\`# ${BYPASS_MARKER}\` / set RG_REPLACE_OK=1 to bypass this guard. ` +
				`(Set RG_AUTOFIX=1 to auto-strip the \`-r\` instead of blocking.)`,
		};
	});
}
