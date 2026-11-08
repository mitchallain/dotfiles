/**
 * tmux-resurrect — track the active Pi session ID per PID for
 * `tmux-assistant-resurrect` (`~/.local/state/tmux-assistant-resurrect/pi-<pid>.json`).
 *
 * Why this is needed:
 * `tmux-assistant-resurrect` has no built-in hook for Pi and falls back to
 * heuristic JSONL file scoring in `~/.pi/agent/sessions/--<cwd>--`, which:
 *   1. Anchors on `process_start` and misses in-process session switches
 *      (`/new`, `/clear`, `/handoff`, `/resume`).
 *   2. Prefers `--session <id>` from `ps` argv after the first restore,
 *      ignoring any subsequent `/new` or `/clear` in that pane forever.
 *   3. Matches fresh uncommitted `pi` instances to old historical sessions.
 *
 * Writing `pi-<pid>.json` from inside Pi gives the save hook the exact live
 * session ID (and marks fresh uncommitted sessions as `unpersisted: true`
 * until the `.jsonl` file actually exists on disk).
 *
 * Note: `restore-assistant-sessions.sh` does not currently lock against
 * concurrent runs. Keep `@continuum-restore 'off'` in `tmux.conf` (since
 * `tmux-assistant-resurrect.tmux` otherwise defaults it to `'on'`) so a
 * background auto-restore on server start cannot race a manual `prefix + C-r`
 * during the `clear` + `sleep 0.3` window and paste the launch command into
 * Pi's prompt.
 */

import { randomUUID } from "node:crypto";
import { existsSync, mkdirSync, renameSync, unlinkSync, writeFileSync } from "node:fs";
import { homedir } from "node:os";
import type { ExtensionAPI, ExtensionContext } from "@earendil-works/pi-coding-agent";

const STATE_DIR =
	process.env.TMUX_ASSISTANT_RESURRECT_DIR ||
	`${process.env.HOME || homedir()}/.local/state/tmux-assistant-resurrect`;
const STATE_FILE = `${STATE_DIR}/pi-${process.pid}.json`;
const CLEANUP_KEY = Symbol.for("pi-tmux-resurrect/exit-cleanup");

function ensureStateDir(): void {
	if (existsSync(STATE_DIR)) return;
	const trimmed = STATE_DIR.replace(/\/+$/, "");
	const parent = trimmed.replace(/\/[^/]*$/, "");
	if (parent && parent !== trimmed) {
		mkdirSync(parent, { recursive: true });
	}
	try {
		mkdirSync(STATE_DIR, { mode: 0o700 });
	} catch (err: unknown) {
		if ((err as NodeJS.ErrnoException)?.code !== "EEXIST") {
			mkdirSync(STATE_DIR, { recursive: true, mode: 0o700 });
		}
	}
}

function cleanupStateFile(): void {
	try {
		unlinkSync(STATE_FILE);
	} catch {
		// Ignore if already removed.
	}
}

function installExitCleanup(): void {
	const g = globalThis as Record<symbol, unknown>;
	if (g[CLEANUP_KEY]) return;
	g[CLEANUP_KEY] = true;
	process.on("exit", cleanupStateFile);
}

export default function tmuxResurrectExtension(pi: ExtensionAPI) {
	let lastSessionId: string | null = null;
	let lastPersisted: boolean | null = null;
	let isTuiSession = false;

	function syncSessionState(ctx: ExtensionContext, force = false): void {
		if (!isTuiSession) return;

		let sessionId = "";
		let sessionFile = "";
		let cwd = "";
		try {
			sessionId = ctx.sessionManager.getSessionId() ?? "";
			sessionFile = ctx.sessionManager.getSessionFile() ?? "";
			cwd = ctx.sessionManager.getCwd() ?? process.cwd();
		} catch {
			return;
		}

		const persisted = Boolean(sessionId && sessionFile && existsSync(sessionFile));
		if (!force && sessionId === lastSessionId && persisted === lastPersisted) {
			return;
		}
		lastSessionId = sessionId;
		lastPersisted = persisted;

		const payload = JSON.stringify(
			{
				tool: "pi",
				pid: process.pid,
				session_id: persisted ? sessionId : "",
				unpersisted: !persisted,
				session_file: sessionFile,
				cwd,
				timestamp: new Date().toISOString(),
			},
			null,
			2,
		);

		const tmpFile = `${STATE_FILE}.${randomUUID()}.tmp`;
		try {
			ensureStateDir();
			writeFileSync(tmpFile, payload, { encoding: "utf8", mode: 0o600, flag: "wx" });
			renameSync(tmpFile, STATE_FILE);
		} catch {
			try {
				unlinkSync(tmpFile);
			} catch {
				// Ignore cleanup failure.
			}
		}
	}

	pi.on("session_start", (_event, ctx) => {
		isTuiSession = ctx.mode === "tui";
		if (!isTuiSession) return;
		installExitCleanup();
		syncSessionState(ctx, true);
	});

	pi.on("agent_start", (_event, ctx) => {
		syncSessionState(ctx);
	});

	pi.on("turn_end", (_event, ctx) => {
		syncSessionState(ctx);
	});

	pi.on("agent_settled", (_event, ctx) => {
		syncSessionState(ctx);
	});
}
