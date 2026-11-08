/**
 * Focus-aware completion: one notification at agent_settled, never agent_end.
 * Keeps the session-name badge FSM and OSC 9;4 terminal progress, not pane titles.
 * Terminal I/O is TUI-only; reactions/input listeners are torn down on reload.
 */
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { StdinBuffer } from "@earendil-works/pi-tui";
import { notify, tmuxLocation, writePassthrough } from "./focus/notifications.ts";
import { FocusController, FocusFSM } from "./focus/state.ts";
export type { FocusState } from "./focus/state.ts";

// jiti reloads extension modules independently. Keep only the shared state global;
// session-owned subscriptions below are always removed and recreated.
const globals = globalThis as Record<symbol, unknown>;
const FSM_KEY = Symbol.for("pi-focus/fsm");
export const fsm = (globals[FSM_KEY] ??= new FocusFSM()) as FocusFSM;
const TERMINAL_KEY = Symbol.for("pi-focus/terminal");
const terminal = (globals[TERMINAL_KEY] ??= { focused: true, reporting: false }) as { focused: boolean; reporting: boolean };

function installExitCleanup(): void {
	const key = Symbol.for("pi-focus/global-cleanup");
	if (globals[key]) return;
	globals[key] = true;
	// Covers crashes that skip session_shutdown without interfering with signals.
	process.on("exit", () => {
		if (!terminal.reporting) return;
		try { process.stdout.write("\x1b[?1004l"); } catch { /* stdout may be gone */ }
	});
}

export default function (pi: ExtensionAPI) {
	const controller = new FocusController(fsm, terminal, {
		progress(state) {
			const value = state === "in_progress" ? "3;0" : state === "notify" ? "2;100" : "0;0";
			writePassthrough(`\x1b]9;4;${value}\x1b\\`);
		},
		location: tmuxLocation,
		notify,
	});
	let buffer: StdinBuffer | undefined;
	let listener: ((data: Buffer | string) => void) | undefined;
	function detachTerminal(): void {
		if (listener) process.stdin.off("data", listener);
		listener = undefined;
		buffer?.destroy();
		buffer = undefined;
		if (terminal.reporting) {
			process.stdout.write("\x1b[?1004l");
			terminal.reporting = false;
		}
	}

	pi.on("session_start", async (_event, ctx) => {
		detachTerminal();
		await controller.activate(ctx.mode === "tui");
		if (ctx.mode !== "tui") return;
		installExitCleanup();
		// DECSET goes to tmux itself, not through passthrough to the outer terminal.
		process.stdout.write("\x1b[?1004h");
		terminal.reporting = true;
		buffer = new StdinBuffer();
		buffer.on("data", (sequence: string) => {
			if (sequence === "\x1b[I") void controller.setFocused(true);
			else if (sequence === "\x1b[O") void controller.setFocused(false);
		});
		listener = data => buffer?.process(data);
		process.stdin.on("data", listener);
	});
	pi.on("agent_start", () => controller.start());
	pi.on("input", () => controller.input());
	pi.on("agent_settled", (_event, ctx) => controller.settled(ctx.sessionManager.getBranch(), ctx.sessionManager.getCwd()));
	pi.on("session_shutdown", () => {
		controller.deactivate();
		detachTerminal();
	});
}
