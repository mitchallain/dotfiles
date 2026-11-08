import { completionNotification, type CompletionNotification } from "./notifications.ts";
import type { SessionEntry } from "@earendil-works/pi-coding-agent";

export type FocusState = "idle" | "in_progress" | "notify";
type Listener = (next: FocusState, prev: FocusState) => void | Promise<void>;

export class FocusFSM {
	private current: FocusState = "idle";
	private listeners = new Set<Listener>();
	get state(): FocusState { return this.current; }
	on(listener: Listener): () => void {
		this.listeners.add(listener);
		return () => { this.listeners.delete(listener); };
	}
	async transition(next: FocusState): Promise<void> {
		if (next === this.current) return;
		const previous = this.current;
		this.current = next;
		for (const listener of this.listeners) {
			try { await listener(next, previous); } catch { /* never break an agent turn */ }
		}
	}
}

interface Effects {
	progress(state: FocusState): void;
	location(): Promise<string | null>;
	notify(notification: CompletionNotification): void;
}

/** Session-owned reactions to a shared FSM; no stale reload-global closures. */
export class FocusController {
	private active = false;
	private interactive = false;
	private generation = 0;
	private unsubscribe: (() => void) | undefined;
	readonly fsm: FocusFSM;
	private focus: { focused: boolean };
	private effects: Effects;
	constructor(fsm: FocusFSM, focus: { focused: boolean }, effects: Effects) {
		this.fsm = fsm;
		this.focus = focus;
		this.effects = effects;
	}

	async activate(interactive: boolean): Promise<void> {
		this.deactivate();
		this.active = true;
		this.interactive = interactive;
		await this.fsm.transition("idle");
		if (interactive) {
			this.unsubscribe = this.fsm.on(next => { this.effects.progress(next); });
			this.effects.progress("idle");
		}
	}
	deactivate(): void {
		this.generation++;
		this.active = false;
		this.unsubscribe?.();
		this.unsubscribe = undefined;
		if (this.interactive) this.effects.progress("idle");
		this.interactive = false;
	}
	async start(): Promise<void> {
		if (!this.active) return;
		this.generation++;
		await this.fsm.transition("in_progress");
	}
	async input(): Promise<void> {
		if (!this.active) return;
		this.generation++; // invalidate a notification waiting for tmux
		// Steering/follow-up input is not evidence that the run is finished.
		if (this.fsm.state === "notify") await this.fsm.transition("idle");
	}
	async setFocused(focused: boolean): Promise<void> {
		this.focus.focused = focused;
		if (focused) await this.input();
	}
	async settled(branch: SessionEntry[], cwd: string): Promise<void> {
		if (!this.active || this.fsm.state !== "in_progress") return;
		const notification = completionNotification(branch, cwd);
		const generation = ++this.generation;
		if (!this.interactive || this.focus.focused || !notification) {
			await this.fsm.transition("idle");
			return;
		}
		await this.fsm.transition("notify");
		try {
			const location = await this.effects.location();
			// Focus/input/new work/reload may have happened while querying tmux.
			if (!this.active || generation !== this.generation || this.focus.focused || this.fsm.state !== "notify") return;
			this.effects.notify({ ...notification, body: location ? `${notification.body} (${location})` : notification.body });
		} catch { /* notification failures must not break completion */ }
	}
}
