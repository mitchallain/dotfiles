import assert from "node:assert/strict";
import { test } from "node:test";
import { FocusController, FocusFSM } from "./state.ts";
import { completionNotification, notificationRoute, singleLine } from "./notifications.ts";
import type { SessionEntry } from "@earendil-works/pi-coding-agent";

const branch = (text = "Done\nMore details", stopReason = "stop") => [
	{ type: "message", message: { role: "assistant", stopReason, content: [{ type: "text", text }] } },
] as SessionEntry[];
function fixture(focused = false, fsm = new FocusFSM(), location: () => Promise<string | null> = async () => "dev:2:app") {
	const progress: string[] = [];
	const notifications: any[] = [];
	const focus = { focused };
	const controller = new FocusController(fsm, focus, { progress: state => { progress.push(state); }, location, notify: value => { notifications.push(value); } });
	return { controller, fsm, progress, notifications, focus };
}

test("one rich unfocused completion notification, no duplicate settled popup", async () => {
	const f = fixture();
	await f.controller.activate(true);
	await f.controller.start();
	await f.controller.settled(branch(), "/src/app");
	await f.controller.settled(branch(), "/src/app");
	assert.equal(f.fsm.state, "notify");
	assert.deepEqual(f.notifications, [{ title: "pi — app", body: "Done (dev:2:app)", isError: false }]);
	assert.deepEqual(f.progress, ["idle", "in_progress", "notify"]);
	await f.controller.setFocused(true);
	assert.equal(f.fsm.state, "idle");
	f.controller.deactivate();
});

test("focused success/errors, aborts, no assistant and non-TUI are silent", async () => {
	for (const [focused, interactive, entries] of [
		[true, true, branch()], [true, true, branch("Failed", "error")],
		[false, true, branch("", "aborted")], [false, true, []], [false, false, branch()],
	] as const) {
		const f = fixture(focused);
		await f.controller.activate(interactive);
		await f.controller.start();
		await f.controller.settled(entries as SessionEntry[], "/repo");
		assert.equal(f.notifications.length, 0);
		assert.equal(f.fsm.state, "idle");
		if (!interactive) assert.deepEqual(f.progress, []);
		f.controller.deactivate();
	}
});

test("steering/follow-up input and focus-in never clear working state", async () => {
	const f = fixture();
	await f.controller.activate(true);
	await f.controller.start();
	await f.controller.input();
	assert.equal(f.fsm.state, "in_progress");
	await f.controller.setFocused(true);
	assert.equal(f.fsm.state, "in_progress");
	await f.controller.setFocused(false);
	await f.controller.settled(branch(), "/repo");
	await f.controller.input();
	assert.equal(f.fsm.state, "idle");
	assert.equal(f.notifications.length, 1);
	f.controller.deactivate();
});

test("reload/session replacement recreates subscriber, old subscriber is removed", async () => {
	const fsm = new FocusFSM();
	const old = fixture(false, fsm);
	await old.controller.activate(true);
	await old.controller.start();
	old.controller.deactivate();
	const oldCount = old.progress.length;
	const next = fixture(false, fsm);
	await next.controller.activate(true);
	await next.controller.start();
	await next.controller.settled(branch(), "/repo");
	assert.equal(old.progress.length, oldCount);
	assert.equal(old.notifications.length, 0);
	assert.equal(next.notifications.length, 1);
	assert.deepEqual(next.progress, ["idle", "in_progress", "notify"]);
	next.controller.deactivate();
	await next.controller.activate(true);
	const before = next.progress.length;
	await next.controller.start();
	assert.equal(next.progress.length, before + 1, "exactly one progress subscriber after reactivation");
	next.controller.deactivate();
});

test("focus/input/new turn/shutdown during tmux query invalidates pending notification", async () => {
	for (const action of ["focus", "input", "start", "shutdown"] as const) {
		let release!: (value: string) => void;
		const f = fixture(false, new FocusFSM(), () => new Promise(resolve => { release = resolve; }));
		await f.controller.activate(true);
		await f.controller.start();
		const pending = f.controller.settled(branch(), "/repo");
		await new Promise(resolve => setImmediate(resolve));
		if (action === "focus") await f.controller.setFocused(true);
		if (action === "input") await f.controller.input();
		if (action === "start") await f.controller.start();
		if (action === "shutdown") f.controller.deactivate();
		release("stale:1:location");
		await pending;
		assert.equal(f.notifications.length, 0, action);
		f.controller.deactivate();
	}
});

test("branch summaries, error details, bounded body and control-character safety", () => {
	assert.equal(completionNotification(branch("Done\nsecond line"), "/repo")?.body, "Done");
	const multiple = [...branch("abandoned"), ...branch("current")];
	assert.equal(completionNotification(multiple, "/repo")?.body, "current");
	const error = branch("", "error");
	(error[0] as any).message.errorMessage = "Connection failed";
	assert.deepEqual(completionNotification(error, "/repo"), { title: "pi error — repo", body: "Connection failed", isError: true });
	assert.equal(completionNotification(branch("x".repeat(500)), "/repo")?.body.length, 160);
	assert.equal(completionNotification(branch("\n\n"), "/repo")?.body, "Ready for input");
	assert.ok(!/[\x00-\x1f\x7f-\x9f]/.test(singleLine("x\x1b\x07\x9c\n\t")));
});

test("terminal-aware routing preserves remote client notifications", () => {
	assert.equal(notificationRoute({}, "linux"), "linux");
	assert.equal(notificationRoute({}, "darwin"), "mac");
	assert.equal(notificationRoute({ WT_SESSION: "1" }, "win32"), "windows");
	assert.equal(notificationRoute({ KITTY_WINDOW_ID: "1" }, "linux"), "osc99");
	assert.equal(notificationRoute({ TERM_PROGRAM: "WezTerm" }, "linux"), "osc777");
	assert.equal(notificationRoute({ SSH_CONNECTION: "x", WT_SESSION: "1" }, "linux"), "osc777");
	assert.equal(notificationRoute({ ET_VERSION: "1" }, "linux"), "osc777");
});
