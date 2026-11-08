import assert from "node:assert/strict";
import { test } from "node:test";
import type { ExtensionAPI, ExtensionContext } from "@earendil-works/pi-coding-agent";
import { visibleWidth } from "@earendil-works/pi-tui";
import extension, { formatTokens, layoutFooter } from "./index.ts";

const stripAnsi = (text: string) => text.replace(/\x1b\[[0-9;]*m/g, "");
const colored = (text: string) => `\x1b[36m${text}\x1b[39m`;

test("formats raw token counts", () => {
	for (const [tokens, expected] of [[0, "0"], [999, "999"], [1234, "1.2k"], [42000, "42k"], [200000, "200k"], [1000000, "1.0M"]] as const) {
		assert.equal(formatTokens(tokens), expected);
	}
});

test("truncates the path before the model, preserving inline context", () => {
	const line = layoutFooter("user@host ~/a/very/long/path (branch)", "[GPT · high]", "42k/200k", 40);
	assert.equal(visibleWidth(line), 40);
	assert.match(stripAnsi(line), /\.\.\. \[GPT · high\]  42k\/200k$/);
	assert.equal(stripAnsi(layoutFooter("pwd", "[model]", "42k/200k", 17)), "[model]  42k/200k");
});

test("keeps the entire status left-aligned with context immediately after the model", () => {
	for (const decorate of [(s: string) => s, colored]) {
		assert.equal(
			stripAnsi(layoutFooter(decorate("user@host ~/dotfiles (master)"), decorate("[GPT · high]"), decorate("42k/200k"), 160)),
			"user@host ~/dotfiles (master) [GPT · high]  42k/200k",
		);
		assert.equal(stripAnsi(layoutFooter("pwd", "[model]", decorate("42k/200k"), 9)), "42k/200k");
	}
});

test("keeps context visible at every width that can fit it, including ANSI colors", () => {
	for (const decorate of [(s: string) => s, colored]) {
		for (let width = 0; width <= 160; width++) {
			const line = layoutFooter(decorate("user@host ~/long/path (branch)"), decorate("[GPT · high]"), decorate("42k/200k"), width);
			assert.ok(visibleWidth(line) <= width, `overflow at width ${width}`);
			if (width >= 8) assert.ok(stripAnsi(line).endsWith("42k/200k"), `missing context at width ${width}`);
		}
	}
});

test("sanitizes multi-line identity and model names", () => {
	const line = layoutFooter("user\npath", "[model\tname]", "0/200k", 80);
	assert.ok(!/[\r\n\t]/.test(line));
});

function fixture() {
	type Footer = { render(width: number): string[]; dispose(): void };
	const handlers = new Map<string, (_event: unknown, ctx: ExtensionContext) => void>();
	let footer: Footer | undefined;
	let usage: { tokens: number | null; contextWindow: number; percent: number | null } | undefined;
	let renders = 0;
	let unsubscribed = false;
	const colors: string[] = [];
	const pi = {
		on(name: string, handler: (_event: unknown, ctx: ExtensionContext) => void) { handlers.set(name, handler); },
		getThinkingLevel: () => "high",
	} as unknown as ExtensionAPI;
	const ctx = {
		mode: "tui",
		model: { id: "gpt-test", name: "GPT (Gateway)", reasoning: true, contextWindow: 200000 },
		sessionManager: { getCwd: () => "/a/very/long/path" },
		getContextUsage: () => usage,
		ui: {
			setFooter(factory: (tui: unknown, theme: unknown, data: unknown) => Footer) {
				footer = factory(
					{ requestRender() { renders++; } },
					{ fg(color: string, text: string) { colors.push(color); return colored(text); } },
					{ getGitBranch: () => "branch", onBranchChange: () => () => { unsubscribed = true; } },
				);
			},
		},
	} as unknown as ExtensionContext;
	extension(pi);
	handlers.get("session_start")!({}, ctx);
	return {
		footer: footer!, colors,
		setUsage(value: typeof usage) { usage = value; },
		emit(name: string) { handlers.get(name)!({}, ctx); },
		get renders() { return renders; },
		get unsubscribed() { return unsubscribed; },
	};
}

test("renders live context, fallback placeholders, gutters and the blank row", () => {
	const f = fixture();
	assert.ok(stripAnsi(f.footer.render(80)[0]).trimEnd().endsWith("?/200k"));
	f.setUsage({ tokens: 42000, contextWindow: 200000, percent: 21 });
	for (let width = 0; width <= 160; width++) {
		const lines = f.footer.render(width);
		assert.equal(lines[1], "");
		assert.ok(visibleWidth(lines[0]) <= width, `overflow at pane width ${width}`);
		if (width >= 10) assert.ok(stripAnsi(lines[0]).trimEnd().endsWith("42k/200k"));
	}
	f.setUsage({ tokens: null, contextWindow: 1000000, percent: null });
	assert.ok(stripAnsi(f.footer.render(80)[0]).trimEnd().endsWith("?/1.0M"));
});

test("renders a devshell snowflake before the model without crowding out context", () => {
	const previous = process.env.IN_NIX_SHELL;
	try {
		for (const value of [undefined, "", "pure", "impure"]) {
			if (value === undefined) delete process.env.IN_NIX_SHELL;
			else process.env.IN_NIX_SHELL = value;
			const f = fixture();
			f.setUsage({ tokens: 42000, contextWindow: 200000, percent: 21 });
			const line = stripAnsi(f.footer.render(160)[0]);
			assert.equal(line.includes("❄ [GPT · high]"), !!value);
			for (let width = 0; width <= 160; width++) {
				const rows = f.footer.render(width);
				assert.ok(visibleWidth(rows[0]) <= width);
				if (width >= 10) assert.ok(stripAnsi(rows[0]).trimEnd().endsWith("42k/200k"));
			}
			f.footer.dispose();
		}
	} finally {
		if (previous === undefined) delete process.env.IN_NIX_SHELL;
		else process.env.IN_NIX_SHELL = previous;
	}
});

test("warns as context fills and clears warnings after usage drops", () => {
	const f = fixture();
	for (const [percent, expected] of [[70, "muted"], [71, "warning"], [90, "warning"], [91, "error"], [20, "muted"]] as const) {
		f.setUsage({ tokens: percent * 2000, contextWindow: 200000, percent });
		f.colors.length = 0;
		f.footer.render(80);
		assert.equal(f.colors[0], expected);
	}
});

test("rerenders on model/thinking changes and cleans up the mounted footer", () => {
	const f = fixture();
	f.emit("model_select");
	f.emit("thinking_level_select");
	assert.equal(f.renders, 2);
	f.footer.dispose();
	assert.equal(f.unsubscribed, true);
	f.emit("model_select");
	assert.equal(f.renders, 2);
});
