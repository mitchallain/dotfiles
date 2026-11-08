/**
 * Session name badge + auto-naming.
 *
 * Prompt styling and naming:
 *
 *   1. Renders a Claude Code-style label on the top-right of the input
 *      box's top border, painted in the same color as the border so it
 *      always matches (focused/unfocused, theme-aware). Technique from
 *      the upstream `border-status-editor.ts` example. A small `❯` prompt
 *      marker uses the existing left padding without moving the text/cursor.
 *
 *   2. Auto-generates a kebab-case session name when one isn't set
 *      explicitly, refreshing it every few user-input cycles so that a
 *      shifting topic gets reflected. Weighted toward the most recent
 *      messages (only the tail of the branch is fed to the LLM).
 *
 *      Hands off to the user as soon as we detect a manual name (set
 *      via pi's built-in `/name` or any other extension) — we never
 *      overwrite a human-set name. State resets per session.
 *
 * pi already shows the name in the session selector, and this
 * extension paints it on the input box's top border.
 */

import { watch, type FSWatcher } from "node:fs";
import { mkdir, readFile, rm, writeFile } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";

import type { AgentMessage } from "@earendil-works/pi-agent-core";
import { complete, type Message } from "@earendil-works/pi-ai";
import {
	convertToLlm,
	CustomEditor,
	type ExtensionAPI,
	type KeybindingsManager,
	serializeConversation,
	type SessionEntry,
} from "@earendil-works/pi-coding-agent";
import {
	CURSOR_MARKER,
	type EditorTheme,
	truncateToWidth,
	type TUI,
	visibleWidth,
} from "@earendil-works/pi-tui";

import {
	createInlineSkillAutocompleteProvider,
	extractMidPromptSlashPrefix,
	transformPromptWithInlineSkills,
} from "./claude-skills/inline-skills.ts";

// ---------- focus FSM (cross-extension singleton) ----------
//
// The focus.ts extension publishes a tiny state machine on globalThis under
// Symbol.for("pi-focus/fsm"). We read it here to paint a state indicator
// into the session-name badge. Lazy (read inside handlers / render) so we
// don't depend on extension load order.

type FocusState = "idle" | "in_progress" | "notify";

interface FocusFSMHandle {
	readonly state: FocusState;
	on(listener: (next: FocusState, prev: FocusState) => void): () => void;
}

const FOCUS_FSM_KEY = Symbol.for("pi-focus/fsm");

function getFocusFSM(): FocusFSMHandle | null {
	const inst = (globalThis as Record<symbol, unknown>)[FOCUS_FSM_KEY];
	return (inst as FocusFSMHandle | undefined) ?? null;
}

function focusIndicator(state: FocusState): string {
	switch (state) {
		case "notify":
			return "\u25CF "; // ● filled circle, reads as "unread"
		case "in_progress":
			return "\u00B7 "; // · middle dot, low-contrast "working"
		case "idle":
			return "";
		default:
			return "";
	}
}

// ---------- tunables ----------

/** Regenerate the name every Nth agent_end after the initial naming. */
const AUTO_REGEN_EVERY_AGENT_ENDS = 3;
/** Tail size: how many recent messages to feed the namer. */
const RECENT_MESSAGE_COUNT = 8;
/** Hard cap on the generated name length (display + kebab cleanup). */
const NAME_MAX_CHARS = 40;
/** Minimum branch size before we bother naming. */
const MIN_MESSAGES_TO_NAME = 2;

const NAMING_SYSTEM_PROMPT = `You are a session-naming assistant for a coding agent.

Given the recent messages of a session, output a short, descriptive name capturing the *current* topic of conversation.

Rules:
- Output ONLY the name. No quotes, no punctuation, no preamble, no explanation.
- Use kebab-case (lowercase words separated by hyphens).
- 3 to 6 words, ${NAME_MAX_CHARS} characters max.
- Weight strongly toward the most recent messages. If the topic shifted, follow the shift — don't anchor on the first message.
- Prefer concrete subjects (file names, feature names, bug names) over generic words like "help", "task", "chat", "session".
- Lowercase, except keep recognised acronyms uppercase (e.g. "fix-jwt-validation", "reorg-CI-pipeline").`;

// ---------- border helpers ----------

/**
 * Splice `right` into the right side of a horizontal border line of the
 * given width. Truncates `right` if it can't fit while preserving a
 * small trailing dash gap.
 *
 * Layout: `─` + filler `─…─` + right + `─`
 */
function borderWithRight(
	right: string,
	width: number,
	border: (text: string) => string,
): string {
	if (width <= 0) return "";
	if (width === 1) return border("─");

	const fixed = 2; // leading + trailing ─
	const minGap = 3;

	let rightText = right;
	while (
		fixed + visibleWidth(rightText) + minGap > width &&
		visibleWidth(rightText) > 0
	) {
		rightText = truncateToWidth(rightText, Math.max(0, visibleWidth(rightText) - 1), "");
	}

	const gap = Math.max(0, width - fixed - visibleWidth(rightText));
	return `${border("─")}${border("─".repeat(gap))}${rightText}${border("─")}`;
}

// ---------- message helpers ----------

function entryToMessage(entry: SessionEntry): AgentMessage | undefined {
	if (entry.type === "message") return entry.message;
	if (entry.type === "compaction") {
		return {
			role: "compactionSummary",
			summary: entry.summary,
			tokensBefore: entry.tokensBefore,
			timestamp: new Date(entry.timestamp).getTime(),
		};
	}
	return undefined;
}

function recentMessages(branch: SessionEntry[], n: number): AgentMessage[] {
	const out: AgentMessage[] = [];
	for (let i = branch.length - 1; i >= 0 && out.length < n; i--) {
		const m = entryToMessage(branch[i]);
		if (m) out.unshift(m);
	}
	return out;
}

/**
 * Coerce LLM output into a clean kebab-case name. Returns null if the
 * result would be too short / unusable.
 */
function sanitizeName(raw: string): string | null {
	let s = raw.trim();
	// Take first non-empty line only.
	s = s.split(/\r?\n/).map((l) => l.trim()).find((l) => l.length > 0) ?? "";
	// Strip surrounding quotes / backticks.
	s = s.replace(/^["'`]+|["'`]+$/g, "");
	// Drop common preambles like "Name: foo".
	s = s.replace(/^(name|title|session)\s*[:\-—]\s*/i, "");
	// Whitespace -> hyphens.
	s = s.replace(/\s+/g, "-");
	// Drop characters that don't belong in a name.
	s = s.replace(/[^A-Za-z0-9_\-./]/g, "");
	// Collapse runs of hyphens.
	s = s.replace(/-{2,}/g, "-").replace(/^-+|-+$/g, "");
	// Length cap.
	if (s.length > NAME_MAX_CHARS) {
		s = s.slice(0, NAME_MAX_CHARS).replace(/-+$/, "");
	}
	if (s.length < 2) return null;
	return s;
}

// ---------- tmux pane focus cursor ----------

const TMUX_PANE = process.env.TMUX_PANE;
const TMUX_HOOK_ID = process.pid;
const TMUX_STATE_DIR = join(tmpdir(), "pi-tmux-pane-cursor");
const TMUX_STATE_FILE = TMUX_PANE
	? join(TMUX_STATE_DIR, `pane-${TMUX_PANE.replace(/[^a-zA-Z0-9_.-]/g, "_")}-${TMUX_HOOK_ID}.state`)
	: undefined;
const TMUX_TIMEOUT_MS = 500;

function shellQuote(value: string): string {
	return `'${value.replace(/'/g, `'\\''`)}'`;
}

function stripFakeCursor(line: string): string {
	const markerIndex = line.indexOf(CURSOR_MARKER);
	if (markerIndex === -1) return line;

	const withoutMarker = line.replace(CURSOR_MARKER, "");
	const afterMarker = withoutMarker.slice(markerIndex);
	const match = afterMarker.match(/^\x1b\[7m([\s\S])\x1b\[0m/);
	if (!match) return withoutMarker;

	const highlightedChar = match[1] ?? "";
	const trailingText = afterMarker.slice(match[0].length);
	const replacement = highlightedChar === " " && /^\s*$/.test(trailingText) ? "" : highlightedChar;

	return withoutMarker.slice(0, markerIndex) + replacement + trailingText;
}

// ---------- extension ----------

export default function (pi: ExtensionAPI) {
	// Per-session state. Reset on session_start.
	let lastAutoSetName: string | null = null;
	let userOverrode = false;
	let agentEndsSinceLastGen = 0;
	let generating = false;
	let activeTui: TUI | undefined;
	let unsubscribeFocus: (() => void) | null = null;

	// Tmux pane focus state.
	let isTmuxPaneFocused = true;
	let tmuxHooksInstalled = false;
	let tmuxStateWatcher: FSWatcher | undefined;
	let currentEditor: SessionNameBorderEditor | null = null;

	async function runTmux(args: string[]): Promise<void> {
		await pi.exec("tmux", args, { timeout: TMUX_TIMEOUT_MS });
	}

	async function applyTmuxState(): Promise<void> {
		if (!TMUX_STATE_FILE) return;
		try {
			const nextFocused = (await readFile(TMUX_STATE_FILE, "utf8")).trim() !== "0";
			if (nextFocused === isTmuxPaneFocused) return;
			isTmuxPaneFocused = nextFocused;
			currentEditor?.setTmuxPaneFocused(nextFocused);
		} catch {
			// Ignore transient read errors.
		}
	}

	function tmuxHookCommand(value: "0" | "1"): string {
		return `run-shell -b \"printf %s ${value} > ${shellQuote(TMUX_STATE_FILE!)}\"`;  
	}

	async function installTmuxHooks(): Promise<void> {
		if (!TMUX_PANE || !TMUX_STATE_FILE || tmuxHooksInstalled) return;
		await mkdir(TMUX_STATE_DIR, { recursive: true });
		await writeFile(TMUX_STATE_FILE, isTmuxPaneFocused ? "1" : "0");
		await runTmux(["set-hook", "-p", "-t", TMUX_PANE, `pane-focus-in[${TMUX_HOOK_ID}]`, tmuxHookCommand("1")]);
		await runTmux(["set-hook", "-p", "-t", TMUX_PANE, `pane-focus-out[${TMUX_HOOK_ID}]`, tmuxHookCommand("0")]);
		tmuxHooksInstalled = true;
	}

	async function uninstallTmuxHooks(): Promise<void> {
		if (!TMUX_PANE || !tmuxHooksInstalled) return;
		await Promise.allSettled([
			runTmux(["set-hook", "-up", "-t", TMUX_PANE, `pane-focus-in[${TMUX_HOOK_ID}]`]),
			runTmux(["set-hook", "-up", "-t", TMUX_PANE, `pane-focus-out[${TMUX_HOOK_ID}]`]),
		]);
		tmuxHooksInstalled = false;
	}

	async function startTmuxWatching(): Promise<void> {
		if (!TMUX_STATE_FILE || tmuxStateWatcher) return;
		tmuxStateWatcher = watch(TMUX_STATE_FILE, { persistent: false }, () => {
			void applyTmuxState();
		});
		tmuxStateWatcher.on("error", () => {
			tmuxStateWatcher?.close();
			tmuxStateWatcher = undefined;
		});
	}

	async function stopTmuxWatching(): Promise<void> {
		tmuxStateWatcher?.close();
		tmuxStateWatcher = undefined;
		if (!TMUX_STATE_FILE) return;
		await rm(TMUX_STATE_FILE, { force: true });
	}

	async function startTmuxMonitoring(): Promise<void> {
		if (!TMUX_PANE || !TMUX_STATE_FILE) return;
		await installTmuxHooks();
		await startTmuxWatching();
		await applyTmuxState();
	}

	async function stopTmuxMonitoring(): Promise<void> {
		await uninstallTmuxHooks();
		await stopTmuxWatching();
	}

	const resetSessionState = (currentName: string | null) => {
		lastAutoSetName = null;
		// If the session already has a name when it starts (e.g. a resumed
		// session that the user previously named), treat it as user-set
		// and don't auto-rename.
		userOverrode = !!currentName;
		agentEndsSinceLastGen = 0;
		generating = false;
	};

	pi.on("session_start", (_event, ctx) => {
		resetSessionState(pi.getSessionName() ?? null);

		// Subscribe to the focus FSM so transitions trigger a re-render of
		// the badge. Done in session_start (not at module load) so we don't
		// depend on extension load order — by now focus.ts has run and
		// populated globalThis.
		unsubscribeFocus?.();
		unsubscribeFocus = null;
		const focusFsm = getFocusFSM();
		if (focusFsm) {
			unsubscribeFocus = focusFsm.on(() => activeTui?.requestRender());
		}

		class SessionNameBorderEditor extends CustomEditor {
			private tmuxPaneFocused = true;

			constructor(tui: TUI, theme: EditorTheme, keybindings: KeybindingsManager) {
				super(tui, theme, keybindings);
				activeTui = tui;
			}

			setTmuxPaneFocused(focused: boolean): void {
				if (this.tmuxPaneFocused === focused) return;
				this.tmuxPaneFocused = focused;
				this.invalidate();
				this.tui.requestRender();
			}

			// Allow `/` after whitespace (or at start of line 2+) to open skill
			// autocomplete, not just `/` at the very beginning of line 0.
			isAtStartOfMessage(): boolean {
				const cursor = this.getCursor();
				const line = this.getLines()[cursor.line] ?? "";
				const beforeCursor = line.slice(0, cursor.col);
				return super.isAtStartOfMessage() || /(?:^|\s)\/?$/.test(beforeCursor);
			}

			isInSlashCommandContext(textBeforeCursor: string): boolean {
				return (
					super.isInSlashCommandContext(textBeforeCursor) ||
					/(?:^|\s)\/[^\s/]*$/.test(textBeforeCursor)
				);
			}

			insertCharacter(char: string, skipUndoCoalescing?: boolean): void {
				super.insertCharacter(char, skipUndoCoalescing);
				if (!this.isShowingAutocomplete() && char === ":") {
					const cursor = this.getCursor();
					const line = this.getLines()[cursor.line] ?? "";
					const textBeforeCursor = line.slice(0, cursor.col);
					if (this.isInSlashCommandContext(textBeforeCursor)) {
						(this as unknown as { tryTriggerAutocomplete(): void }).tryTriggerAutocomplete();
					}
				}
			}

			handleTabCompletion(): void {
				const cursor = this.getCursor();
				if (extractMidPromptSlashPrefix(this.getLines(), cursor.line, cursor.col) !== null) {
					(this as unknown as { handleSlashCommandCompletion(): void }).handleSlashCommandCompletion();
					return;
				}
				super.handleTabCompletion();
			}

			handleInput(data: string): void {
				const cursor = this.getCursor();
				const midPromptSlash =
					this.isShowingAutocomplete() &&
					extractMidPromptSlashPrefix(this.getLines(), cursor.line, cursor.col) !== null;
				if (!midPromptSlash) {
					super.handleInput(data);
					return;
				}
				// Base Editor auto-submits the message on Enter when completing a `/`
				// prefix. For mid-prompt skill completions, Enter should only insert
				// the completion so the user can keep typing.
				const prevDisableSubmit = this.disableSubmit;
				this.disableSubmit = true;
				try {
					super.handleInput(data);
				} finally {
					this.disableSubmit = prevDisableSubmit;
				}
				if (!this.isShowingAutocomplete()) {
					this.onChange?.(this.getText());
				}
			}

			render(width: number): string[] {
				let lines = super.render(width);
				if (lines.length < 2) return lines;

				// Strip fake cursor when tmux pane is not focused.
				if (!this.tmuxPaneFocused) {
					lines = lines.map(stripFakeCursor);
				}

				// Paint the marker inside the existing gutter, leaving layout,
				// cursor markers, wrapped lines, and autocomplete untouched.
				const padding = Math.min(this.getPaddingX(), Math.max(0, Math.floor((width - 1) / 2)));
				if (padding >= 2) {
					lines[1] = " ".repeat(padding - 2) + this.borderColor("❯") + " " + lines[1].slice(padding);
				}

				const name = pi.getSessionName();
				const focusFsm = getFocusFSM();
				const indicator = focusFsm ? focusIndicator(focusFsm.state) : "";

				// Show the badge if we have a name OR a non-idle indicator.
				// (An indicator alone in a fresh, unnamed session still tells
				// you whether the agent is working / done.)
				if (!name && !indicator) return lines;

				const borderColor = (text: string) => this.borderColor(text);
				const label = borderColor(` ${indicator}${name ?? ""} `.replace(/\s+$/g, " "));

				lines[0] = borderWithRight(label, width, borderColor);
				return lines;
			}
		}

		ctx.ui.setEditorComponent(
			(tui, theme, keybindings) => {
				const editor = new SessionNameBorderEditor(tui, theme, keybindings);
				editor.setTmuxPaneFocused(isTmuxPaneFocused);
				currentEditor = editor;
				return editor;
			},
		);

		if (typeof ctx.ui.addAutocompleteProvider === "function") {
			ctx.ui.addAutocompleteProvider((current) =>
				createInlineSkillAutocompleteProvider(current, () => {
					if (pi.getSettings().enableSkillCommands === false) return [];
					return pi.getCommands().filter((cmd) => cmd.source === "skill");
				}),
			);
		}

		// Start tmux pane focus monitoring.
		if (TMUX_PANE) {
			void startTmuxMonitoring();
		}
	});

	pi.on("input", (event) => {
		const commands = pi.getCommands();
		const skills = commands.filter((cmd) => cmd.source === "skill");
		const nonSkillCommands = new Set(
			commands.filter((cmd) => cmd.source !== "skill").map((cmd) => cmd.name),
		);
		const transformed = transformPromptWithInlineSkills(event.text, skills, {
			nonSkillCommands,
		});
		if (transformed !== null) {
			return { action: "transform", text: transformed };
		}
	});

	pi.on("session_shutdown", async () => {
		activeTui = undefined;
		currentEditor = null;
		unsubscribeFocus?.();
		unsubscribeFocus = null;
		await stopTmuxMonitoring();
	});

	pi.on("agent_end", async (_event, ctx) => {
		if (generating || userOverrode) return;
		if (!ctx.model) return;

		// Detect manual override: if the current name doesn't match what we
		// last set, the user (or another extension) has taken over naming.
		const current = pi.getSessionName();
		if (current && current !== lastAutoSetName) {
			userOverrode = true;
			return;
		}

		agentEndsSinceLastGen++;
		const isFirstNaming = lastAutoSetName === null;
		if (!isFirstNaming && agentEndsSinceLastGen < AUTO_REGEN_EVERY_AGENT_ENDS) {
			return;
		}

		const branch = ctx.sessionManager.getBranch();
		const messages = recentMessages(branch, RECENT_MESSAGE_COUNT);
		if (messages.length < MIN_MESSAGES_TO_NAME) return;

		generating = true;
		try {
			const auth = await ctx.modelRegistry.getApiKeyAndHeaders(ctx.model);
			if (!auth.ok || !auth.apiKey) return;

			const llmMessages = convertToLlm(messages);
			const conversationText = serializeConversation(llmMessages);

			const userMessage: Message = {
				role: "user",
				content: [
					{
						type: "text",
						text: `## Recent messages (most recent last)\n\n${conversationText}\n\nGenerate the session name now.`,
					},
				],
				timestamp: Date.now(),
			};

			const response = await complete(
				ctx.model,
				{ systemPrompt: NAMING_SYSTEM_PROMPT, messages: [userMessage] },
				{ apiKey: auth.apiKey, headers: auth.headers },
			);

			if (response.stopReason === "aborted") return;

			const text = response.content
				.filter((c): c is { type: "text"; text: string } => c.type === "text")
				.map((c) => c.text)
				.join("\n");

			const name = sanitizeName(text);
			if (!name) return;
			if (name === lastAutoSetName) {
				// No change — still reset the counter so we wait another window.
				agentEndsSinceLastGen = 0;
				return;
			}

			// Re-check for manual override that may have happened during
			// the LLM call.
			const beforeApply = pi.getSessionName();
			if (beforeApply && beforeApply !== lastAutoSetName) {
				userOverrode = true;
				return;
			}

			pi.setSessionName(name);
			lastAutoSetName = name;
			agentEndsSinceLastGen = 0;
			activeTui?.requestRender();
		} catch {
			// Naming is best-effort. Don't surface errors to the user.
		} finally {
			generating = false;
		}
	});
}
