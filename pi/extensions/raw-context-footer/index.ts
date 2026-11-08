/**
 * Claude Code-style footer: user@host cwd (branch) ❄ [model · thinking]  tokens/window.
 * Keeps everything left-aligned; reserves context space, truncating the path first.
 * Keeps a one-column gutter and a blank row below the status line.
 */

import { homedir, hostname, userInfo } from "node:os";

import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { truncateToWidth, visibleWidth } from "@earendil-works/pi-tui";

/** Strip DNS suffixes and user/asset prefixes on hostnames. */
function getHostInfo(): string {
	const shortHost = hostname().split(".")[0];
	const match = /^[a-z]+-\d+-(.+)$/i.exec(shortHost);
	return match ? match[1] : shortHost;
}

/** Match _dir_chomp in scripts/claude_statusline.sh. */
export function shortenPath(cwd: string, home: string, maxLength = 20): string {
	let path = cwd;
	if (home && (cwd === home || cwd.startsWith(`${home}/`))) {
		path = `~${cwd.slice(home.length)}`;
	}
	const parts = path.split("/");
	for (let i = 1; parts.join("/").length > maxLength && i < parts.length - 1; i++) {
		parts[i] = parts[i].slice(0, parts[i].startsWith(".") ? 2 : 1);
	}
	return parts.join("/");
}

export function formatModelName(model: { id: string; name?: string; contextWindow?: number }): string {
	// Use the ID for Claude versions; display names can omit the minor version.
	const claude = /claude-(opus|sonnet|haiku)-(\d+)(?:-(\d{1,2}))?(?=[^\d]|$)/i.exec(model.id);
	if (claude) {
		const family = claude[1][0].toUpperCase() + claude[1].slice(1).toLowerCase();
		const version = claude[2] + (claude[3] ? `.${claude[3]}` : "");
		const context = model.id.toLowerCase().includes("[1m]") || (model.contextWindow ?? 0) >= 1_000_000;
		return `${family} ${version}${context ? " 1M" : ""}`;
	}
	return (model.name || model.id).replace(/\s*\([^)]*\)\s*$/, "").trim();
}

function singleLine(text: string): string {
	return text.replace(/[\r\n\t]/g, " ");
}

export function formatTokens(count: number): string {
	if (count < 1000) return count.toString();
	if (count < 10000) return `${(count / 1000).toFixed(1)}k`;
	if (count < 1000000) return `${Math.round(count / 1000)}k`;
	if (count < 10000000) return `${(count / 1000000).toFixed(1)}M`;
	return `${Math.round(count / 1000000)}M`;
}

/** Preserve context, then model; spend only the remaining columns on the path. */
export function layoutFooter(identity: string, model: string, context: string, width: number): string {
	identity = singleLine(identity);
	model = singleLine(model);
	const contextWidth = visibleWidth(context);
	if (width <= contextWidth) return truncateToWidth(context, width, "");

	const available = Math.max(0, width - contextWidth - 2);
	const modelWidth = visibleWidth(model);
	let left: string;
	if (available <= modelWidth) {
		left = truncateToWidth(model, available, "...");
	} else {
		const path = truncateToWidth(identity, available - modelWidth - 1, "...");
		left = path ? `${path} ${model}` : model;
	}
	return left ? `${left}  ${context}` : context;
}

export default function (pi: ExtensionAPI) {
	const userHost = `${userInfo().username}@${getHostInfo()}`;
	const home = homedir();
	let currentTui: { requestRender: () => void } | null = null;
	pi.on("thinking_level_select", () => currentTui?.requestRender());
	pi.on("model_select", () => currentTui?.requestRender());

	pi.on("session_start", (_event, ctx) => {
		if (ctx.mode !== "tui") return;
		ctx.ui.setFooter((tui, theme, footerData) => {
			currentTui = tui;
			const unsubBranch = footerData.onBranchChange(() => tui.requestRender());

			return {
				dispose: () => {
					unsubBranch();
					if (currentTui === tui) currentTui = null;
				},
				invalidate() {},
				render(paneWidth: number): string[] {
					const gutter = " ".repeat(Math.min(1, Math.floor(Math.max(0, paneWidth) / 2)));
					const width = Math.max(0, paneWidth - gutter.length * 2);
					const pwd = shortenPath(ctx.sessionManager.getCwd(), home);
					const branch = footerData.getGitBranch();
					const model = ctx.model ? formatModelName(ctx.model) : "no-model";
					const thinking = ctx.model?.reasoning ? ` · ${pi.getThinkingLevel()}` : "";
					const usage = ctx.getContextUsage();
					const contextWindow = usage?.contextWindow ?? ctx.model?.contextWindow ?? 0;
					const tokens = usage?.tokens;
					const contextDisplay = `${tokens == null ? "?" : formatTokens(tokens)}/${contextWindow > 0 ? formatTokens(contextWindow) : "?"}`;
					const percent = usage?.percent ?? 0;
					const context = theme.fg(percent > 90 ? "error" : percent > 70 ? "warning" : "muted", contextDisplay);
					const identity = theme.fg("muted", singleLine(`${userHost} ${pwd}${branch ? ` (${branch})` : ""}`));
					const devshell = process.env.IN_NIX_SHELL ? "❄ " : "";
					const modelDisplay = theme.fg("muted", singleLine(`${devshell}[${model}${thinking}]`));
					const line = layoutFooter(identity, modelDisplay, context, width);

					return [gutter + line + gutter, ""];
				},
			};
		});
	});
}
