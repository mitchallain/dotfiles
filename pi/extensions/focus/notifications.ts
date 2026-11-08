import { execFile } from "node:child_process";
import { basename } from "node:path";
import { promisify } from "node:util";
import type { SessionEntry } from "@earendil-works/pi-coding-agent";

export interface CompletionNotification {
	title: string;
	body: string;
	isError: boolean;
}

/** Notifications and terminal escape fields must not contain control sequences. */
export function singleLine(text: string): string {
	return text.replace(/[\r\n\t]/g, " ").replace(/[\x00-\x1f\x7f-\x9f]/g, "");
}

export function completionNotification(branch: SessionEntry[], cwd: string): CompletionNotification | null {
	for (let i = branch.length - 1; i >= 0; i--) {
		const entry = branch[i];
		if (entry.type !== "message" || entry.message.role !== "assistant") continue;
		const message = entry.message;
		if (message.stopReason === "aborted") return null;
		const isError = message.stopReason === "error";
		const text = message.content.filter(block => block.type === "text").map(block => block.text).join("\n");
		const first = (text || message.errorMessage || "").split(/\r?\n/).map(line => singleLine(line).trim()).find(Boolean) || "Ready for input";
		return {
			title: `pi${isError ? " error" : ""} — ${singleLine(basename(cwd))}`,
			body: first.length > 160 ? `${first.slice(0, 159)}…` : first,
			isError,
		};
	}
	return null;
}

const execFileAsync = promisify(execFile);
export async function tmuxLocation(): Promise<string | null> {
	if (!process.env.TMUX_PANE) return null;
	try {
		const { stdout } = await execFileAsync("tmux", ["display-message", "-p", "-t", process.env.TMUX_PANE, "#{session_name}:#{window_index}:#{window_name}"], { timeout: 2000 });
		return singleLine(stdout).trim() || null;
	} catch { return null; }
}

export type NotificationRoute = "osc99" | "osc777" | "windows" | "linux" | "mac";
export function notificationRoute(env: NodeJS.ProcessEnv, platform: string): NotificationRoute {
	const term = env.TERM || "";
	if (env.KITTY_WINDOW_ID || term.startsWith("xterm-kitty")) return "osc99";
	if (["ghostty", "iTerm.app", "WezTerm"].includes(env.TERM_PROGRAM || "") || env.GHOSTTY_RESOURCES_DIR || env.ITERM_SESSION_ID || env.WEZTERM_PANE || /^(xterm-ghostty|wezterm|rxvt-unicode)/.test(term)) return "osc777";
	// Unknown SSH/ET terminals must notify the client, never the server desktop.
	if (env.SSH_TTY || env.SSH_CONNECTION || env.SSH_CLIENT || env.ET_VERSION) return "osc777";
	if (env.WT_SESSION) return "windows";
	if (platform === "linux") return "linux";
	if (platform === "darwin") return "mac";
	return "osc777";
}

export function writePassthrough(payload: string): void {
	process.stdout.write(process.env.TMUX ? `\x1bPtmux;${payload.replace(/\x1b/g, "\x1b\x1b")}\x1b\\` : payload);
}

export function notify(notification: CompletionNotification): void {
	const title = singleLine(notification.title);
	const body = singleLine(notification.body);
	const run = (command: string, args: string[]) => {
		execFile(command, args, { timeout: 3000 }, () => { /* best effort */ });
	};
	switch (notificationRoute(process.env, process.platform)) {
		case "osc99":
			writePassthrough(`\x1b]99;i=1:d=0;${title}\x1b\\`);
			writePassthrough(`\x1b]99;i=1:p=body;${body}\x1b\\`);
			break;
		case "osc777":
			writePassthrough(`\x1b]777;notify;${title.replace(/;/g, ",")};${body.replace(/;/g, ",")}\x07`);
			break;
		case "linux":
			run("notify-send", ["--app-name=Pi", `--icon=${notification.isError ? "dialog-error" : "terminal"}`, "--expire-time=10000", "--", title, body]);
			break;
		case "mac":
			run("osascript", ["-e", `display notification ${JSON.stringify(body)} with title ${JSON.stringify(title)}`]);
			break;
		case "windows": {
			const quote = (text: string) => `'${text.replace(/'/g, "''")}'`;
			const type = "Windows.UI.Notifications";
			const manager = `[${type}.ToastNotificationManager, ${type}, ContentType = WindowsRuntime]`;
			run("powershell.exe", ["-NoProfile", "-Command", [
				`${manager} > $null`,
				`$xml = [${type}.ToastNotificationManager]::GetTemplateContent([${type}.ToastTemplateType]::ToastText01)`,
				`$xml.GetElementsByTagName('text')[0].AppendChild($xml.CreateTextNode(${quote(body)})) > $null`,
				`[${type}.ToastNotificationManager]::CreateToastNotifier(${quote(title)}).Show([${type}.ToastNotification]::new($xml))`,
			].join("; ")]);
			break;
		}
	}
}
