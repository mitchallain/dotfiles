/**
 * Handoff extension
 *
 * /handoff              — model writes HANDOFF.md → new session → editor pre-filled to read it
 * /handoff <goal>       — same, with a focus for the new session
 */

import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";
import { existsSync } from "fs";
import { join } from "path";

export default function (pi: ExtensionAPI) {
	pi.registerCommand("handoff", {
		description: "Write HANDOFF.md, new session, read it",
		handler: async (args, ctx) => {
			if (!ctx.hasUI) {
				ctx.ui.notify("handoff requires interactive mode", "error");
				return;
			}

			const goal = args.trim();

			// Step 1: Ask the model to write HANDOFF.md
			pi.sendUserMessage(
				"Write a HANDOFF.md for this session. Be thorough — it will be used to begin a fresh session.",
			);

			// Step 2: Wait for the model to finish
			await new Promise<void>((resolve) => {
				const unsub = pi.on("agent_settled", async () => {
					unsub();
					resolve();
				});
			});

			// Step 3: Check it exists
			const handoffPath = join(ctx.cwd, "HANDOFF.md");
			if (!existsSync(handoffPath)) {
				ctx.ui.notify(
					"Model didn't write HANDOFF.md — try again",
					"warning",
				);
				return;
			}

			// Step 4: New session — pre-fill editor (pi is stale after replacement,
			// so we can't sendUserMessage — just set editor text for one Enter)
			const currentSessionFile = ctx.sessionManager.getSessionFile();
			const readPrompt = `Read HANDOFF.md in the current directory. It contains the full context from the previous session. Continue from where we left off.${goal ? ` Focus on: ${goal}` : ""}`;

			await ctx.newSession({
				parentSession: currentSessionFile,
				withSession: async (replacementCtx) => {
					replacementCtx.ui.setEditorText(readPrompt);
					replacementCtx.ui.notify(
						"HANDOFF.md ready — press Enter to continue",
						"info",
					);
				},
			});
		},
	});
}
