---
name: claude-todos
description: "Manual-invocation only via /claude-todos slash command. Finds and completes TODO(claude): markers in the specified files or directories. Do not auto-invoke — this skill is triggered explicitly by the user through its slash command and should not be used in response to generic requests about TODOs, code cleanup, or task completion."
---

# claude-todos

Find every `TODO(claude):` marker in the paths given as arguments, complete each one, then remove the marker.

## Arguments

The user invokes this as `/claude-todos [<path> ...]`. Each argument is a file or directory. If no paths are given, scan the current working directory recursively.

## Workflow

1. **Find the markers.** Run `rg -n 'TODO\(claude\):' <paths>` to list every matching line with its file and line number (omit `<paths>` to scan the working directory). Use `rg` over `grep` so `.gitignore` is respected. If `rg` isn't available, fall back to `grep -rn 'TODO(claude):' <paths-or-dot>`.

2. **Tell the user what you found.** Print a short list: file:line — the TODO text. If there are none, say so and stop.

3. **Work through them one at a time.** For each marker:
   - Read enough surrounding code to understand the intent. The TODO text is often terse ("handle empty input", "use the new API") and assumes context from the nearby code.
   - If the TODO is ambiguous or the completion involves a non-obvious choice (which library, which error behavior, which name), ask the user before implementing. It's fine to batch related questions for a single TODO, but don't guess your way through unclear requirements.
   - Implement the change.
   - Remove the marker. If the `TODO(claude):` is a standalone comment line, delete the whole line. If it's an inline comment trailing code (`foo()  // TODO(claude): rename`), delete just the comment, leaving the code. If the TODO text spanned multiple comment lines, delete all of them.
   - Move on to the next one.

4. **Leave other TODOs alone.** Only `TODO(claude):` is in scope. Plain `TODO:`, `FIXME:`, `TODO(someone-else):`, etc. must not be touched — they belong to humans or other tooling.

5. **Report at the end.** Brief summary: how many TODOs were completed, which files changed. Don't re-paste the diffs; the user can read them.

## Matching rules

- The pattern is literal: `TODO(claude):`. Case-sensitive. The colon is required.
- `TODO(claude)` without the colon is **not** a match — ignore it.
- `// TODO(claude): foo` and `# TODO(claude): foo` both match — the comment syntax doesn't matter, only the marker.

## When to ask vs. when to act

Lean toward asking when:
- The TODO implies a design choice that affects more than the immediate line (naming of a new public function, which error type to raise, which of two equivalent libraries to use).
- The surrounding code doesn't make the intent obvious and you'd be guessing.
- Completing the TODO would touch files the user didn't list.

Lean toward acting when:
- The TODO is mechanical (rename, extract, inline, replace-with-X).
- The fix is fully determined by the surrounding code or a comment right above.
- The user has already answered a similar question earlier in the session — reuse that answer.

## Notes on scope

Don't expand scope. If completing one TODO reveals unrelated problems (a bug two functions away, missing tests elsewhere), mention them in the final report but don't fix them as part of this invocation.
