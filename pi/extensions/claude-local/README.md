# claude-local

`CLAUDE.local.md` support for pi — a drop-in port of Claude Code's
project-local memory file.

## What it does

Claude Code auto-loads `CLAUDE.local.md` as a personal, usually-gitignored
override that layers on top of the committed `CLAUDE.md`. Pi only
auto-discovers `AGENTS.md` / `CLAUDE.md`, so without this the local file
never reaches the model.

This extension fills that gap:

- **Discovery** — walks from `cwd` up to (and including) the git repo root,
  collecting every `CLAUDE.local.md` along the way. Files are ordered
  outermost-first (repo root) → cwd last, so the most-specific file appears
  last and wins, matching how pi layers `CLAUDE.md`.
- **Not in a repo** — only `cwd` itself is checked. We never walk up to
  `~` and slurp an unrelated personal file.
- **`@imports`** — `@path` references inside a `CLAUDE.local.md` are
  expanded inline (see [Imports](#imports) below).
- **Injection** — discovered files are appended to the system prompt under
  a `## Local Instructions` heading on every `before_agent_start`, with an
  mtime cache so mid-session edits are picked up on the next turn.

## Imports

`@path` expansion reuses the sibling [`at-imports`](../at-imports/) extension's
`expandImports`. We expand here rather than relying on `at-imports`' own
pass: `at-imports` runs first (alphabetical discovery order) and chains off
`event.systemPrompt`, so it never sees text this extension appends later.

Resolution follows `at-imports`' rules, including the caveat that `./` /
`../` imports resolve relative to pi's current working directory, not the
`CLAUDE.local.md` file's directory. Absolute and `~/` paths are unambiguous.

## Discovery boundary

```
<git-root>/CLAUDE.local.md          ← loaded first (least specific)
<git-root>/pkg/CLAUDE.local.md
<cwd>/CLAUDE.local.md               ← loaded last  (most specific, wins)
```

Stops at the git root. Outside a repo, only `<cwd>/CLAUDE.local.md` is read.

## Tests

```bash
cd ~/dotfiles/pi/extensions/claude-local
nix shell nixpkgs#nodejs_22 -c npx -y tsx test.ts
```

The harness exercises the pure logic (discovery, ordering, dedup, the
not-in-a-repo path, `@import` expansion, section assembly) against fixtures
in `./test-fixtures/`, without pi or any of its deps.

## Activating

It's already wired up — `~/.pi/agent/extensions` is a symlink to
`~/dotfiles/pi/extensions/`, and pi auto-discovers
`extensions/*/index.ts`. New pi sessions pick it up automatically; for a
running session, run `/reload` (which re-fires `session_start` and
re-discovers).

## Not included

- User-level `~/.claude/CLAUDE.local.md` — `CLAUDE.local.md` is a
  project-scoped concept; user-global prefs live in `CLAUDE.md` /
  `AGENTS.md` (and reach pi via `at-imports`).
- Trust gating — like the sibling `claude-rules` / `claude-skills`
  extensions, project-local files are read directly. `CLAUDE.local.md` is
  low-risk personal notes by convention.
