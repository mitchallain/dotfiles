# at-imports — `@path/to/file` imports for pi context files

A pi extension that mimics Claude Code's
[memory imports](https://docs.anthropic.com/en/docs/claude-code/memory#memory-imports)
behavior. Pi loads `AGENTS.md` / `CLAUDE.md` files verbatim, so a line like:

```
@~/synced-data/CLAUDE.md
```

gets handed to the model as literal text rather than being expanded inline.
This extension scans the system prompt at the start of every agent turn and
replaces `@path` references with the referenced file's contents.

## Why

I keep my public pi / Claude Code preferences in
`~/dotfiles/claude/CLAUDE.md` and my private,
Syncthing-synced preferences in `~/synced-data/CLAUDE.md`. Claude Code
happily stitches them together via `@~/synced-data/CLAUDE.md`. Pi doesn't —
so without this, the private file never reaches the model.

## Rules

Close to Claude Code's behavior:

| Rule | Behavior |
|---|---|
| Where `@` is matched | Start of line or after whitespace |
| What counts as a "path" | Starts with `/`, `~/`, `./`, or `../` |
| Non-path `@mention`s | Left untouched (email, npm scopes, `@username`) |
| File doesn't exist | Reference left verbatim |
| Inside ` ``` `-fenced code block | Not expanded (code examples stay literal) |
| Recursion | Expanded recursively |
| Cycles | Detected; the offending reference is left literal |
| Depth limit | 5 |
| Caching | Files are re-read only when their `mtime` changes |

## Files

```
at-imports/
├── index.ts               # the extension
├── test.ts                # runs without pi; uses the fixtures below
├── test-fixtures/
│   ├── fixture.md
│   ├── imported.md
│   ├── nested.md
│   ├── inline.md
│   └── should-not-expand.md
└── README.md              # this file
```

## Testing

```bash
cd ~/dotfiles/pi/extensions/at-imports
npx tsx test.ts

# or without npx on PATH:
nix shell nixpkgs#nodejs_22 -c npx -y tsx test.ts
```

The test harness imports `expandImports` directly from `index.ts`, so
anything it catches is a real regression in the extension.

## Activating

It's already wired up — `~/.pi/agent/extensions` is a symlink to
`~/dotfiles/pi/extensions/`, and pi auto-discovers
`extensions/*/index.ts`. New pi sessions pick it up automatically. For a
currently-running session, run `/reload`.

## Caveats

- **Not a full Claude Code parser.** We match `@path` where the path starts
  with `/`, `~/`, `./`, or `../`. Bare `@README.md` style (which Claude Code
  supports) is intentionally skipped to avoid matching `@username` /
  `@scope/pkg` in prose.
- Imports are resolved relative to pi's current working directory (for `./`
  / `../` paths). Absolute and `~/` paths are always unambiguous.
- Runs on every `before_agent_start`. The `mtime` cache makes this cheap,
  but if you edit a referenced file mid-session, the next turn will pick
  up the change automatically.
