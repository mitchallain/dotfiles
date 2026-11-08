# Claude skills for Pi

Imports project `.claude/skills/` and skills from **enabled, installed** Claude
plugins. The global extension is already linked into `~/.pi/agent/extensions/`.
Run `/reload`, then type `/skill:` to browse; for example,
`/skill:frontend-design`.

## Installed plugins

- Reads `~/.claude/plugins/installed_plugins.json` (registry version 2) and the
  user `~/.claude/settings.json` `enabledPlugins` mapping.
- With Pi project trust, merges ancestor project `.claude/settings.json` and
  `.claude/settings.local.json` from repo root to cwd. Local settings override
  shared settings at each directory. Project/local installs must match one of
  those project directories; scoped installs override user installs.
- Reads the registered installation only, never stale/uninstalled cache versions.
- Discovers default `skills/` plus additive manifest `skills` paths (string or
  array of directories, including individual skill directories). Paths and skill
  symlinks must stay inside the plugin. Glob/file manifest paths aren't supported.
- Validates YAML, including multiline descriptions and metadata. Conservatively
  repairs simple unquoted `: ` values; malformed skills get warnings and are
  omitted. Existing project-skill normalization remains unchanged.
- Mirrors the plugin layout under a per-process temporary cache, preserving
  relative script/reference paths. Expands `${CLAUDE_PLUGIN_ROOT}` and
  `$CLAUDE_PLUGIN_ROOT` in imported skill text to the registered plugin root.
  Original plugin files are never changed.
- Any `learning-protocol@*` plugin is excluded in Pi because
  the personal `~/.claude/skills/learning-protocol` skill is already loaded.
  Other duplicate names use Pi's usual first-found/collision behavior.

**Skills only:** importing does not load Claude commands, agents, hooks, MCP
servers, LSP configuration, or executable plugin code. A loaded skill may still
instruct the agent to run a script or require tools not available in Pi. Treat
plugin enablement as a trust decision; this is discovery, not a sandbox or a full
Claude compatibility layer. Managed/policy settings and alternate Claude config
roots are not imported.

## Options

- `CLAUDE_SKILLS_INCLUDE_PLUGINS=0`: opt out of plugin imports.
- `CLAUDE_SKILLS_INCLUDE_USER=1`: also discover `~/.claude/skills`; usually leave
  off because Pi settings already load this directory.
- `CLAUDE_SKILLS_NESTED_DEPTH=0`: disable nested project discovery (default 4).
- `CLAUDE_SKILLS_FORCE_NESTED=1`: allow nested discovery outside a git repository.

Project skills/settings always require `ctx.isProjectTrusted()`.

## Dependencies and tests

From this directory, with Node 22.18+ (native TypeScript stripping):

```bash
npm ci --ignore-scripts --legacy-peer-deps
node test.ts
node --test plugins.test.ts inline-skills.test.ts
```

Only `yaml` is installed at runtime; Pi provides the coding-agent API. The legacy
peer flag avoids installing a second copy of Pi just to satisfy the peer type
contract. No plugin servers or hooks are launched by these tests.
