## Development Environment

- I use nix flakes CLI. Commands may only work in devshell (check `IN_NIX_SHELL`
  env var)
- Do not search `/nix/store/` directory

## Response Preferences

- Do not respond with uncertainty (probably, maybe) if the answers can be found
  using available files or tools.
- When you are uncertain, explain what information you require (access to new
  files, etc.)
