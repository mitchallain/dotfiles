# Focus-aware completion

`../focus.ts` publishes the session-name badge FSM at `Symbol.for("pi-focus/fsm")`.
It owns focus reporting, terminal progress, and the single completion notifier.
This directory contains its helpers/tests, not another auto-loaded extension.

- `agent_start` marks working; only `agent_settled` marks completion. Intermediate
  `agent_end` events, retries, and queued continuations do not announce done.
- Focus/input dismisses attention, not working state. Abort completes silently.
- Only unfocused TUI completions notify, including errors. Focused errors stay
  visible in the transcript without a popup. RPC/print/JSON never attach stdin
  observers, write terminal escapes, or send desktop notifications.
- Notification uses the current branch's last assistant message: project title,
  first nonempty response/error line (160 characters), and tmux window location.
  Response snippets therefore appear in desktop notification history.
- Local Linux uses notify-send with error/success icons and a ten-second expiry.
  Recognized terminals and SSH/ET use terminal notifications so remote processes
  do not notify the server desktop. Existing macOS/Windows routes remain.
- OSC 9;4 progress and badge working/attention states remain; pane-title animation
  and the independent notify-send extension were removed.
- Session-owned subscribers/input observers are removed on shutdown and recreated
  at startup/reload. Pending tmux lookups cannot notify after focus, input, new
  work, or shutdown. Lookup/notification subprocesses have bounded timeouts.

Run helper tests from this directory:

```sh
node --test state.test.ts
```

Tests mock effects, so they never send real desktop notifications. Pi-loaded
smoke checks additionally covered real StdinBuffer focus sequences, startup,
reload, observer cleanup, terminal-output safety, and the session-name editor.

Tmux needs `focus-events on` and `allow-passthrough on` (already configured).
Startup assumes focused until a terminal focus event arrives. This does not
aggregate multiple panes or decorate tmux window names; that future feature is
specified in `../../TODO.md`.
