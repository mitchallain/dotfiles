# Top level
Before
@./imported.md
After

Some code:
```
@./should-not-expand.md
```

Inline @./inline.md here.

Email foo@bar.com should NOT match.
npm scope @foo/bar should NOT match.

Self-reference (cycle): @./fixture.md end.
