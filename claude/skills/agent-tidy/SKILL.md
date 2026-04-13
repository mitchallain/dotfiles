---
name: agent-tidy
description: "Housekeeping for the agent notes knowledge graph. Re-organize notes, add missing wikilinks, deduplicate content, find orphan notes, validate structure, regenerate indexes, and lint for content issues (contradictions, stale claims, knowledge gaps). Run on a schedule or on demand. Triggers on: 'agent tidy', 'tidy notes', 'organize notes', 'clean up notes', 'fix links', 'find orphans', 'deduplicate notes', 'reindex notes', 'lint notes', 'check notes health'."
---

# Agent Tidy

Housekeeping for the agent notes knowledge graph.

**Notes root:** `/home/mallain/notes/12-agent-notes/`

## What This Skill Does

Eight maintenance operations, run individually or as a full sweep.
The primary goal is improving connections, reducing redundancy, and
ensuring content health. Reorganization is rare — only suggest moves
when clearly warranted.

1. **Link** — find and add missing `[[wikilinks]]` (primary)
2. **Dedup** — find and merge duplicate content (primary)
3. **Lint** — content-level health checks (primary)
4. **Orphans** — find disconnected notes
5. **Validate** — check structural health
6. **Index** — regenerate `00-projects/_registry.md` and `01-notes/_index.md`
7. **Organize** — restructure directories, move misplaced notes (rare)
8. **Log** — append a tidy summary to `log.md`

## Directory Structure

```
{notes_root}/
├── 00-projects/
│   ├── _registry.md              # Auto-generated project index
│   ├── {NN}-{domain}/
│   │   └── {NN}-{area}/
│   │       └── {NN}-{project}/
│   │           ├── {project}.md  # Unique filename matching directory name
│   │           └── *.md          # Research notes (unique filenames)
├── 01-notes/
│   ├── _index.md                 # Auto-generated content catalog
│   ├── {topic}/
│   │   └── *.md
│   └── *.md
├── 02-archive/
│   └── (mirrors 00-projects/ structure)
└── log.md                        # Append-only activity log (shared)
```

## Full Tidy

Run all checks in order. Report findings first, then apply fixes.
Confirm with the user before destructive changes (merges, moves, deletes).

### 1. Link

For each note:
- Extract key terms from its title and content
- Grep the vault for those terms in other notes
- If another note discusses the same topic but doesn't link here, add `[[wikilink]]`
- Link additions are safe (additive only) — apply without confirmation

### 2. Dedup

- Find notes with similar titles or significant content overlap
- Present duplicates with a summary of what each contains
- If the user confirms, merge into the richer note:
  - Combine content, keeping the more detailed version
  - Update all `[[wikilinks]]` that pointed to the deleted note
  - Delete the duplicate

### 3. Lint

Content-level health checks. Read notes and look for:

- **Contradictions** — two or more notes making conflicting claims about the
  same topic. Report both sides with `[[wikilinks]]` so the user can resolve.
- **Stale claims** — older notes whose assertions have been superseded by
  newer notes (compare `created` dates). Flag with suggested updates.
- **Missing pages** — concepts or terms referenced frequently across multiple
  notes but lacking their own dedicated note. Suggest creating them.
- **Knowledge gaps** — based on the topics covered, suggest questions worth
  investigating or areas where a web search / new source could fill holes.

Present all findings as a report. Contradictions and stale claims are
actionable — offer to fix them with user confirmation. Missing pages and
knowledge gaps are suggestions only.

### 4. Orphans

- Build a link graph: scan all `.md` files for `[[wikilinks]]`
- Report notes with zero incoming links (nothing points to them)
- Report notes with zero outgoing links (they link to nothing)
- Suggest connections based on content overlap

### 5. Validate

- Every `.md` file has frontmatter with at least `title`, `type`, `created`
- Every project note also has `id`, `status`, `priority`, `domain`, `area`
- No two `.md` files share the same filename anywhere in the vault
- `.counter` files contain valid integers
- Counter values >= the highest existing numbered sibling + 1
- Project IDs in frontmatter are consistent with their domain/area path

### 6. Index

Regenerate both indexes by scanning the filesystem:

**`00-projects/_registry.md`** — project registry:

```markdown
# Project Registry

## {Domain Acronym} — {Domain Name}

### {Area Acronym} — {Area Name}

| ID | Project | Status | Priority | Progress |
|----|---------|--------|----------|----------|
| HL-API-003 | [[add-rate-limiting-to-rest-api]] | active | P1 | 1/3 |
```

Read each project's frontmatter for status/priority. Count `[x]` vs `[ ]` lines
for progress.

**`01-notes/_index.md`** — notes content catalog:

```markdown
# Notes Index

## {topic}

- [[{note-name}]] — one-line summary from the note's title or content
```

Scan all `.md` files in `01-notes/`, grouped by topic directory. Each entry gets
a wikilink and a one-line summary. Top-level notes go under `## general`.
Entries sorted alphabetically within each topic section.

### 7. Organize (rare)

Only suggest reorganization when there's a clear structural problem — not
just because a directory has few files. A topic directory with one note is
fine; it will grow over time. Focus on genuine issues:

- **Project status drift**: projects with all tasks `[x]` but status still
  `active` — flag for archival
- **Misplaced files**: notes clearly in the wrong location (e.g., a research
  note orphaned outside its project directory)
- **Overgrown directories**: topic directories with >15 notes that should split
- May move files and create/remove directories, but always confirm first

### 8. Log

After completing a tidy pass, append a summary to `{notes_root}/log.md`:

```markdown
## [2026-04-06] tidy | Full sweep
Links added: 5. Duplicates merged: 1. Contradictions found: 2.
Orphans: 3 (suggested connections). Validation: 2 issues fixed.
Indexes regenerated. Knowledge gaps: suggested 3 new topics.
```

See `/agent-notes` for full `log.md` format specification. Create
`log.md` if it doesn't exist, starting with `# Activity Log\n`.

## Permissions

- **Read:** any file under `/home/mallain/notes/`
- **Write:** only within the notes root
- **May move and delete files** within the notes root
- **Always confirm** before merging or deleting notes
