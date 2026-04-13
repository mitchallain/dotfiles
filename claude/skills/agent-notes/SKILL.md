---
name: agent-notes
description: "Save, retrieve, and research knowledge in the agent notes knowledge graph. Use when asked to remember something, save what was learned, look up past findings, capture session context, or research a URL/topic. Triggers on: 'agent notes', 'save this to notes', 'remember this', 'what did we learn about', 'notes about', 'can you remember', 'save to notes', 'check notes for', 'write this down', 'note this', 'research this', 'what does this article say', 'read this link'."
---

# Agent Notes

Save and retrieve knowledge in the agent notes knowledge graph.

**Notes root:** `/home/mallain/notes/12-agent-notes/`

## Operations

**Save** — capture knowledge, learnings, or session context as a note in `01-notes/`.
**Recall** — search existing notes and return what's relevant.
**Research** — fetch a URL, recall related existing notes, discuss takeaways, file what's worth keeping.

## Directory Structure

Knowledge notes live in `01-notes/`, organized by topic. The structure is flexible
and `/agent-tidy` will reorganize over time. Projects live in `00-projects/` and
are managed by `/agent-tasks` — but you should search them too when recalling.

```
{notes_root}/
├── 00-projects/         # Projects (managed by /agent-tasks, searchable)
├── 01-notes/            # Knowledge notes (this skill)
│   ├── _index.md        # Content catalog (auto-maintained)
│   ├── {topic}/         # Topic directories, created as needed
│   │   └── {note}.md
│   └── {note}.md        # Top-level notes are fine too
├── 02-archive/          # Completed work (searchable)
└── log.md               # Append-only activity log
```

## Note Format

```markdown
---
title: Configure PostgreSQL Connection Pooling
type: note
created: 2026-03-30
tags: [postgres, connection-pooling, pgbouncer]
---
# Configure PostgreSQL Connection Pooling

Content goes here. Be specific and actionable — write something
useful to a future reader, not a vague summary.

Include code examples, commands, or concrete steps when applicable.

Link to related knowledge: [[postgres-tuning-guide]], [[connection-timeout-handling]].
```

**Required frontmatter:** `title`, `type`, `created`
**Optional:** `tags`, `source` (where the knowledge came from)

**Type values:** `note` (general knowledge), `session` (session capture),
`howto` (procedure/recipe), `decision` (recording a decision and rationale),
`research` (derived from a source URL)

**Research notes additionally:** `source` (the URL the knowledge came from)

## Saving Knowledge

When asked to save or remember something:

1. **Determine the topic.** What is this knowledge about?
2. **Search for existing notes** on the topic — grep `01-notes/` and `00-projects/`
   for key terms. If a relevant note already exists, **update it** rather than
   creating a duplicate.
3. **Choose a filename.** Descriptive, kebab-case, globally unique across the vault.
   Check for collisions with a glob search before creating.
4. **Pick or create a topic directory** in `01-notes/`. Use an existing topic if it
   fits, or create a new one. Don't over-organize — `/agent-tidy` handles that.
5. **Write the note.** Include:
   - Clear, actionable content
   - `[[wikilinks]]` to all related notes (search for them)
   - Code examples, commands, or steps where applicable
6. **Add backlinks.** If existing notes should link to this new note, edit them
   to add the `[[wikilink]]`.
7. **Update the index** — add the new note to `01-notes/_index.md`.
8. **Log the save** — append to `log.md`.

## Recalling Knowledge

When asked to recall or look up something:

1. **Search broadly** — grep `01-notes/`, `00-projects/`, and `02-archive/` for
   relevant terms. Try multiple search terms if the first doesn't match.
2. **Read matching notes** and synthesize the answer.
3. **Cite sources** using `[[wikilinks]]` so the user can follow up.
4. If nothing is found, say so. Don't guess.
5. **Offer to file back.** If the synthesis is substantial or connects ideas in
   a new way, ask the user: "Want me to file this as a note?" Good answers
   should compound in the wiki, not vanish into chat history.
6. **Log the recall** — append to `log.md`.

## Researching a Source

When the user provides a URL or asks to research a link:

1. **Fetch the URL** using WebFetch. Extract key information.
2. **Recall related notes** — search `01-notes/` and `00-projects/` for terms
   from the source. Read matching notes to understand existing knowledge.
3. **Discuss with the user.** Present key takeaways from the source and how they
   relate to (or contradict) existing notes. Don't auto-file — let the user
   direct what's worth keeping.
4. **File what the user approves.** Create or update notes in `01-notes/` with:
   - `type: research` and `source: {URL}` in frontmatter
   - `[[wikilinks]]` to all related existing notes
   - Backlinks from existing notes to the new one
5. **Update the index** — add new entries to `01-notes/_index.md`.
6. **Log the research** — append to `log.md`.

## Session Capture

When asked to capture what happened in a session:

1. Create a note with `type: session` and a date-prefixed filename
   (e.g., `2026-03-30-agent-notes-redesign.md`).
2. Summarize: what was done, key decisions made, what changed, what's next.
3. Link to all projects and notes that were touched or discussed.
4. **Update the index** — add the session note to `01-notes/_index.md`.
5. **Log the session** — append to `log.md`.

## Activity Log

`{notes_root}/log.md` is an append-only chronological record of all wiki activity.
Shared across all agent skills (`/agent-notes`, `/agent-tasks`, `/agent-tidy`).

### Format

```markdown
# Activity Log

## [2026-04-06] save | Configure PostgreSQL Connection Pooling
Filed new note in `01-notes/postgres/`. Links: [[postgres-tuning-guide]], [[connection-timeout-handling]].

## [2026-04-06] recall | pytest fixture scoping
Searched for pytest fixture scope rules. Found [[pytest-fixture-scoping]].

## [2026-04-06] research | https://example.com/article
Key takeaways discussed. Filed [[article-summary]] in `01-notes/topic/`.
```

Each entry starts with `## [{date}] {operation} | {subject}` for grep parseability.
Operations: `save`, `recall`, `research`, `session`, `create-project`, `add-task`,
`complete-task`, `archive`, `tidy`.

Append to the end of the file. Create `log.md` if it doesn't exist, starting with
`# Activity Log\n`.

## Notes Index

`01-notes/_index.md` is a content catalog of all knowledge notes, organized by
topic directory. Updated on every save and research operation.

### Format

```markdown
# Notes Index

## docker

- [[multi-stage-build-caching]] — How to leverage Docker layer caching in multi-stage builds

## python

- [[virtualenv-vs-venv-tradeoffs]] — When to use virtualenv vs built-in venv
- [[pytest-fixture-scoping]] — How pytest fixture scopes affect test isolation
```

Each entry: `- [[filename]] — one-line summary`. Grouped under `## {topic}` headings
matching the directory names in `01-notes/`. Top-level notes (no topic directory) go
under `## general`.

When adding a note, insert it alphabetically within its topic section. Create a new
topic section if needed.

## Permissions

- **Read:** any file under `/home/mallain/notes/`
- **Write:** only within the notes root
