# Project instructions


## Skills to Follow
Before any coding work, read and follow these skills:

- `.agents/skills/karpathy-guidelines/SKILL.md`
- `.agents/skills/isaac-sim-sensor/SKILL.md`
- `.agents/skills/oc-g1-ros/SKILL.md`

## Isaac Sim skills
When planning a task that touches Isaac Sim, pick the matching skills from `_isaacsim/skills/` (the skills
that ship with the installed Isaac Sim, linked by `oc-init-proj`): match the task against
`_isaacsim/skills/SKILLS.md` and each candidate's `description`, read the picked `SKILL.md` files, and name
them in the plan. Check their API claims against the installed Isaac Sim source; some are stale for this version.

## Init
Don't create, regenerate or overwrite `AGENTS.md`, and don't create `CLAUDE.md` or `GEMINI.md`.
Skills live in `.agents/skills/`. Claude Code is used and `.claude/skills/` is not existed. create the symlink from `.agents/skills/` to `.claude/skills/`. Do figure out how to create the symlink based on your OS (Linux/macOS or Windows).

## Commits
- Commit only when the user says to. Don't offer or ask to commit.
- Commit as the user's git identity only. No agent author or `Co-Authored-By` trailers.
- Run `git add -A`, show the status and message, and get the user's confirmation before committing.
