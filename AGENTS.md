# Project instructions


## Skills to Follow
Before any coding work, read and follow these skills:

- `.agents/skills/karpathy-guidelines/SKILL.md`
- `.agents/skills/isaac-sim-sensor/SKILL.md`
- `.agents/skills/oc-g1-ros/SKILL.md`

Claude Code: read them from `.claude/skills/` instead. If that symlink is
missing, create it first: `mkdir -p .claude && ln -s ../.agents/skills .claude/skills`

## Commits
- Commit only when the user says to. Don't offer or ask to commit.
- Commit as the user's git identity only. No agent author or `Co-Authored-By` trailers.
- Run `git add -A`, show the status and message, and get the user's confirmation before committing.
