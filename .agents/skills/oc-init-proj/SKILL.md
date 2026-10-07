---
name: oc-init-proj
description: Initialize this repo for agents: install all Pixi envs and create the `.claude/skills`, `_isaacsim` and Isaac Sim skill symlinks; never create CLAUDE.md. Use when the user says `init` or `run init`, or when any of these symlinks is missing or broken (e.g. a fresh clone).
---

# oc-init-proj

## Rules

- Do not create `CLAUDE.md` or `GEMINI.md`. `AGENTS.md` is the project's agent instructions.
- Create each symlink below only if it is missing.
- Link targets must be relative (to the link's own folder), so the links survive moving the checkout.
- Run every command from the project root.
- On Windows, symlinks need Developer Mode or an admin shell. In PowerShell, use `cmd /c mklink`
  as shown: `New-Item -ItemType SymbolicLink` in Windows PowerShell 5.1 stores an absolute target.
- All links are gitignored (`.claude/`, `/_isaacsim`, the Isaac Sim skill links in `.agents/skills/`);
  don't commit them. Their targets differ by OS, so a committed link breaks on other OSes.
- The project's `pixi.toml` lists `linux-64` and `win-64` platforms; `pixi install --all` installs
  the envs for the current OS.
- think before create .claude or .gemini: only do if you actually use Claude Code or Gemini Code, respectively.

## 1. Install all Pixi envs

Run this first, so every env (including `sim`, which `_isaacsim` points into) is installed.

Linux / macOS:

```sh
pixi install --all
```

Windows (cmd):

```bat
pixi install --all
```

Windows (PowerShell):

```powershell
pixi install --all
```

## 2. `.claude/skills` → `.agents/skills`

Only if you are Claude Code. Link `.claude/skills` to `.agents/skills`, creating `.claude/` first if needed.
Work out the symlink command for the current OS yourself, following the Rules above
(relative target from `.claude/`, directory link, only if missing).

## 3. `_isaacsim` → Isaac Sim package

Link `_isaacsim` at the project root to the `isaacsim` package inside the Pixi `sim` env's
site-packages (installed in step 1). The site-packages path differs by OS; find it in the installed env.
Work out the symlink command for the current OS yourself, following the Rules above
(relative target from the project root, directory link, only if missing).

## 4. Isaac Sim skills → `.agents/skills/<name>`

Link each skill below from the `isaacsim` package (installed in step 1) into `.agents/skills/<name>`:

- isaac-camera
- isaac-sim-orchestrator
- isaac-sim-ros-workspaces
- isaac-sim-ros2-bridge
- isaac-sim-sensor
- physics-simulation
- profile-isaac-sim

The source is `isaacsim/skills/<name>` inside the `sim` env's site-packages; the same folder `_isaacsim` points to (step 3).
Work out the symlink command for the current OS yourself, following the Rules above
(relative target from `.agents/skills/`, directory link, only if missing).

## Verify

- Linux / macOS: `ls -l .claude/skills _isaacsim` shows
  `.claude/skills -> ../.agents/skills` and
  `_isaacsim -> .pixi/envs/sim/lib/python3.12/site-packages/isaacsim`.
- Windows: `dir .claude` and `dir` show `<SYMLINKD>` entries with those relative targets.
- `.claude/skills/` lists the skill folders; `_isaacsim/` lists the package (`apps`, `exts`, `extscache`, ...).
- Each Isaac Sim skill resolves: `.agents/skills/isaac-sim-sensor/SKILL.md` (and the other six) opens.
