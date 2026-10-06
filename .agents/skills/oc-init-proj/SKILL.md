---
name: oc-init-proj
description: Initialize this repo for agents: install all Pixi envs and create the `.claude/skills` and `_isaacsim` symlinks; never create CLAUDE.md. Use when the user says `init` or `run init`, or when either symlink is missing.
---

# oc-init-proj

## Rules

- Do not create `CLAUDE.md` or `GEMINI.md`. `AGENTS.md` is the project's agent instructions.
- Create each symlink below only if it is missing.
- Link targets must be relative (to the link's own folder), so the links survive moving the checkout.
- Run every command from the project root.
- On Windows, symlinks need Developer Mode or an admin shell. In PowerShell, use `cmd /c mklink`
  as shown: `New-Item -ItemType SymbolicLink` in Windows PowerShell 5.1 stores an absolute target.
- Both links are gitignored (`.claude/`, `/_isaacsim`); don't commit them.
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

## 2. `_isaacsim` → Isaac Sim package

The target lives in the Pixi `sim` env (installed in step 1).
The site-packages path differs by OS.

Linux / macOS:

```sh
ln -s .pixi/envs/sim/lib/python3.12/site-packages/isaacsim _isaacsim
```

Windows (cmd):

```bat
mklink /D _isaacsim .pixi\envs\sim\Lib\site-packages\isaacsim
```

Windows (PowerShell):

```powershell
cmd /c mklink /D _isaacsim .pixi\envs\sim\Lib\site-packages\isaacsim
```

## Verify

- Linux / macOS: `ls -l .claude/skills _isaacsim` shows
  `.claude/skills -> ../.agents/skills` and
  `_isaacsim -> .pixi/envs/sim/lib/python3.12/site-packages/isaacsim`.
- Windows: `dir .claude` and `dir` show `<SYMLINKD>` entries with those relative targets.
- `.claude/skills/` lists the skill folders; `_isaacsim/` lists the package (`apps`, `exts`, `extscache`, ...).
