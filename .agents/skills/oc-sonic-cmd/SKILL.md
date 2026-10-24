---
name: oc-sonic-cmd
description: TODO
---

# oc-sonic-cmd

## SONIC modes

SONIC's kinematic planner has 27 modes, ids 0–26. The weights from `scripts/sonic_download.py` are NVIDIA's V2
planner, and every id produces valid planner motion. `oc.control.g1_sonic` exposes only 0–6, 8 and 14 today.

- **Moving** modes are driven by velocity and facing.
- **Static** modes hold in place and replan only when mode, facing or height changes.
- **Height** is the target pelvis height [m]; "–" means the mode ignores it.
- **Speed** "clip" means the planner uses the motion clip's own speed (NVIDIA sends `target_vel = -1`); a positive
  command is still tracked.

| id | name | kind | speed (m/s) | height (m) | exposed |
|---|---|---|---|---|---|
| 0 | idle | static | – | – | yes |
| 1 | slow walk | moving | 0.2–0.8 | – | yes |
| 2 | walk | moving | 0.8–1.5 | – | yes |
| 3 | run | moving | 1.5–3.0 | – | yes |
| 4 | squat | static | – | 0.4–0.8 | yes |
| 5 | kneel, two legs | static | – | 0.2–0.4 | yes |
| 6 | kneel, one leg | static | – | 0.2–0.4 | yes |
| 7 | lying face down | static | – | required | no |
| 8 | hand crawl | moving | 0.4–1.0 | 0.2–0.8 | yes |
| 9 | boxing, idle stance | static | – | – | no |
| 10 | boxing, walk | moving | 0.7–1.5 | – | no |
| 11 | boxing, left jab | moving | 0.7–1.5 | – | no |
| 12 | boxing, right jab | moving | 0.7–1.5 | – | no |
| 13 | boxing, random punches | moving | 0.7–1.5 | – | no |
| 14 | elbow crawl (overheats on the real robot) | moving | 0.7–1.0 | 0.2–0.8 | yes |
| 15 | boxing, left hook | moving | 0.7–1.5 | – | no |
| 16 | boxing, right hook | moving | 0.7–1.5 | – | no |
| 17 | happy walk (`FORWARD_JUMP` in NVIDIA's enum; it walks, it does not jump) | moving | clip | – | no |
| 18 | stealth walk | moving | clip | – | no |
| 19 | injured walk | moving | clip | – | no |
| 20 | careful walk (`LEDGE_WALKING`) | moving | clip | – | no |
| 21 | object carrying | moving | clip | – | no |
| 22 | crouch walk (`STEALTH_WALK_2`) | moving | clip | – | no |
| 23 | happy dance walk (forward only) | moving | clip | – | no |
| 24 | zombie walk | moving | clip | – | no |
| 25 | point walk (`GUN_WALK`) | moving | clip | – | no |
| 26 | scared walk | moving | clip | – | no |

Ids 27 and above are not modes. The planner accepts them silently and NVIDIA's C++ replaces them with 0, so validate
ids to 0–26.

Unverified: whether the SONIC tracking policy follows modes 7 and 9–26 in Isaac Sim (planner output being sane does
not prove it), and whether the boxing and styled walks can side-step or walk backwards.

Sources: NVIDIA GR00T-WholeBodyControl `gear_sonic_deploy/.../include/localmotion_kplanner.hpp:78-115` (enum, static
modes), `docs/source/references/planner_onnx.md:105-150` (V2 names, heights), `input_interface/keyboard_handler.hpp:260-294`
(speed ranges).
