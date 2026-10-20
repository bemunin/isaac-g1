---
name: oc-benchmark
description: Benchmark the warehouse Scenario across 6 scenarios (Newton/PhysX × headless, ROS headless, ROS windowed) and summarize the KPIs. Use when asked to benchmark, measure performance of, or compare physics/ROS/headless for the warehouse.
---

# oc-benchmark

Runs `pixi run warehouse --benchmark` (standalone workflow, `isaacsim.benchmark.services` KPIs) for each
scenario below, then tabulates the results. Run from the project root.

## Scenarios

| # | Scenario (folder suffix) | Flags |
|---|---|---|
| 1 | `baseline1_newton_headless` | `--physics newton --headless` |
| 2 | `newton_ros_headless` | `--physics newton --ros --headless` |
| 3 | `newton_ros` | `--physics newton --ros` |
| 4 | `baseline2_physx_headless` | `--physics physx --headless` |
| 5 | `physx_ros_headless` | `--physics physx --ros --headless` |
| 6 | `physx_ros` | `--physics physx --ros` |

One session writes everything into one folder (`warehouse` is the Scenario name; `research/` is gitignored):

```
research/benchmark_output/<YYYYMMDD>_<HHMMSS>_warehouse/
  settings.txt       # seconds, controller, dds, isaacsim
  summarize.md       # final report: settings, KPI table, deltas, Analysis
  <scenario>/        # one per scenario: kpis_warehouse_*.json, kpis_app_startup.json, <scenario>.log
```

## Steps

1. **Settings.** Defaults: 60 s measured (after a 60-frame warmup), the session's DDS
   (`pixi run dds` shows it). Use what the user gave; otherwise the defaults. `run.sh` also records the
   `isaacsim` version pinned in `pixi.toml` in `settings.txt`.
2. **Run** in the background, once (≈15–20 min; scenarios run sequentially because they share the GPU):
   ```sh
   .agents/skills/oc-benchmark/run.sh [SECONDS]
   ```
   It prints `session=<dir>` first. Scenarios 3 and 6 open an Isaac Sim window that closes itself.
   To use another DDS, run `pixi run dds <name>` first.
3. **Check.** Done when `run.sh` printed 6 scenario lines, each with `kpis=1`. `rc≠0` with `kpis=1` is a
   valid run (see shutdown crash below). For `kpis=0`, read the tail of `<session>/<scenario>/<scenario>.log`,
   fix, and rerun only that scenario into the same session:
   ```sh
   .agents/skills/oc-benchmark/run.sh SECONDS <session> <scenario> [...]
   ```
4. **Summarize:**
   ```sh
   python3 .agents/skills/oc-benchmark/summarize.py [SESSION_DIR]
   ```
   With no arg it picks the newest `*_warehouse` session. It writes `<session>/summarize.md` (settings,
   KPI table, deltas: Newton runs vs baseline1, PhysX runs vs baseline2, baseline2 vs baseline1) and keeps
   any existing `## Analysis` section.
5. **Analyse.** Replace the placeholder under `## Analysis` in `summarize.md` with bullets on: Newton vs
   PhysX, ROS cost, window cost, bottleneck (per "Reading the KPIs"), real-time factor and hitches, and
   anomalies (crashes, missing runs). When an earlier `research/benchmark_output/*_warehouse/summarize.md`
   exists, compare against the newest one (check its `isaacsim=` setting: numbers from another version
   are not like for like). Done when every one of those bullets is written. Report the
   analysis in chat with the `summarize.md` path.

## Reading the KPIs

- **The G1 stands during runs** (no loop since `oc.scenario.warehouse` 0.10.0), so results from earlier
  sessions aren't comparable.
- **Bottleneck:** compare `GPU frame ms` with `App frame ms`. GPU far below app frame ⇒ not GPU-bound.
  Then compare `CPU %` with `num_cpus × 100` (in the JSON): well below ⇒ a serial main-thread bottleneck
  (Python loop, SONIC ONNX inference, ROS graphs), not total CPU capacity.
- **`Physics_Step Frametime` is the interval between physics steps, not physics cost.**
  Mean × `Steps/update` equals `App frame ms` exactly, so it can't single out physics. Attribute
  main-thread time with a Tracy capture (`profile-isaac-sim` skill).
- **Shutdown crash:** `newton_ros` (windowed) can segfault (rc=139) on timeline stop in the Newton
  tensors + ROS 2 nodes plugins, *after* "Writing metrics to …" in the log — the KPIs are valid.
