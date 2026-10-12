"""Write <session>/summarize.md (settings, KPI table, deltas vs baselines) for a warehouse benchmark session, and
print it. An existing `## Analysis` section in summarize.md is kept.

Usage: summarize.py [SESSION_DIR]   (default: the newest research/benchmark_output/*_warehouse folder)
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
OUTPUT = ROOT / "research" / "benchmark_output"
ANALYSIS = "## Analysis"
SCENARIOS = [
    "baseline1_newton_headless",
    "newton_ros_headless",
    "newton_ros",
    "baseline2_physx_headless",
    "physx_ros_headless",
    "physx_ros",
]
COLUMNS = [
    ("FPS", "Mean FPS"),
    ("App frame ms", "Mean App_Update Frametime"),
    ("Phys step ms", "Mean Physics_Step Frametime"),
    ("Phys P99 ms", "P99 Physics_Step Frametime"),
    ("RTF", "Real Time Factor"),
    ("RTF stdev", "Stdev Windowed RTF"),
    ("GPU frame ms", "Mean GPU Frametime"),
    ("CPU %", "Mean CPU Usage"),
    ("RSS GB", "System Memory RSS"),
    ("VRAM GB", "GPU Memory Dedicated"),
]


def load(session: Path) -> dict[str, dict]:
    results = {}
    for scenario in SCENARIOS:
        kpis = sorted((session / scenario).glob("kpis_warehouse_*.json"))
        if kpis:
            results[scenario] = json.loads(kpis[0].read_text())
    return results


def pct(value: float, base: float) -> str:
    return f"{(value - base) / base * 100:+.1f}%"


def report(session: Path, results: dict[str, dict]) -> list[str]:
    first = next(iter(results.values()))
    bench = first["benchmark"]
    settings = session / "settings.txt"
    lines = [
        f"# Warehouse benchmark — {session.name}",
        "",
        f"- GPU: {bench.get('gpu_device_name', '-')}, CPUs: {bench.get('num_cpus', '-')}",
        f"- Isaac Sim: {first.get('App Info', ['-'])[0]}, Kit {first.get('Kit', '-')}",
        f"- Controller: {bench.get('controller', '-')}, measured: {bench.get('duration_s', '-')} s",
    ]
    if settings.is_file():
        lines.append("- Settings: " + ", ".join(settings.read_text().split()))
    missing = [s for s in SCENARIOS if s not in results]
    if missing:
        lines.append("- **Missing KPI file:** " + ", ".join(missing))

    lines += ["", "## KPIs", ""]
    lines.append("| Scenario | " + " | ".join(label for label, _ in COLUMNS) + " | Steps/update |")
    lines.append("|---" * (len(COLUMNS) + 2) + "|")
    for scenario, data in results.items():
        b = data["benchmark"]
        steps = b["Physics Step Count"] / b["Num App Updates"]
        lines.append(f"| {scenario} | " + " | ".join(str(b.get(k, "-")) for _, k in COLUMNS) + f" | {steps:.2f} |")

    lines += ["", "## Deltas vs baseline", "", "| Scenario | Baseline | FPS | RTF | GPU frame ms | CPU % |", "|---|---|---|---|---|---|"]
    for scenario in SCENARIOS:
        if scenario not in results or scenario.startswith("baseline1"):
            continue
        # Each engine's runs compare with its own headless baseline; baseline2 (PhysX) compares with Newton's.
        if "newton" in scenario or scenario.startswith("baseline2"):
            base = "baseline1_newton_headless"
        else:
            base = "baseline2_physx_headless"
        if base not in results:
            continue
        b, ref = results[scenario]["benchmark"], results[base]["benchmark"]
        cells = [pct(b[k], ref[k]) for k in ("Mean FPS", "Real Time Factor", "Mean GPU Frametime", "Mean CPU Usage")]
        lines.append(f"| {scenario} | {base} | " + " | ".join(cells) + " |")
    return lines


def main() -> int:
    if len(sys.argv) > 1:
        session = Path(sys.argv[1]).resolve()
    else:
        sessions = sorted(OUTPUT.glob("*_warehouse"))
        if not sessions:
            print(f"No *_warehouse session in {OUTPUT}", file=sys.stderr)
            return 1
        session = sessions[-1]

    results = load(session)
    if not results:
        print(f"No KPI files in {session}", file=sys.stderr)
        return 1

    out = session / "summarize.md"
    analysis = f"{ANALYSIS}\n\n_To be written._\n"
    if out.is_file() and ANALYSIS in (old := out.read_text()):
        analysis = old[old.index(ANALYSIS) :]
    text = "\n".join(report(session, results)) + "\n\n" + analysis
    out.write_text(text)
    print(text)
    print(f"Wrote {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
