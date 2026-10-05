"""Public API: open the DDS Benchmark's Scenario."""

from pathlib import Path

import isaacsim.core.experimental.utils.stage as stage_utils

__all__ = ["SCENARIO", "open_scenario"]

SCENARIO = Path(__file__).resolve().parents[3] / "usd" / "simple_g1_scenario.usda"


def open_scenario() -> None:
    """Open SCENARIO as the stage, replacing the current one. Its G1 is at /World/G1.

    Raises:
        RuntimeError: The Scenario could not be opened.
    """
    opened, _ = stage_utils.open_stage(str(SCENARIO))
    if not opened:
        raise RuntimeError(f"open_scenario: could not open {SCENARIO}")
