from pathlib import Path

__all__ = ["G1_USD", "add"]

# G1 asset; its root prim is /G1 (defaultPrim).
G1_USD = Path(__file__).resolve().parents[3] / "usd" / "g1.usda"


def add(a: float, b: float) -> float:
    """Example public function, importable as `oc.robot.g1.add`."""
    return a + b
