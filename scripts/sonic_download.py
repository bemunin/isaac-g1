"""Download NVIDIA's default SONIC weights (huggingface.co/nvidia/GEAR-SONIC) for oc.control.g1_wbc_sonic.

The files (~865 MB) go to sim/exts/oc.control.g1_wbc_sonic/data/sonic/, which git ignores. Files already
there with the expected size are skipped. Weights are under the NVIDIA Open Model License (LICENSE)."""

import argparse
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DESTINATION = ROOT / "sim" / "exts" / "oc.control.g1_wbc_sonic" / "data" / "sonic"
URL = "https://huggingface.co/nvidia/GEAR-SONIC/resolve/main/{}"
FILES = {  # name: size in bytes, from the Hugging Face tree API
    "LICENSE": 8886,
    "config.json": 2428,
    "observation_config.yaml": 2336,
    "model_encoder.onnx": 50100513,
    "model_decoder.onnx": 40900688,
    "planner_sonic.onnx": 773952989,
}


def download(name: str, size: int) -> None:
    path = DESTINATION / name
    if path.is_file() and path.stat().st_size == size:
        print(f"{name}: present")
        return
    partial = path.with_suffix(path.suffix + ".part")
    print(f"{name}: downloading {size / 1e6:.1f} MB", flush=True)
    with urllib.request.urlopen(URL.format(name)) as response, partial.open("wb") as out:
        while chunk := response.read(1 << 20):
            out.write(chunk)
    if partial.stat().st_size != size:
        raise RuntimeError(f"{name}: got {partial.stat().st_size} bytes, expected {size}")
    partial.replace(path)


def main() -> int:
    argparse.ArgumentParser(prog="pixi run sonic-download", description=__doc__).parse_args()
    DESTINATION.mkdir(parents=True, exist_ok=True)
    try:
        for name, size in FILES.items():
            download(name, size)
    except (OSError, RuntimeError) as exc:
        print(f"Unable to download SONIC: {exc}", file=sys.stderr)
        return 1
    print(f"SONIC weights in {DESTINATION}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
