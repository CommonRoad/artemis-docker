"""Check shared and installed dependency versions with Python 3.11."""

import argparse
import importlib.metadata
from pathlib import Path
import tomllib
import sys

CORE = (
    "commonroad-io",
    "commonroad-drivability-checker",
    "commonroad-route-planner",
    "commonroad-clcs",
    "commonroad-vehicle-models",
    "numpy",
    "scipy",
    "matplotlib",
    "shapely",
    "triangle",
    "networkx",
    "imageio",
    "omegaconf",
)


def pins(root: Path) -> dict[str, str]:
    project = tomllib.loads((root / "pyproject.toml").read_text())["project"]
    exact = dict(item.split("==", 1) for item in project["dependencies"] if "==" in item)
    missing = set(CORE) - exact.keys()
    if missing:
        raise SystemExit(f"Missing shared version pins: {sorted(missing)}")
    return {name: exact[name] for name in CORE}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--peer", type=Path, help="Path to the other repository")
    parser.add_argument(
        "--installed", action="store_true", help="Also check installed distributions"
    )
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    project = tomllib.loads((root / "pyproject.toml").read_text())["project"]
    expected = pins(root)
    if args.peer:
        other = pins(args.peer)
        differences = {
            name: (version, other[name])
            for name, version in expected.items()
            if version != other[name]
        }
        if differences:
            raise SystemExit(f"Shared dependency mismatch: {differences}")
        own_lock = tomllib.loads((root / "poetry.lock").read_text())
        peer_lock = tomllib.loads((args.peer / "poetry.lock").read_text())
        own_versions = {p["name"]: p["version"] for p in own_lock["package"]}
        peer_versions = {p["name"]: p["version"] for p in peer_lock["package"]}
        conflicts = {
            name: (own_versions[name], peer_versions[name])
            for name in own_versions.keys() & peer_versions.keys()
            if own_versions[name] != peer_versions[name]
        }
        if conflicts:
            raise SystemExit(f"Overlapping locked dependencies conflict: {conflicts}")
    if args.installed:
        if sys.version_info[:2] != (3, 11):
            raise SystemExit("The supported runtime is Python 3.11")
        expected = dict(item.split("==", 1) for item in project["dependencies"] if "==" in item)
        actual = {name: importlib.metadata.version(name) for name in expected}
        differences = {
            name: (version, actual[name])
            for name, version in expected.items()
            if version != actual[name]
        }
        if differences:
            raise SystemExit(f"Installed dependency mismatch: {differences}")
    print("Dependency versions match.")


if __name__ == "__main__":
    main()
