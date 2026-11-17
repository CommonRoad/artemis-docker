"""Load and package planner source files and required assets."""

from __future__ import annotations

import argparse
from collections.abc import Iterator
from contextlib import contextmanager
import importlib
import os
from pathlib import Path
import shutil
import sys
import tempfile
from typing import TYPE_CHECKING
import zipfile

if TYPE_CHECKING:
    from ..spec import PlannerSpec

DEFAULT_SUBMISSION = Path(__file__).resolve().parents[2] / "my_submission"

EXCLUDED_DIRECTORIES = {
    ".git",
    ".idea",
    ".vscode",
    ".ipynb_checkpoints",
    ".pytest_cache",
    ".mypy_cache",
    ".ruff_cache",
    "__pycache__",
    ".venv",
    "venv",
    "env",
    ".tox",
    ".nox",
    "build",
    "dist",
    "outputs",
    "solutions",
}
EXCLUDED_FILES = {".git", ".gitignore", ".gitattributes", ".gitmodules", ".DS_Store"}


def submission_files(directory: str | Path) -> list[Path]:
    """Select source and assets, excluding metadata, caches and generated output."""
    directory = Path(directory)
    if not (directory / "student_planner.py").is_file():
        raise ValueError(f"Missing student_planner.py in {directory}")
    selected = []
    for root, directories, filenames in os.walk(directory):
        directories[:] = [
            name
            for name in directories
            if name not in EXCLUDED_DIRECTORIES
            and not name.endswith((".egg-info", ".dist-info"))
        ]
        for name in directories + filenames:
            path = Path(root) / name
            if path.is_symlink():
                raise ValueError(
                    f"Symlinks are not supported; copy the file or directory into the bundle: {path}"
                )
            if (
                path.is_file()
                and name not in EXCLUDED_FILES
                and not name.endswith((".pyc", ".pyo", ".swp", ".swo", "~"))
            ):
                selected.append(path)
    return sorted(selected)


@contextmanager
def load_submission(directory: str | Path) -> Iterator[PlannerSpec]:
    """Stage source and assets and select Python imports for the duration of one run."""
    directory = Path(directory)
    files = submission_files(directory)
    with tempfile.TemporaryDirectory(prefix="commonroad-submission-") as temporary:
        for source in files:
            target = Path(temporary) / source.relative_to(directory)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
        staged = Path(temporary)
        roots = {
            path.relative_to(directory).parts[0].removesuffix(".py")
            for path in files
            if path.suffix == ".py"
        }
        # A notebook can evaluate several bundles with the same module names,
        # including a complete SMP tree. Restore its existing imports afterward.
        previous_path = sys.path[:]
        previous_modules = {
            name: module for name, module in sys.modules.items() if name.split(".")[0] in roots
        }
        for name in previous_modules:
            del sys.modules[name]
        sys.path.insert(0, str(staged))
        importlib.invalidate_caches()
        try:
            module = importlib.import_module("student_planner")
            yield module.create_planner()
        finally:
            sys.path[:] = previous_path
            for name in list(sys.modules):
                if name.split(".")[0] in roots:
                    del sys.modules[name]
            sys.modules.update(previous_modules)
            importlib.invalidate_caches()


def package_submission(directory: str | Path, destination: str | Path) -> Path:
    """Package the same source and assets used by the loader, preserving relative paths."""
    directory = Path(directory).resolve()
    destination = Path(destination).resolve()
    if destination.is_relative_to(directory):
        raise ValueError("Write the submission ZIP outside the submission directory")
    files = submission_files(directory)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(destination, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            info = zipfile.ZipInfo(
                path.relative_to(directory).as_posix(), date_time=(2026, 1, 1, 0, 0, 0)
            )
            info.compress_type = zipfile.ZIP_DEFLATED
            with path.open("rb") as source, archive.open(info, "w", force_zip64=True) as target:
                shutil.copyfileobj(source, target)
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    parser.add_argument("destination")
    args = parser.parse_args()
    print(package_submission(args.directory, args.destination))
