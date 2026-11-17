"""Export a completed drive and the traffic it encountered, without resimulation."""

from __future__ import annotations

from dataclasses import asdict
import hashlib
from importlib.metadata import distributions
from importlib.util import find_spec
import json
from pathlib import Path
import platform
import tempfile
from typing import TYPE_CHECKING
from uuid import uuid4

from commonroad.common.file_writer import CommonRoadFileWriter, OverwriteExistingFile
from commonroad.common.solution import CommonRoadSolutionWriter

from .submission import submission_files

if TYPE_CHECKING:
    from ..spec import PlannerSpec
    from .execution import RunResult


def _source_hash(directory: Path, files: list[Path]) -> str:
    digest = hashlib.sha256()
    for path in files:
        digest.update(path.relative_to(directory).as_posix().encode())
        digest.update(b"\0")
        with path.open("rb") as source:
            while chunk := source.read(1024 * 1024):
                digest.update(chunk)
        digest.update(b"\0")
    return digest.hexdigest()


def submission_metadata(directory: str | Path) -> dict[str, str]:
    """Identify the source and assets selected by the submission loader."""
    directory = Path(directory)
    return {
        "kind": "submission",
        "sha256": _source_hash(directory, submission_files(directory)),
    }


def run_metadata(scenario_path: str | Path, spec: PlannerSpec) -> dict[str, object]:
    """Capture settings and source identities while the selected imports are active."""
    framework = {}
    for name in ("planner", "simulation", "commonroad_rp", "SMP"):
        directory = Path(next(iter(find_spec(name).submodule_search_locations)))
        framework[name] = _source_hash(directory, sorted(directory.rglob("*.py")))
    planner_type = type(spec.planner)
    return {
        "run_id": str(uuid4()),
        "input_scenario_sha256": hashlib.sha256(Path(scenario_path).read_bytes()).hexdigest(),
        "planner": f"{planner_type.__module__}.{planner_type.__qualname__}",
        "source": {"kind": "in_memory"},
        "settings": asdict(spec.settings),
        "environment": {
            "python": platform.python_version(),
            "platform": platform.platform(),
            "packages": {package.metadata["Name"]: package.version for package in distributions()},
            "framework_sha256": framework,
        },
    }


def export_result(result: RunResult, directory: str | Path) -> Path:
    """Write one benchmark folder; exporting it again replaces its three files.

    The Scenario contains simulated traffic and the original planning problems.
    Ego trajectories remain in the Solution, avoiding a duplicate ego obstacle.
    Metadata records the run settings, source identity and validation result.
    All files are written before replacing an existing pair. Metadata is published
    last, so an interrupted replacement cannot retain the old pair's metadata.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    destination = directory / str(result.scenario.scenario_id)
    with tempfile.TemporaryDirectory(prefix=".commonroad-export-", dir=directory) as temporary:
        staged = Path(temporary)
        scenario_path = staged / "scenario.xml"
        solution_path = staged / "solution.xml"
        metadata_path = staged / "metadata.json"
        CommonRoadFileWriter(
            result.scenario,
            result.planning_problems,
            author=result.scenario.author,
            affiliation=result.scenario.affiliation,
            source="Interactive CommonRoad run; executed traffic",
            tags=result.scenario.tags,
            decimal_precision=12,
        ).write_to_file(str(scenario_path), OverwriteExistingFile.ALWAYS, check_validity=True)
        CommonRoadSolutionWriter(result.solution).write_to_file(
            output_path=str(staged), filename=solution_path.name, overwrite=True
        )
        metadata = {
            **result.metadata,
            "format_version": 2,
            "scenario_id": str(result.scenario.scenario_id),
            "planning_problem_ids": result.solution.planning_problem_ids,
            "dt": result.scenario.dt,
            "validation": asdict(result.validation),
            "cost": result.cost,
            "scoring_error": result.scoring_error,
            "cost_function": result.solution.planning_problem_solutions[0].cost_function.name,
            "files": {
                path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                for path in (scenario_path, solution_path)
            },
        }
        metadata_path.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n")
        destination.mkdir(parents=True, exist_ok=True)
        (destination / "metadata.json").unlink(missing_ok=True)
        for path in (scenario_path, solution_path, metadata_path):
            path.replace(destination / path.name)
    return destination
