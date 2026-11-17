"""Select bundled scenarios and primitives, or discover standalone CommonRoad XMLs."""

from collections.abc import Iterator
from importlib.resources import files
from pathlib import Path
import xml.etree.ElementTree as ET

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario, ScenarioID

ASSETS = Path(str(files("planner").joinpath("assets")))
BENCHMARK_SIMULATION_STEPS = 200
BENCHMARK_RANDOM_SEED = 1234


def public_scenarios() -> list[Path]:
    """Return all 100 public competition benchmarks in filename order."""
    return sorted((ASSETS / "scenarios" / "public").glob("*.xml"))


def primitive_path(scenario_id: ScenarioID | str, speed_spacing: int | None = None) -> Path:
    if speed_spacing is None:
        speed_spacing = 4 if str(scenario_id) == "RUS_Bicycle-11_1_I-1" else 2
    if str(speed_spacing) not in ("2", "4"):
        raise ValueError("The SMP starter provides speed spacings of 2 or 4 m/s")
    filename = (
        f"V_0.0_20.0_Vstep_{speed_spacing}.0_"
        "SA_-1.066_1.066_SAstep_0.18_T_0.5_Model_BMW_320i.xml"
    )
    return ASSETS / "primitives" / filename


def is_scenario_file(path: str | Path) -> bool:
    path = Path(path)
    if not path.is_file() or path.suffix != ".xml":
        return False
    with path.open("rb") as source:
        _, root = next(ET.iterparse(source, events=("start",)))
        return root.tag == "commonRoad"


def scenario_candidates(path: str | Path) -> Iterator[Path]:
    """Discover XML paths before parsing, so batch callers can report per-file errors."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(path)
    candidates = [path] if path.is_file() else sorted(path.rglob("*.xml"))
    return iter(candidates)


def scenario_files(path: str | Path) -> Iterator[Path]:
    """Return CommonRoad scenario XMLs, excluding solution and other XML files."""
    return (candidate for candidate in scenario_candidates(path) if is_scenario_file(candidate))


def all_scenario_iterator(path: str | Path) -> Iterator[tuple[Scenario, PlanningProblemSet]]:
    return (CommonRoadFileReader(str(p)).open() for p in scenario_files(path))


def is_interactive(scenario: Scenario) -> bool:
    return scenario.scenario_id.obstacle_behavior == "I"


def scenario_iterator_non_interactive(
    path: str | Path,
) -> Iterator[tuple[Scenario, PlanningProblemSet]]:
    return ((s, p) for s, p in all_scenario_iterator(path) if not is_interactive(s))


def scenario_iterator_interactive(path: str | Path) -> Iterator[str]:
    """Yield paths to interactive CommonRoad scenarios."""
    for filename in scenario_files(path):
        scenario, _ = CommonRoadFileReader(str(filename)).open()
        if is_interactive(scenario):
            yield str(filename)


def scenario_iterator(
    path: str | Path,
) -> tuple[
    Iterator[tuple[Scenario, PlanningProblemSet]], Iterator[tuple[Scenario, PlanningProblemSet]]
]:
    return scenario_iterator_non_interactive(path), (
        CommonRoadFileReader(p).open() for p in scenario_iterator_interactive(path)
    )
