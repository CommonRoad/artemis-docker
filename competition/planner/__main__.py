"""Evaluate a submission on interactive benchmarks using the competition settings."""

import argparse
from pathlib import Path

from commonroad.common.file_reader import CommonRoadFileReader

from .common.errors import failure_cause
from .common.scenarios import (
    is_interactive,
    is_scenario_file,
    public_scenarios,
    scenario_candidates,
)
from .runtime.artifacts import export_result
from .runtime.execution import run_submission
from .runtime.submission import DEFAULT_SUBMISSION


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--scenarios", help="Scenario XML file or directory")
    selection.add_argument("--public", action="store_true", help="Run all 100 public benchmarks")
    parser.add_argument(
        "--submission",
        type=Path,
        default=DEFAULT_SUBMISSION,
        help="Submission directory (default: the supplied my_submission/)",
    )
    parser.add_argument(
        "--solutions",
        default="solutions",
        help="Result directory (default: solutions/) for scenario, solution and metadata",
    )
    args = parser.parse_args()
    if args.public:
        paths = public_scenarios()
    else:
        paths = scenario_candidates(args.scenarios)
    Path(args.solutions).mkdir(parents=True, exist_ok=True)
    count, failed = 0, False
    for path in paths:
        destination = None
        try:
            if not is_scenario_file(path):
                continue
            scenario, _ = CommonRoadFileReader(str(path)).open()
            if not is_interactive(scenario):
                continue
            count += 1
            destination = Path(args.solutions) / str(scenario.scenario_id)
            print(f"Processing {scenario.scenario_id}", flush=True)
            result = run_submission(path, args.submission)
            export_result(result, args.solutions)
            print(
                f"  valid={result.valid}, feasible={result.validation.feasible}, cost={result.cost}, "
                f"seconds={result.elapsed_seconds:.2f}"
            )
            if not result.valid:
                print(f"  {result.validation.message}", flush=True)
            if result.scoring_error:
                print(f"  scoring_error: {result.scoring_error}", flush=True)
            failed |= not result.valid or result.scoring_error is not None
        except Exception as error:
            error = failure_cause(error)
            failed = True
            print(f"  {path.name}: {getattr(error, 'code', 'planner_error')}: {error}", flush=True)
            if destination is not None and (destination / "metadata.json").is_file():
                print(f"  Previous result retained at {destination}", flush=True)
    if not count and not failed:
        parser.error(f"No interactive scenarios found at {args.scenarios}")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
