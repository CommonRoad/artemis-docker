"""Run saved submissions sequentially or in separate SUMO worker processes."""

from collections.abc import Iterable, Iterator
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from multiprocessing import get_context
from pathlib import Path
import time

from ..common.errors import failure_cause
from .artifacts import export_result
from .execution import run_submission


@dataclass
class BatchResult:
    scenario_name: str
    status: str
    valid: bool | None
    feasible: bool | None
    cost: float | None
    message: str
    elapsed_seconds: float | None
    output_directory: str | None
    previous_result_retained: bool


def _run_scenario(path: Path, submission: Path, directory: Path) -> BatchResult:
    started = time.monotonic()
    destination = directory / path.stem
    record = BatchResult(path.stem, "planner_error", None, None, None, "", 0.0, None, False)
    try:
        result = run_submission(path, submission)
        destination = directory / str(result.scenario.scenario_id)
        record.valid = result.valid
        record.feasible = result.validation.feasible
        record.cost = result.cost
        record.message = result.scoring_error or result.validation.message
        record.status = "invalid_solution"
        if result.valid:
            record.status = "scoring_error" if result.scoring_error else "passed"
    except Exception as error:
        error = failure_cause(error)
        code = getattr(error, "code", None)
        record.status = code if isinstance(code, str) else "planner_error"
        record.message = f"{type(error).__name__}: {error}"
    else:
        try:
            record.output_directory = str(export_result(result, directory))
        except Exception as error:
            record.status = "export_error"
            record.message = f"{type(error).__name__}: {error}"
    record.previous_result_retained = (
        record.output_directory is None and (destination / "metadata.json").is_file()
    )
    record.elapsed_seconds = round(time.monotonic() - started, 2)
    return record


def _worker_failure(path: Path, directory: Path, error: Exception) -> BatchResult:
    return BatchResult(
        path.stem, "worker_error", None, None, None,
        f"{type(error).__name__}: {error}", None, None,
        (directory / path.stem / "metadata.json").is_file(),
    )


def run_batch(
    paths: Iterable[str | Path],
    submission: str | Path,
    directory: str | Path,
    *,
    workers: int = 1,
) -> Iterator[BatchResult]:
    """Yield one record per task, exporting completed drives before releasing them.

    One worker runs in the calling process for debugging. Multiple workers use
    spawn so each process owns its SUMO instance and submission imports. Parallel
    records arrive in completion order. Use distinct scenario IDs within a batch.
    """
    if workers < 1:
        raise ValueError("workers must be at least 1")
    paths = [Path(path).resolve() for path in paths]
    submission = Path(submission).resolve()
    directory = Path(directory).resolve()
    if workers == 1:
        for path in paths:
            yield _run_scenario(path, submission, directory)
    else:
        with ProcessPoolExecutor(max_workers=workers, mp_context=get_context("spawn")) as pool:
            futures = {}
            for path in paths:
                try:
                    futures[pool.submit(_run_scenario, path, submission, directory)] = path
                except Exception as error:
                    # A failed pool may reject later tasks before they start.
                    yield _worker_failure(path, directory, error)
            for future in as_completed(futures):
                path = futures[future]
                try:
                    yield future.result()
                except Exception as error:
                    # A worker can fail outside _run_scenario (for example, if its
                    # process exits). Keep the remaining tasks in the report.
                    yield _worker_failure(path, directory, error)
