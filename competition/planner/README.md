# Planner package

Use the [main README](../README.md) for setup, planner selection and submission.
This page maps the implementation and Python entry points.

```text
planner/
├── spec.py                # PlannerSpec, smp_spec(), reactive_spec()
├── __main__.py            # python -m planner
├── algorithms/            # Adapters for the supplied planners
│   ├── smp.py             # Repeated search planning
│   ├── smp_automaton.py   # Exact-state and finite-horizon primitive access
│   ├── smp_connector.py   # Initial-speed transitions to the primitive grid
│   └── reactive.py        # Repeated reactive planning
├── common/                # Shared planning and scenario utilities
│   ├── prediction.py      # Forecast observed traffic
│   ├── trajectory.py      # State conversion and executable-prefix checks
│   ├── validation.py      # Summarize upstream full-solution validation
│   ├── scenarios.py       # Public assets, primitive paths, XML discovery
│   ├── visualization.py   # Animation and traffic comparisons
│   ├── errors.py          # Reported planner failures
│   ├── scoring.py         # CommonRoad cost-evaluator compatibility
│   └── vendor/            # Pinned route matcher and license
├── runtime/               # Run and assess a submitted planner
│   ├── submission.py      # Load and package source files and planner assets
│   ├── execution.py       # Execute, validate, and score drives
│   ├── artifacts.py       # Save the ego/traffic pair and execution metadata
│   ├── batch.py           # Sequential and process-based batches with per-task exports
│   └── evaluation.py      # Result summaries and per-scenario feedback
├── competition_data.py    # Download of the course data kept out of Git
├── fetch_data.py          # python -m planner.fetch_data
└── assets/                # Scenario XMLs (fetched) and motion primitives
```

For a participant, the main entry points are:

| Task | Entry point |
| --- | --- |
| Implement or select a planner | [my_submission/student_planner.py](../my_submission/student_planner.py) |
| Configure its vehicle and execution | [spec.py](spec.py) |
| Package a submission | [runtime/submission.py](runtime/submission.py) |
| Evaluate a submission from Python | [runtime/execution.py](runtime/execution.py): `run_submission()` |
| Experiment with an in-memory planner | [runtime/execution.py](runtime/execution.py): `run_spec()` |
| Export a completed drive and its traffic | [runtime/artifacts.py](runtime/artifacts.py): `export_result()` |
| Inspect success, cost, or failure | [runtime/evaluation.py](runtime/evaluation.py): `describe_result()` or `evaluate_scenario()` |
| Validate a loaded ego/traffic pair | [common/validation.py](common/validation.py): `validate_solution()` |
| Select public scenarios | [common/scenarios.py](common/scenarios.py): `public_scenarios()` |
| Run and export a batch | [runtime/batch.py](runtime/batch.py): `run_batch()`; see [tutorial 05](../tutorials/05_batch_processing.ipynb) |

`run_submission()` loads saved source with the fixed competition settings;
`run_spec()` runs an in-memory planner with its own settings. Both use `simulation/`
to drive SUMO and validate and score the executed trajectory. `export_result()`
saves that completed run without rerunning the simulation.

`result.validation` contains the named `valid`, `feasible` and `message` summary;
`result.valid` and `result.cost` provide the overall outcome. `feasible=None` means
the upstream checker stopped without returning feasibility results. Tutorial 03
uses the same summary for saved XML pairs and explains the upstream return contract.
If scoring fails, the completed drive remains available with `result.cost=None`
and the reason in `result.scoring_error`.

For implementation details, see:

- [Search adapter](../tutorials/00_interactive_search_quickstart.ipynb#Inspect-the-drive)
- [Example planners and reactive adapter](../tutorials/02_example_planners.ipynb)
- [Validation and scoring](../tutorials/03_solution_and_cost_validation.ipynb)
- [Custom planner contract](../tutorials/04_reuse_and_evaluate_planners.ipynb#Use-a-custom-planner)

`simulation/planner_adapter.py` supplies observations and checks planner output.
`common/trajectory.py` supports the supplied adapters' state conversion and
executable-prefix checks. Prefix and final dynamics checks use the upstream CommonRoad
feasibility checker directly. The supplied scenario XMLs already use local map coordinates.
Search uses the bundled SMP package; reactive planning
uses `commonroad_rp/`.
