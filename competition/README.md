# CommonRoad Competition — Stage 1: Urban Challenge

This repository provides the environment and starter planners for **Stage 1: Urban
Challenge** of the CommonRoad competition.

Build a planner for interactive urban traffic simulated by SUMO using the supplied
search or reactive planner, or your own implementation. Submit the executed ego trajectory, simulated
traffic from the same run, and the exact source and configuration that produced them.

## Course data

The 100 public scenarios (`planner/assets/scenarios/public/`), the bundled SMP planner
wheel (`vendor/commonroad_search-2025.1-py3-none-any.whl`) and the framework's tests
(`tests/`) never change, so they are not part of this repository. Fetch them once
(about 7 MB) from the repository root, **before** `poetry install`, because Poetry
installs SMP from that wheel:

```sh
python -m planner.fetch_data
```

This needs only Python 3 and the standard library, so it works before anything is
installed. The download is checked against a fixed SHA-256 before anything is
unpacked. Afterwards everything works offline. If scenarios are missing later, the
first `import planner` (the CLI, the tutorials, the batch processing) fetches them
again by itself.

| Command | Effect |
| --- | --- |
| `python -m planner.fetch_data` | Fetch the public scenarios and the SMP wheel if they are missing |
| `python -m planner.fetch_data --tests` | Also fetch the tests into `tests/`, needed only for `pytest` and `make test` |
| `python -m planner.fetch_data --force` | Download and unpack again, for example after deleting or editing a scenario |
| `python -m planner.fetch_data --archive FILE` | Unpack an archive downloaded in a browser, without network access |

If the automatic download does not work (for example behind a restrictive network),
the error message prints a download link for the archive; download it in a browser and
pass it with `--archive`.

Git ignores the fetched files, so they never end up in a commit. Do not change them:
the competition evaluates the original public scenarios.

## Install

Use **Python 3.11 on Linux x86_64**; on Windows/macOS, use the
[Linux amd64 Docker environment](#docker-and-vs-code). Install
[Poetry 2.2.1 separately](https://python-poetry.org/docs/#installation), for example
with `pipx install poetry==2.2.1`.

```sh
sudo apt-get install build-essential ffmpeg libgl1 libgomp1 libxerces-c3.2 libfox-1.6-0
conda create -n commonroad-competition python=3.11 pip=24.0
conda activate commonroad-competition
python -m planner.fetch_data
poetry install --with dev,test
export SUMO_HOME="$CONDA_PREFIX/lib/python3.11/site-packages/sumo"
export PATH="$SUMO_HOME/bin:$PATH"
jupyter lab
```

Poetry installs the dependencies and the bundled SMP planner (the wheel fetched in
[Course data](#course-data)) into the Conda environment.

In a new terminal, activate Conda and repeat the two exports. For VS Code, launch
`code .` from that terminal and select the `commonroad-competition` environment.

After repository updates, rerun `poetry install --with dev,test` or rebuild Docker.

<details>
<summary>Refreshing the bundled SMP planner</summary>

If an update replaces the SMP wheel without changing its version, reinstall it:

```sh
python -m pip install --no-deps --force-reinstall vendor/commonroad_search-2025.1-py3-none-any.whl
```

</details>

The [Dockerfile](Dockerfile) provides the reference environment, with dependencies
matching `poetry.lock`. It uses the course data already fetched into your checkout and
downloads it during the build otherwise.

## Tutorials

Start Jupyter from the repository root with the competition Python 3.11 environment.
Each notebook runs independently with **Restart Kernel and Run All**. Start with
**00**, try **02**, then use **04** to run and submit your own code. Use **05** to
run your saved planner across all public tasks.

| Notebook | What it covers |
| --- | --- |
| [00: Search quickstart](tutorials/00_interactive_search_quickstart.ipynb) | Run SMP A*, inspect predictions and animate a drive |
| [01: Benchmark I/O](tutorials/01_read_and_write_benchmarks.ipynb) | Read a planning problem and write a separate XML copy |
| [02: Example planners](tutorials/02_example_planners.ipynb) | Explore prediction, curvilinear coordinates, routing and dummy/reactive planning on one benchmark |
| [03: Validation and SM1](tutorials/03_solution_and_cost_validation.ipynb) | Reload, validate and score a saved ego/traffic pair |
| [04: Your submission](tutorials/04_reuse_and_evaluate_planners.ipynb) | Reuse search code, implement a custom planner, inspect failures and export/package results |
| [05: Batch processing](tutorials/05_batch_processing.ipynb) | Attempt all 100 scenarios sequentially or in parallel, export drives and save JSON/CSV reports |

## Code layout

```text
my_submission/
├── student_planner.py  # Select search, reactive or your own planner
└── custom_planner.py   # Implement your custom planner here
planner/
├── spec.py            # PlannerSpec and SMP/reactive factories
├── __main__.py        # python -m planner
├── algorithms/        # Adapters, automaton and speed connectors
├── common/            # Prediction, validation, scenarios and visualization
├── runtime/           # Source loading, execution, evaluation and export
├── competition_data.py  # Download of the course data (python -m planner.fetch_data)
└── assets/            # Public scenarios (fetched) and motion primitives
simulation/            # SUMO integration
```

See the [planner package guide](planner/README.md) for its modules and Python APIs.

## Choose your planner

Edit [my_submission/student_planner.py](my_submission/student_planner.py) directly.
Leave **exactly one option active**, including its imports and `create_planner()`
factory. Keep your helper modules in the same folder.

| Option | Starting point | Tutorial |
| --- | --- | --- |
| Search (active by default) | Use SMP A* or replace it with your exercise class | [Reuse a search class](tutorials/04_reuse_and_evaluate_planners.ipynb#Reuse-a-search-class) |
| Reactive (commented) | Enable `reactive_spec()` and configure the sampling-based planner | [Reactive planner](tutorials/02_example_planners.ipynb#5.-Observe-reactive-planner-behavior) |
| Custom (commented) | Extend the route-following dummy in [custom_planner.py](my_submission/custom_planner.py), or import your own class | [Use a custom planner](tutorials/04_reuse_and_evaluate_planners.ipynb#Use-a-custom-planner) |

Both supplied planners use the shared [lane-following predictor](tutorials/00_interactive_search_quickstart.ipynb#Predict-traffic). The custom starter includes an optional prediction helper.

The runner loads `create_planner() -> PlannerSpec` once per scenario. Configure the
planner and replanning interval there. Your planner implements
`plan(scenario, planning_problem) -> Trajectory`; the framework handles simulation,
final validation and scoring.

## Run a scenario

From the repository root, run your saved planner on the Augsburg starter:

```sh
python -m planner --scenarios planner/assets/scenarios/public/DEU_AugsburgCRC26P-23_16300_I-1.xml
```

| CLI option | Purpose |
| --- | --- |
| `--public` | Run all 100 public tasks instead of selecting a scenario |
| `--scenarios PATH` | Read an interactive XML file or a directory |
| `--submission PATH` | Select a source folder; defaults to `my_submission/` |
| `--solutions PATH` | Choose the result directory; defaults to `solutions/` |

Choose cases from the [benchmark catalogue](tutorials/public_benchmarks.md).
See [tutorial 04](tutorials/04_reuse_and_evaluate_planners.ipynb#Run-the-starters)
for execution settings, Python examples and failure feedback. For an automated
batch with progress and result reports, use [tutorial 05](tutorials/05_batch_processing.ipynb).

## Submit on Artemis

This repository is your Artemis exercise. **Every push is graded**: commit your changes
and push them.

```sh
git add my_submission
git commit -m "Improve my planner"
git push
```

On every push, Artemis runs the planner of your `my_submission/` folder, exactly as
`python -m planner --submission my_submission` does, with the competition settings
(SM1, seed 1234, at most 200 steps):

- **Public tests**: the three introductory scenarios
  (`DEU_AugsburgCRC26P-23_16300_I-1`, `DEU_FlensburgCRC26E-71_11400_I-1` and
  `DEU_ErlangenCRC26P-12_17300_I-1`). Each test fails if your planner crashes, takes longer
  than **60 seconds** (including starting Python and loading your planner), or drives a
  solution that is not valid. The test result on Artemis shows the reason.
- **Leaderboard**: a fixed set of further public scenarios. The SM1 cost of every valid
  solution is posted to the competition leaderboard. This test fails only if no scenario
  is solved.

What Artemis uses from your repository:

- **Only `my_submission/`.** The framework (`planner/`, `simulation/`, `commonroad_rp/`,
  SMP and all packages of `pyproject.toml`) comes from the grading image, in exactly the
  version of this repository. Changes to it outside `my_submission/` are ignored on
  Artemis, so keep your own helper modules and assets inside `my_submission/`.
- **Extra Python packages**: list them in `my_submission/requirements.txt`; Artemis
  installs them before running your planner.
- **Not your result files**: Artemis runs your planner again and simulates the traffic
  itself; `solutions/` stays ignored by Git.

Test locally before pushing: `python -m planner --scenarios <scenario>` with the same
scenario gives the same result as Artemis.

## Save results for submission

The CLI saves completed runs under `solutions/<scenario_id>/`. Check results with
[tutorial 03](tutorials/03_solution_and_cost_validation.ipynb), then package the
exact generating source and configuration:

```sh
python -m planner.runtime.submission my_submission submission.zip
```

For us to evaluate and reproduce your results, please submit these together:

- Public result folders, each containing `solution.xml`, `scenario.xml` and
  `metadata.json` from the same run.
- `submission.zip` with the exact planner source, configuration and required assets
  used to generate the results.
- The repository-root `pyproject.toml` and `poetry.lock` used for those runs,
  even if unchanged. Submit them separately from the planner bundle.
- Optionally, a short `environment.md` describing any steps beyond the standard
  Conda/Poetry setup needed to reproduce your results.

Keep required files inside `my_submission/`; the loader, ZIP and source fingerprint
include them regardless of extension. Git/editor metadata, caches, environments,
build folders and generated `outputs/` and `solutions/` are excluded. Copy files
instead of using symlinks, and write the ZIP outside `my_submission/`.

Use the [Conda/Poetry setup](#install). Add extra Python dependencies through Poetry,
then run `poetry install --with dev,test` before generating results. Submit those
same dependency files; do not regenerate the lockfile afterward.

For Docker, also update the Dockerfile's explicit package pins to match your lockfile
and rebuild. Poetry dependency changes alone do not update the image.

See [tutorial 04](tutorials/04_reuse_and_evaluate_planners.ipynb#Package-and-submit)
for notebook export, packaging and loading bundled assets relative to your Python file.

TUM participants submit by pushing this repository to Artemis, see
[Submit on Artemis](#submit-on-artemis). External participants contact [commonroad@lists.lrz.de](mailto:commonroad@lists.lrz.de)
for submission access.

Urban ranking counts per-scenario first places by lowest valid SM1, with equal
weight for all 100 public tasks. All evaluation scenarios are available in the
[benchmark catalogue](tutorials/public_benchmarks.md).

## Docker and VS Code

```sh
docker build --platform linux/amd64 -t commonroad-competition:2026-smp .
mkdir -p solutions
docker run --rm --platform linux/amd64 --cpus=1 --memory=4g --memory-swap=4g \
  --network none \
  --mount type=bind,source="$(pwd)/my_submission",target=/assignment,readonly \
  --mount type=bind,source="$(pwd)/solutions",target=/commonroad/solutions \
  commonroad-competition:2026-smp \
  --scenarios planner/assets/scenarios/public/DEU_AugsburgCRC26P-23_16300_I-1.xml \
  --submission /assignment \
  --solutions /commonroad/solutions
```

The image build uses the course data in your checkout, or downloads it if
[it is missing](#course-data); the build therefore needs network access the first time.
The source mount runs your current saved planner. For another source bundle, change
the mount's source path. `--target development` includes notebooks/tests.
VS Code's **Dev Containers: Reopen in Container** uses development.
Adapt Windows mount paths to your shell.

| Make command | Action |
| --- | --- |
| `make build` / `make run` | Build and run saved `my_submission/` on the Augsburg starter |
| `make run ARGS=--public` | Run all public tasks |
| `make build-dev` / `make test` | Build the development image / run tests |

Override `IMAGE`, `TAG` or `PLATFORM` as needed. Generated results stay outside
image builds. The tests are not part of the repository; fetch them with
`python -m planner.fetch_data --tests` before `make build-dev` / `make test` or the native
tests: `python -m pytest tests -q --disable-warnings`.

## License

Original code and documentation use the [MIT License](LICENSE). Bundled third-party
code/assets retain their own terms, including BSD-3-Clause for the
[reactive planner](commonroad_rp/LICENSE) and [route matcher](planner/common/vendor/LICENSE.commonroad_dc).
