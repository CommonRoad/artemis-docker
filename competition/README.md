# CommonRoad Competition — Stage 1: Urban Challenge

This repository provides the environment and starter planners for **Stage 1: Urban
Challenge** of the CommonRoad competition.

Build a planner for interactive urban traffic simulated by SUMO using the supplied
search or reactive planner, or your own implementation. Submit the executed ego trajectory, simulated
traffic from the same run, and the exact source and configuration that produced them.

## Course data

The full `commonroad-docker-submission` repository and the slim Artemis template
share the same code, README and tutorials. The full repository includes the 100 public
scenarios (`planner/assets/scenarios/public/`), bundled SMP wheel
(`vendor/commonroad_search-2025.1-py3-none-any.whl`) and framework tests (`tests/`).
The slim template omits these downloadable assets.

From either checkout, run the following before `poetry install`. It reuses assets
already present and fetches missing data once (about 8 MB). Poetry needs the wheel
before installation:

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
| `python -m planner.fetch_data --force` | Download and unpack again, for example after deleting or editing a scenario |
| `python -m planner.fetch_data --archive FILE` | Unpack an archive downloaded in a browser, without network access |

If the automatic download does not work (for example behind a restrictive network),
the error message prints a download link for the archive; download it in a browser and
pass it with `--archive`.

In the slim template, Git ignores downloaded assets. The full repository retains its
tracked copies. Do not change the public scenarios: grading uses the canonical tasks.

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

Run the steps below from your personal Artemis checkout. Artemis evaluates your
saved results; it does not run your planner or install your dependencies.

### 1. Run and review your planner

Save your planner and required assets in `my_submission/`, then run the public tasks:

```sh
python -m planner --public
```

This uses the competition settings and saves completed runs in
`solutions/<scenario_id>/`. For batch progress and reports, use
[tutorial 05](tutorials/05_batch_processing.ipynb).

Check your results with [tutorial 03](tutorials/03_solution_and_cost_validation.ipynb).
A failed rerun can leave an older result folder: review these before packaging.
Keep the exact source, `pyproject.toml` and `poetry.lock` used for your runs.

<a id="save-results-for-submission"></a>

### 2. Package your results and source

```sh
python3 package_results.py --solutions solutions --output solutions.zip
python -m planner.runtime.submission my_submission submission.zip
```

- **`solutions.zip`** contains the saved results for grading. Each result folder must
  contain `scenario.xml`, `solution.xml` and `metadata.json` from the same run.
  Use the exported files without editing them.
- **`submission.zip`** contains your planner source and assets for reproduction.

**Check the size of `solutions.zip`. If it is larger than 30 MB, contact
[commonroad@lists.lrz.de](mailto:commonroad@lists.lrz.de) before submitting.**

### 3. Commit and push

```sh
git add solutions.zip submission.zip my_submission pyproject.toml poetry.lock
git commit -m "Submit saved competition results"
git push
```

Keep expanded `solutions/` Git-ignored. If reproduction needs extra setup steps,
include an `environment.md` and add it to the commit too.

### 4. Check your Artemis feedback

You need **at least 30 distinct valid solutions** to pass and enter the leaderboard.
Artemis checks the saved ego trajectories against their recorded traffic and
recomputes validity and SM1 costs. A valid solution with a scoring error still
counts toward 30, but receives no ranking score.

All **100 public benchmarks** count equally. Ranking counts per-scenario first
places by lowest valid SM1. Your latest submission replaces your previous results;
falling below 30 valid solutions removes leaderboard eligibility. Check the build
feedback for missing, invalid or unscored cases.

<details>
<summary>Custom dependencies and packaging help</summary>

Add dependencies through Poetry and run `poetry install --with dev,test` before
producing results. Submit the same dependency files used for those runs. For Docker,
also update its explicit package pins to match the lockfile and rebuild.

Keep required configs, model weights and data inside `my_submission/`, using files
rather than symlinks. See [tutorial 04](tutorials/04_reuse_and_evaluate_planners.ipynb#Package-and-submit)
for notebook export and packaging examples.

</details>

External participants can contact [commonroad@lists.lrz.de](mailto:commonroad@lists.lrz.de)
for submission access.

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
the mount's source path. `--target development` includes notebook tools.
VS Code's **Dev Containers: Reopen in Container** uses development.
Adapt Windows mount paths to your shell.

| Make command | Action |
| --- | --- |
| `make build` / `make run` | Build and run saved `my_submission/` on the Augsburg starter |
| `make run ARGS=--public` | Run all public tasks |
| `make build-dev` | Build the development image with notebook tools |

Override `IMAGE`, `TAG` or `PLATFORM` as needed. Generated results stay outside
image builds.

## License

Original code and documentation use the [MIT License](LICENSE). Bundled third-party
code/assets retain their own terms, including BSD-3-Clause for the
[reactive planner](commonroad_rp/LICENSE) and [route matcher](planner/common/vendor/LICENSE.commonroad_dc).
