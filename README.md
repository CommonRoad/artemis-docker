# artemis-docker

Docker images in which Artemis grades the CommonRoad programming exercises. Each exercise's build runs in its image, which already has the fixed grading dependencies installed, so a build does not install them again.

| Image | Exercise | What it contains |
|---|---|---|
| `ghcr.io/commonroad/artemis-docker/bonus` | Bonus Challenge | pytest, `commonroad-io`, `commonroad-drivability-checker`, `commonroad-vehicle-models`, `triangle`, `commonroad-clcs`. The build script installs nothing. |
| `ghcr.io/commonroad/artemis-docker/competition` | Motion Planning Competition | The `evaluator` stage of the Template's Dockerfile: the framework (`planner`, `simulation`), SUMO, pytest and the 100 public scenarios. The build script installs only the student's own dependencies into a separate venv. |
| `ghcr.io/commonroad/artemis-docker/search` | CommonRoad Search (the Bonus Challenge without scenarios in its repositories) | The `bonus` image plus the 500 ground-truth scenarios in `/opt/commonroad-search-data/scenarios/exercise`. The build script installs nothing. |

CI pushes each image to this project's registry and to the public GitHub Container Registry of the CommonRoad organization. Artemis pulls from ghcr.io without logging in; its build agents cannot pull from this registry, which listens on port 5005.

## Updating an image

1. Change `<name>/Dockerfile` and push to `main`. For `competition`, see "Grading image of the competition" below.
2. CI builds the image and pushes it with the tags `<short commit SHA>` and `latest`. The last line of the job log is the full ghcr.io image reference.
3. In Artemis, open the exercise's build configuration and set **Docker Image** to that reference.

Artemis always uses a SHA tag, so a new build never changes a running exercise until its Docker Image field is updated. A build can also be started by hand with **Build → Pipelines → Run pipeline**.

## Exercise data of CommonRoad Search

The scenarios and motion primitives of the exercise never change during an edition and are about 400 MB of XML. Kept in the exercise's Artemis repositories, they are copied into every student repository and checked out for every build, so the `search` image and its exercise keep them here instead, as one archive: `search/data/commonroad-search-data-<version>.tar.xz` (about 20 MB).

- `search/Dockerfile` unpacks the archive and copies the exercise scenarios into the image, where the tests read them. Its base is a pinned `bonus` image, so the grading dependencies are defined in one place only, `bonus/Dockerfile`. After a change there, set the `FROM` tag in `search/Dockerfile` to the new `bonus` image.
- `SMP/exercise_data.py` in the exercise's Template repository downloads the archive from this repository into a student's checkout the first time `SMP` is imported. It pins the commit of the archive and its SHA-256.

The `bonus` image and exercises that use it are not affected: their Test repository still holds the scenarios.

To change the data (a new edition, or a corrected scenario):

1. Run `python search/data/make_bundle.py <complete checkout of the exercise> <new version>` and commit the new archive. Keep the old archive: checkouts of earlier Template versions still download it.
2. Set `DATA_VERSION` and `DATA_SHA256` in `search/Dockerfile` to the printed values and push to `main`. Use the new image in Artemis as described above.
3. In the exercise's Template and Solution repositories, set `DATA_VERSION`, `ARCHIVE_SIZE`, `ARCHIVE_SHA256` and `_ARCHIVE_COMMIT` (the commit of step 1) in `SMP/exercise_data.py`. Checkouts with the old data fetch the new archive on their next import of `SMP`.

## Grading image of the competition (Urban Challenge)

`competition/` is a snapshot of the competition's Artemis Template repository (the tracked files, without `.gitignore`), and CI builds the `evaluator` stage of its `Dockerfile`. The grading code is thus exactly the framework the students get. The Dockerfile downloads the course data (below) while it builds, so the snapshot has no scenarios.

After a change in the Template, replace `competition/` with the tracked files of the new Template commit (`git archive <commit> | tar -x -C competition`, then delete `competition/.gitignore`), push to `main`, and set the new tag in the exercise's Docker Image.

## Course data of the competition (Urban Challenge)

The competition's Artemis Template repository is a slim copy of the branch `develop/ws_26` of [commonroad-docker-submission](https://gitlab.lrz.de/tum-cps/commonroad-docker-submission) without three parts that students never change: the 100 public scenarios in `planner/assets/scenarios/public/` (about 61 MB of XML), the SMP wheel `vendor/commonroad_search-2025.1-py3-none-any.whl` and the framework's `tests/`. This repository keeps them as one archive: `competition-data/commonroad-competition-data-<version>.tar.xz` (about 7 MB). No image is built from `competition-data/`.

- `planner/competition_data.py` in the Template repository downloads the archive from this repository into a student's checkout: by hand with `python -m planner.fetch_data` (needed before `poetry install`, which installs the wheel), during the Docker build, and the first time `planner` is imported. It pins the commit of the archive and its SHA-256. The tests are only unpacked with `--tests`.

To change the data (new public scenarios, a new SMP wheel or new tests):

1. Run `python competition-data/make_bundle.py <complete checkout of commonroad-docker-submission> <new version>` and commit the new archive. Keep the old archive: checkouts of earlier Template versions still download it.
2. In the Template and Solution repositories, set `DATA_VERSION`, `ARCHIVE_SIZE`, `ARCHIVE_SHA256` and `_ARCHIVE_COMMIT` (the commit of step 1) in `planner/competition_data.py`, and the file counts in `GROUPS` if they changed.

## ghcr.io access

The push to ghcr.io uses the CI/CD variables `GHCR_USER` and `GHCR_TOKEN` (masked): a GitHub account in the CommonRoad organization and a token of it with the `write:packages` scope. Without them, CI pushes to this project's registry only. Every package must be public on GitHub (package settings → Change visibility); a new image name, such as `search`, starts out private.
