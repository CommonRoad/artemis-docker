# artemis-docker

Docker images in which Artemis grades the CommonRoad programming exercises. Each exercise's build runs in its image, which already has the fixed grading dependencies installed, so a build does not install them again.

| Image | Exercise | What it contains |
|---|---|---|
| `ghcr.io/commonroad/artemis-docker/bonus` | Bonus Challenge | pytest, `commonroad-io`, `commonroad-drivability-checker`, `commonroad-vehicle-models`, `triangle`, `commonroad-clcs`. The build script installs nothing. |
| `ghcr.io/commonroad/artemis-docker/competition` | Motion Planning Competition | The grading dependencies in `competition/requirements.txt`. The build script installs the student's own dependencies into a separate venv at build time. |

CI pushes each image to this project's registry and to the public GitHub Container Registry of the CommonRoad organization. Artemis pulls from ghcr.io without logging in; its build agents cannot pull from this registry, which listens on port 5005.

## Updating an image

1. Change `<name>/Dockerfile` (or `competition/requirements.txt`) and push to `main`.
2. CI builds the image and pushes it with the tags `<short commit SHA>` and `latest`. The last line of the job log is the full ghcr.io image reference.
3. In Artemis, open the exercise's build configuration and set **Docker Image** to that reference.

Artemis always uses a SHA tag, so a new build never changes a running exercise until its Docker Image field is updated. A build can also be started by hand with **Build → Pipelines → Run pipeline**.

## ghcr.io access

The push to ghcr.io uses the CI/CD variables `GHCR_USER` and `GHCR_TOKEN` (masked): a GitHub account in the CommonRoad organization and a token of it with the `write:packages` scope. Without them, CI pushes to this project's registry only. Both packages must be public on GitHub (package settings → Change visibility).
