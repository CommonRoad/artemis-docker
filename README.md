# artemis-docker

Docker images in which Artemis grades the CommonRoad programming exercises. Each exercise's build runs in its image, which already has the fixed grading dependencies installed, so a build does not install them again.

| Image | Exercise | What it contains |
|---|---|---|
| `gitlab.lrz.de:5005/tum-cps/artemis-docker/bonus` | Bonus Challenge | pytest, `commonroad-io`, `commonroad-drivability-checker`, `commonroad-vehicle-models`, `triangle`, `commonroad-clcs`. The build script installs nothing. |
| `gitlab.lrz.de:5005/tum-cps/artemis-docker/competition` | Motion Planning Competition | The grading dependencies in `competition/requirements.txt`. The build script installs the student's own dependencies into a separate venv at build time. |

This project is public because Artemis pulls the images without logging in to the registry.

## Updating an image

1. Change `<name>/Dockerfile` (or `competition/requirements.txt`) and push to `main`.
2. CI builds the image and pushes it as `<name>:<short commit SHA>` and `<name>:latest`. The last line of the job log is the full image reference.
3. In Artemis, open the exercise's build configuration and set **Docker Image** to that reference.

Artemis always uses a SHA tag, so a new build never changes a running exercise until its Docker Image field is updated. A build can also be started by hand with **Build → Pipelines → Run pipeline**.
