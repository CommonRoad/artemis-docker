# Grading image for the Bonus Challenge exercise.
#
# The grading dependencies never change between builds, so they are baked in
# here and the build script installs nothing. See README.md for how this
# image is built, published and selected in Artemis.

FROM python:3.11-slim-trixie

# - curl: not used by the build script; can be dropped
# - build-essential, cmake, libspatialindex-dev: needed to build
#   commonroad-drivability-checker's native extensions from source
RUN apt-get update -y -q \
    && apt-get install -y --no-install-recommends \
       curl build-essential cmake libspatialindex-dev -q \
    && rm -rf /var/lib/apt/lists/*

# Grading dependencies. They must match the versions in the exercise's
# template and solution (pyproject.toml / poetry.lock), or a solution that is
# valid under one drivability-checker version may not validate under another.
RUN pip install --no-cache-dir \
    pytest==8.3.4 \
    "commonroad-io==2024.3" \
    "commonroad-drivability-checker==2025.3.1" \
    "commonroad-vehicle-models==3.0.2" \
    "triangle" \
    "commonroad-clcs"
