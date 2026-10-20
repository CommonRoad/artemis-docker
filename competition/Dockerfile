# Grading image for the Motion Planning Competition exercise.
#
# The grading dependencies (requirements.txt) are fixed, so they are baked
# into the system site-packages here. The build script creates venv_test with
# --system-site-packages to use them, and installs the student's own
# dependencies into a separate venv_student at build time. See README.md for
# how this image is built, published and selected in Artemis.

FROM python:3.10-bookworm

# - curl, git: not used by the build script or the tests; can be dropped
# - build-essential, cmake, libspatialindex-dev: needed to build
#   commonroad-drivability-checker's native extensions from source
RUN apt-get update -qq \
    && apt-get install -y -qq --no-install-recommends \
       curl git build-essential cmake libspatialindex-dev \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt
