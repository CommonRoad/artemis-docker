# Grading image for the CommonRoad Search exercise (the Bonus Challenge with
# its scenarios kept out of the Artemis repositories).
#
# It is the bonus image plus the 500 ground-truth scenarios. The scenarios
# never change between builds and are about 370 MB of XML, so they are baked
# in here instead of being part of the exercise's Test repository. See
# README.md for how this image is built, published and selected in Artemis.

# Unpacks the data bundle of the exercise (data/make_bundle.py builds it).
# Only the exercise scenarios are copied into the image below; the rest of
# the bundle (tutorial scenarios, motion primitives) is for the students.
FROM python:3.11-slim-trixie AS data
ARG DATA_VERSION=2026.1
ARG DATA_SHA256=ef7fd2448bf1b5863e9eb26929855505f105f6df2f075ff32140a95c0f5b0678
COPY data/commonroad-search-data-${DATA_VERSION}.tar.xz /tmp/data.tar.xz
RUN echo "${DATA_SHA256}  /tmp/data.tar.xz" | sha256sum -c - \
    && mkdir -p /opt/commonroad-search-data \
    && python -c "import tarfile; tarfile.open('/tmp/data.tar.xz').extractall('/opt/commonroad-search-data', filter='data')" \
    && test "$(ls /opt/commonroad-search-data/scenarios/exercise | wc -l)" -eq 500

# The grading dependencies are those of the bonus image, pinned to the build
# that the Bonus Challenge is graded with, so that both exercises validate
# solutions identically. After a change to ../bonus/Dockerfile, set this tag
# to the new bonus image.
FROM ghcr.io/commonroad/artemis-docker/bonus:1136ffe5

# Ground-truth scenarios, against which the tests check the submitted
# solutions (behavior/test_solutions.py reads them from this folder).
COPY --from=data /opt/commonroad-search-data/scenarios/exercise \
     /opt/commonroad-search-data/scenarios/exercise
