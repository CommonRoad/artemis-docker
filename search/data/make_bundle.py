"""Builds the data bundle of the CommonRoad Search exercise.

The bundle holds the files that never change during an edition and are
therefore kept out of the Artemis repositories: the exercise and tutorial
scenarios and the motion primitives with their pickled automata. It is used
in two places:

- search/Dockerfile copies the exercise scenarios into the grading image.
- SMP/exercise_data.py in the exercise's Template repository downloads the
  bundle into a student's checkout the first time it is needed.

Usage, from a complete checkout of the exercise (one that still has the files):

    python make_bundle.py <checkout> <version>

It writes commonroad-search-data-<version>.tar.xz next to this script and
prints the values to paste into search/Dockerfile and SMP/exercise_data.py.
The archive depends only on the file contents: with the same Python and
liblzma, the same input files give the same SHA-256. Building it takes a
few minutes.
"""

import hashlib
import io
import sys
import tarfile
from pathlib import Path

# Member paths are relative to the checkout root. The download code and the
# Dockerfile rely on exactly these four folders.
GROUPS = (
    ("scenarios/exercise", "*.xml"),
    ("scenarios/tutorial", "*.xml"),
    ("SMP/maneuver_automaton/primitives", "*.xml"),
    ("SMP/maneuver_automaton/automata", "*.automaton"),
)


def collect(checkout):
    members = []
    for folder, pattern in GROUPS:
        files = sorted((checkout / folder).glob(pattern), key=lambda p: p.name)
        if not files:
            sys.exit(f"No {pattern} files in {checkout / folder}")
        print(f"{folder + '/' + pattern:<50} {len(files):>4} files")
        members.extend((f"{folder}/{path.name}", path) for path in files)
    return members


def build(checkout, version):
    target = Path(__file__).resolve().parent / f"commonroad-search-data-{version}.tar.xz"
    members = collect(checkout)
    unpacked = 0
    with tarfile.open(target, mode="w:xz", format=tarfile.USTAR_FORMAT, preset=6) as tar:
        for name, path in members:
            data = path.read_bytes()
            # Fixed metadata, so that the archive depends on the file contents only.
            info = tarfile.TarInfo(name)
            info.size = len(data)
            info.mode = 0o644
            info.mtime = 0
            tar.addfile(info, io.BytesIO(data))
            unpacked += len(data)
    packed = target.read_bytes()

    print(f"\n{target.name}")
    print(f"  files:        {len(members)}")
    print(f"  unpacked:     {unpacked / 1e6:.1f} MB")
    print(f"  ARCHIVE_SIZE   = {len(packed)}")
    print(f"  ARCHIVE_SHA256 = \"{hashlib.sha256(packed).hexdigest()}\"")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    build(Path(sys.argv[1]).resolve(), sys.argv[2])
