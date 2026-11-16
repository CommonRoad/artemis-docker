"""Builds the data bundle of the CommonRoad competition (Urban Challenge).

The bundle holds the files that are kept out of the competition's Artemis
template repository because they are large and never changed by students:

- planner/assets/scenarios/public/*.xml   the 100 public scenarios
- vendor/commonroad_search-*.whl          the bundled SMP planner wheel
- tests/                                  the framework's developer tests

They come from the branch develop/ws_26 of
https://gitlab.lrz.de/tum-cps/commonroad-docker-submission. The template
downloads the bundle with planner/competition_data.py (run by hand with
"python -m planner.fetch_data").

Usage, from a complete checkout of commonroad-docker-submission (one that
still has the files):

    python make_bundle.py <checkout> <version>

It writes commonroad-competition-data-<version>.tar.xz next to this script and
prints the values to paste into planner/competition_data.py. The archive
depends only on the file contents: with the same Python and liblzma, the
same input files give the same SHA-256.
"""

import hashlib
import io
import subprocess
import sys
import tarfile
from pathlib import Path

# Member paths are relative to the checkout root. planner/competition_data.py
# relies on exactly these groups.
GROUPS = (
    ("planner/assets/scenarios/public", "*.xml"),
    ("vendor", "commonroad_search-*.whl"),
    ("tests", "**/*"),
)


def collect(checkout):
    # Only files tracked by Git, so caches and local files never end up in the bundle.
    tracked = set(
        subprocess.run(
            ["git", "-C", str(checkout), "ls-files", "-z"], check=True, capture_output=True, text=True
        ).stdout.split("\0")
    )
    members = []
    for folder, pattern in GROUPS:
        files = sorted(
            path.relative_to(checkout).as_posix()
            for path in (checkout / folder).glob(pattern)
            if path.is_file() and path.relative_to(checkout).as_posix() in tracked
        )
        if not files:
            sys.exit(f"No tracked {pattern} files in {checkout / folder}")
        print(f"{folder + '/' + pattern:<50} {len(files):>4} files")
        members.extend((name, checkout / name) for name in files)
    return members


def build(checkout, version):
    target = Path(__file__).resolve().parent / f"commonroad-competition-data-{version}.tar.xz"
    members = collect(checkout)
    unpacked = 0
    with tarfile.open(target, mode="w:xz", format=tarfile.PAX_FORMAT, preset=9) as tar:
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
    commit = subprocess.run(
        ["git", "-C", str(checkout), "rev-parse", "HEAD"], check=True, capture_output=True, text=True
    ).stdout.strip()

    print(f"\n{target.name}")
    print(f"  source commit: {commit}")
    print(f"  files:         {len(members)}")
    print(f"  unpacked:      {unpacked / 1e6:.1f} MB")
    print(f"  ARCHIVE_SIZE   = {len(packed)}")
    print(f"  ARCHIVE_SHA256 = \"{hashlib.sha256(packed).hexdigest()}\"")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    build(Path(sys.argv[1]).resolve(), sys.argv[2])
