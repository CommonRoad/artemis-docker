"""Course data assets that may be omitted from slim checkouts.

The full repository includes these assets; the slim Artemis template omits them.
Missing assets are downloaded once and unpacked into the places where the
framework, tutorials, Dockerfile and Poetry expect them:

    planner/assets/scenarios/public/            100 public scenarios
    vendor/commonroad_search-2025.1-...whl        the bundled SMP planner wheel
    tests/                                        the framework's tests (only with --tests)

Poetry needs the wheel before "poetry install", so run this once right after
cloning, in the root folder of the checkout:

    python -m planner.fetch_data

This module uses only the Python standard library, so it works before
anything is installed. Afterwards, importing planner fetches missing
scenarios by itself. Git ignores downloaded assets in slim checkouts; the full
repository retains its already tracked copies.
"""

import fnmatch
import hashlib
import json
import os
import shutil
import sys
import tarfile
import tempfile
import time
import urllib.request
from pathlib import Path

# The archive is built by competition-data/make_bundle.py in the artemis-docker
# repository from the branch develop/ws_26 of commonroad-docker-submission. Its
# SHA-256 is checked before anything is unpacked.
DATA_VERSION = "2026.2"
ARCHIVE_NAME = f"commonroad-competition-data-{DATA_VERSION}.tar.xz"
ARCHIVE_SIZE = 7966244
ARCHIVE_SHA256 = "8639e0979b17ebc0002bd1b58c0b7675b20c2f8d9b1816e28abdb13ee0ad7f5c"
_ARCHIVE_COMMIT = "79d88b44e3f1daee44b97533c80fd1ed53f2ad62"
ARCHIVE_URLS = (
    f"https://gitlab.lrz.de/tum-cps/artemis-docker/-/raw/{_ARCHIVE_COMMIT}/competition-data/{ARCHIVE_NAME}",
    f"https://raw.githubusercontent.com/CommonRoad/artemis-docker/{_ARCHIVE_COMMIT}/competition-data/{ARCHIVE_NAME}",
)

PACKAGE_DIR = Path(__file__).resolve().parent
ROOT = PACKAGE_DIR.parent

# Group -> (folder in the archive and on disk, file pattern, number of files,
# whether the files may be in subfolders).
GROUPS = {
    "scenarios": ("planner/assets/scenarios/public", "*.xml", 100, False),
    "wheel": ("vendor", "commonroad_search-2025.1-py3-none-any.whl", 1, False),
    "tests": ("tests", "*", 48, True),
}
DEFAULT_GROUPS = ("scenarios", "wheel")

# All three are covered by the "/.commonroad-competition-data*" rule in .gitignore.
_MARKER = ROOT / ".commonroad-competition-data.json"
_LOCK = ROOT / ".commonroad-competition-data.lock"
_STAGING_PREFIX = ".commonroad-competition-data-"

_LOCK_WAIT_SECONDS = 600
_DOWNLOAD_TIMEOUT_SECONDS = 30

_HELP = """
CommonRoad competition: the course data could not be fetched:
    {error}
The public scenarios or SMP wheel are missing from this checkout. To get them, either
  - check your internet connection and run, in the root folder of your checkout:
        python -m planner.fetch_data
  - or download this file in a browser:
        {url}
    and run:
        python -m planner.fetch_data --archive <path to the downloaded file>
"""


class DataError(RuntimeError):
    """The data files could not be fetched or unpacked."""


def _say(message):
    print(f"CommonRoad competition: {message}", flush=True)


def _count(group):
    folder, pattern, _, recursive = GROUPS[group]
    target = ROOT / folder
    if recursive:
        return sum(
            1
            for path in target.rglob(pattern)
            if path.is_file() and "__pycache__" not in path.parts and not path.name.endswith(".pyc")
        )
    try:
        return sum(1 for entry in os.scandir(target) if entry.is_file() and fnmatch.fnmatch(entry.name, pattern))
    except OSError:
        return 0


def missing_groups(groups=DEFAULT_GROUPS):
    """The groups of which files are missing."""
    return [group for group in groups if _count(group) < GROUPS[group][2]]


def _marker():
    try:
        marker = json.loads(_MARKER.read_text(encoding="utf-8"))
        return marker if isinstance(marker, dict) else {}
    except (OSError, ValueError):
        return {}


def is_available(groups=DEFAULT_GROUPS):
    """True if every file of these groups is in place, in this data version."""
    if missing_groups(groups):
        return False
    marker = _marker()
    if not marker:
        # Complete, but not unpacked by this module: for example copied over
        # from another checkout. Files are moved into place whole, so a
        # complete set of them is also an intact one.
        try:
            _write_marker(groups)
        except OSError:
            pass  # a read-only checkout: check again next time
        return True
    return marker.get("version") == DATA_VERSION


def _write_marker(groups):
    groups = sorted(set(groups) | set(_marker().get("groups", ())))
    _MARKER.write_text(
        json.dumps({"version": DATA_VERSION, "sha256": ARCHIVE_SHA256, "groups": groups}) + "\n",
        encoding="utf-8",
    )


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _download(target):
    """Downloads the archive from the first URL that delivers the right file."""
    errors = []
    for url in ARCHIVE_URLS:
        try:
            request = urllib.request.Request(url, headers={"User-Agent": "commonroad-competition"})
            with urllib.request.urlopen(request, timeout=_DOWNLOAD_TIMEOUT_SECONDS) as response:
                with open(target, "wb") as out:
                    shutil.copyfileobj(response, out, 1 << 20)
        except Exception as error:  # every network and file error counts: the next URL may still work
            errors.append(f"{url}: {error}")
            continue
        if _sha256(target) == ARCHIVE_SHA256:
            return
        errors.append(f"{url}: the downloaded file is incomplete or not the expected one")
    raise DataError("; ".join(errors))


def _group_of(name):
    """The group an archive entry belongs to, or None if it is not an expected entry."""
    parts = name.split("/")
    if any(part in ("", ".", "..") or "\\" in part or ":" in part for part in parts):
        return None
    for group, (folder, pattern, _, recursive) in GROUPS.items():
        prefix = folder + "/"
        if not name.startswith(prefix):
            continue
        rest = name[len(prefix):]
        if (recursive or "/" not in rest) and fnmatch.fnmatch(rest.rpartition("/")[2], pattern):
            return group
    return None


def _unpack(archive, staging, groups):
    """Unpacks the wanted groups into the staging folder and returns (staged file, final file) pairs."""
    moves = []
    found = dict.fromkeys(GROUPS, 0)
    with tarfile.open(archive, "r:xz") as tar:
        for member in tar:
            group = _group_of(member.name)
            # Only plain files inside the known folders are accepted.
            if group is None or not member.isfile():
                raise DataError(f"unexpected entry in the archive: {member.name}")
            found[group] += 1
            if group not in groups:
                continue
            staged = staging / f"{len(moves)}"
            with tar.extractfile(member) as source, open(staged, "wb") as out:
                shutil.copyfileobj(source, out, 1 << 20)
            moves.append((staged, ROOT / member.name))
    for group, (_, _, count, _) in GROUPS.items():
        if found[group] != count:
            raise DataError(f"the archive holds {found[group]} files of {group} instead of {count}")
    return moves


def _install(archive, staging, groups):
    if _sha256(archive) != ARCHIVE_SHA256:
        raise DataError(f"{archive} is not {ARCHIVE_NAME} (its SHA-256 differs)")
    moves = _unpack(archive, staging, groups)
    for staged, final in moves:
        final.parent.mkdir(parents=True, exist_ok=True)
        os.replace(staged, final)
    _write_marker(groups)


def _acquire_lock(groups):
    """Returns True once this process holds the lock, False if another process finished the job."""
    deadline = time.monotonic() + _LOCK_WAIT_SECONDS
    announced = False
    while True:
        try:
            os.close(os.open(_LOCK, os.O_CREAT | os.O_EXCL | os.O_WRONLY))
            return True
        except FileExistsError:
            pass
        if is_available(groups):
            return False
        try:
            stale = time.time() - _LOCK.stat().st_mtime > _LOCK_WAIT_SECONDS
        except OSError:
            continue  # the other process just released the lock
        if stale or time.monotonic() > deadline:
            # Left behind by a process that was stopped half-way.
            try:
                _LOCK.unlink()
            except OSError:
                pass
            continue
        if not announced:
            _say("another process is fetching the course data, waiting for it ...")
            announced = True
        time.sleep(1)


def fetch(groups=DEFAULT_GROUPS, archive=None, force=False):
    """Puts the data files in place and returns True if anything had to be done.

    :param groups: names from GROUPS to unpack; by default the scenarios and the wheel
    :param archive: an already downloaded copy of the archive; by default it is downloaded
    :param force: unpack again even if all files are there
    """
    groups = tuple(groups)
    if not force and is_available(groups):
        return False
    if not _acquire_lock(groups):
        return False
    staging = None
    try:
        if not force and is_available(groups):
            return False
        staging = Path(tempfile.mkdtemp(prefix=_STAGING_PREFIX, dir=ROOT))
        if archive is None:
            _say(
                "the public scenarios or SMP wheel are missing from this checkout. "
                f"Downloading them once ({ARCHIVE_SIZE / 1e6:.0f} MB) ..."
            )
            archive = staging / ARCHIVE_NAME
            _download(archive)
        _install(Path(archive), staging, groups)
        folders = ", ".join(GROUPS[group][0] + "/" for group in groups)
        _say(f"the course data is ready in {folders}.")
        return True
    finally:
        if staging is not None:
            shutil.rmtree(staging, ignore_errors=True)
        try:
            _LOCK.unlink()
        except OSError:
            pass


def help_text(error):
    return _HELP.format(error=error, url=ARCHIVE_URLS[0])


def ensure_available():
    """Fetches missing scenarios and the wheel. Called when planner is imported; never raises.

    Does nothing outside of a checkout of the template (for example for a copy
    of planner installed into site-packages), when COMMONROAD_COMPETITION_SKIP_DATA
    is set, and while `python -m planner.fetch_data` runs, which reports by itself.
    """
    if os.environ.get("COMMONROAD_COMPETITION_SKIP_DATA"):
        return
    if "planner.fetch_data" in getattr(sys, "orig_argv", ()):
        return
    if not (ROOT / "pyproject.toml").is_file():
        return
    try:
        fetch()
    except Exception as error:  # importing planner must keep working without the data
        print(help_text(error), file=sys.stderr, flush=True)
