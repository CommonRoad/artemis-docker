"""Fetches course data omitted from slim checkouts; reuses complete local assets.

    python -m planner.fetch_data                      the public scenarios and the SMP wheel
    python -m planner.fetch_data --tests              also the framework's tests
    python -m planner.fetch_data --force              download and unpack again
    python -m planner.fetch_data --archive <file>     unpack an archive downloaded in a browser

Run it once after cloning, before "poetry install". It needs only the Python
standard library. See planner/competition_data.py.
"""

import argparse
import sys

from planner import competition_data


def main():
    parser = argparse.ArgumentParser(description="Fetch the public scenarios and the SMP wheel of the competition.")
    parser.add_argument("--tests", action="store_true", help="also fetch the framework's tests into tests/")
    parser.add_argument("--force", action="store_true", help="download and unpack again, even if all files are there")
    parser.add_argument("--archive", help=f"use this downloaded copy of {competition_data.ARCHIVE_NAME}")
    arguments = parser.parse_args()
    groups = competition_data.DEFAULT_GROUPS + (("tests",) if arguments.tests else ())
    try:
        changed = competition_data.fetch(
            groups, archive=arguments.archive, force=arguments.force or bool(arguments.archive)
        )
    except Exception as error:
        print(competition_data.help_text(error), file=sys.stderr)
        return 1
    if not changed:
        print(f"CommonRoad competition: all course data is in place (version {competition_data.DATA_VERSION}).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
