"""Create a deterministic solutions.zip from complete export_result() folders."""

from __future__ import annotations

import argparse
from pathlib import Path
import stat
import tempfile
import warnings
import zipfile

from artifact_archive import (
    ARTIFACT_NAMES, COPY_CHUNK_BYTES, MAX_ARTIFACT_FILES, MAX_CASE_BYTES,
    MAX_COMPRESSED_BYTES, MAX_EXPANDED_BYTES,
)


def _same_bytes(left: Path, right: Path) -> bool:
    if not right.is_file() or left.stat().st_size != right.stat().st_size:
        return False
    with left.open("rb") as first, right.open("rb") as second:
        while chunk := first.read(COPY_CHUNK_BYTES):
            if chunk != second.read(COPY_CHUNK_BYTES):
                return False
    return True


def package_results(directory: Path, destination: Path) -> list[str]:
    """Package immediate complete folders, excluding unrelated/incomplete local output."""
    if directory.is_symlink() or not directory.is_dir():
        raise ValueError("the solutions directory must be a real directory")
    if destination.is_symlink():
        raise ValueError("the destination ZIP must not be a symlink")
    paths: list[tuple[str, Path]] = []
    total = 0
    for folder in sorted(directory.iterdir()):
        if folder.is_symlink():
            raise ValueError(f"refusing symlink export folder: {folder.name}")
        if not folder.is_dir():
            continue
        files = [folder / name for name in ARTIFACT_NAMES]
        if any(path.is_symlink() for path in files):
            raise ValueError(f"refusing symlink artifact in {folder.name}")
        if not all(path.is_file() for path in files):
            warnings.warn(f"Skipping incomplete export folder: {folder.name}")
            continue
        size = sum(path.stat().st_size for path in files)
        total += size
        if size > MAX_CASE_BYTES or total > MAX_EXPANDED_BYTES:
            raise ValueError("exports exceed a case or total expanded ZIP limit")
        paths.extend((f"solutions/{folder.name}/{path.name}", path) for path in files)
    if len(paths) > MAX_ARTIFACT_FILES:
        raise ValueError("exports exceed the 300-artifact-file ZIP limit")
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=destination.parent, suffix=".zip", delete=False) as temp:
        staged = Path(temp.name)
    try:
        streamed_total = 0
        streamed_cases: dict[str, int] = {}
        with zipfile.ZipFile(
            staged, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6,
        ) as archive:
            for name, source in sorted(paths):
                info = zipfile.ZipInfo(name, date_time=(1980, 1, 1, 0, 0, 0))
                info.create_system = 3
                info.external_attr = (stat.S_IFREG | 0o644) << 16
                info.compress_type = zipfile.ZIP_DEFLATED
                info._compresslevel = 6
                with source.open("rb") as stream, archive.open(info, "w") as target:
                    while chunk := stream.read(COPY_CHUNK_BYTES):
                        streamed_total += len(chunk)
                        case = source.parent.name
                        streamed_cases[case] = streamed_cases.get(case, 0) + len(chunk)
                        if (streamed_cases[case] > MAX_CASE_BYTES
                                or streamed_total > MAX_EXPANDED_BYTES):
                            raise ValueError("exports grew past expanded ZIP limits while packaging")
                        target.write(chunk)
        if staged.stat().st_size > MAX_COMPRESSED_BYTES:
            raise ValueError("solutions.zip exceeds the 32 MiB compressed limit")
        if not _same_bytes(staged, destination):
            staged.replace(destination)
    finally:
        staged.unlink(missing_ok=True)
    return sorted({source.parent.name for _, source in paths})


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--solutions", type=Path, default=Path("solutions"))
    parser.add_argument("--output", type=Path, default=Path("solutions.zip"))
    args = parser.parse_args()
    names = package_results(args.solutions, args.output)
    print(f"Packaged {len(names)} complete benchmark folder(s) into {args.output}")


if __name__ == "__main__":
    main()
