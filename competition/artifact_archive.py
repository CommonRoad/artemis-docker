"""Bounded ZIP handling for paired competition artifacts; never extract a whole archive."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass, field
import errno
from pathlib import Path
import stat
import tempfile
from typing import Iterator
import zipfile

ARTIFACT_NAMES = ("scenario.xml", "solution.xml", "metadata.json")
MAX_COMPRESSED_BYTES = 32 * 1024 * 1024
MAX_EXPANDED_BYTES = 512 * 1024 * 1024
MAX_CASE_BYTES = 32 * 1024 * 1024
MAX_ARTIFACT_FILES = 300
ALLOWED_COMPRESSION = {zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED}
COPY_CHUNK_BYTES = 64 * 1024


class ArtifactError(ValueError):
    """A submitted artifact cannot be accepted."""


@dataclass
class ArchivePlan:
    cases: list[str] = field(default_factory=list)
    incomplete: dict[str, str] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)


def inspect_archive(path: Path, canonical_ids: set[str]) -> ArchivePlan:
    """Check structure and advertised bounds before any worker reads artifact content.

    Structural errors reject the entire submission. A known folder with missing files
    is isolated as an invalid case. CRC and actual stream length checks happen when
    each complete case is read, so a damaged member need not discard other cases.
    """
    plan = ArchivePlan()
    by_case: dict[str, set[str]] = {}
    seen: set[str] = set()
    expanded = 0
    files = 0
    case_sizes: dict[str, int] = {}
    try:
        if path.is_symlink() or path.parent.is_symlink():
            plan.errors.append("solutions.zip must be a regular file, not a symlink")
            return plan
        if not path.is_file():
            plan.errors.append("solutions.zip is missing; commit the paired-artifact ZIP")
            return plan
        if path.stat().st_size > MAX_COMPRESSED_BYTES:
            plan.errors.append("solutions.zip exceeds the 32 MiB compressed limit")
            return plan
        with zipfile.ZipFile(path) as archive:
            for entry in archive.infolist():
                name = entry.filename
                parts = name.rstrip("/").split("/")
                if entry.orig_filename != name:
                    raise ArtifactError("ZIP member contains a NUL or an ambiguous filename")
                if name in seen:
                    raise ArtifactError(f"duplicate ZIP member: {name}")
                seen.add(name)
                if (
                    "\\" in name or "\x00" in name or name.startswith("/")
                    or any(part in {"", ".", ".."} for part in parts)
                    or parts[0] != "solutions"
                ):
                    raise ArtifactError(f"unsafe or unexpected ZIP path: {name}")
                mode = stat.S_IFMT(entry.external_attr >> 16)
                if mode not in {0, stat.S_IFREG, stat.S_IFDIR}:
                    raise ArtifactError(f"ZIP member is a symlink or special file: {name}")
                if entry.flag_bits & 1:
                    raise ArtifactError(f"encrypted ZIP member: {name}")
                if entry.compress_type not in ALLOWED_COMPRESSION:
                    raise ArtifactError(f"unsupported ZIP compression: {name}")
                if entry.is_dir():
                    if name != "/".join(parts) + "/":
                        raise ArtifactError(f"non-canonical ZIP directory path: {name}")
                    if entry.file_size != 0 or entry.CRC != 0:
                        raise ArtifactError(f"ZIP directory carries unexpected content: {name}")
                    if len(parts) == 1:
                        continue
                    if len(parts) != 2 or parts[1] not in canonical_ids:
                        raise ArtifactError(
                            f"unknown benchmark or unexpected ZIP directory: {name}"
                        )
                    by_case.setdefault(parts[1], set())
                    continue
                if mode == stat.S_IFDIR:
                    raise ArtifactError(f"inconsistent ZIP directory entry: {name}")
                if len(parts) != 3 or parts[1] not in canonical_ids:
                    raise ArtifactError(f"unknown benchmark or unexpected ZIP member: {name}")
                if parts[2] not in ARTIFACT_NAMES:
                    raise ArtifactError(f"unexpected artifact file: {name}")
                if entry.file_size < 0 or entry.compress_size < 0:
                    raise ArtifactError(f"invalid ZIP member sizes: {name}")
                files += 1
                expanded += entry.file_size
                case_sizes[parts[1]] = case_sizes.get(parts[1], 0) + entry.file_size
                if files > MAX_ARTIFACT_FILES or expanded > MAX_EXPANDED_BYTES:
                    raise ArtifactError("ZIP exceeds the 300-file or 512 MiB expanded limit")
                if case_sizes[parts[1]] > MAX_CASE_BYTES:
                    raise ArtifactError(f"benchmark exceeds the 32 MiB expanded limit: {parts[1]}")
                by_case.setdefault(parts[1], set()).add(parts[2])
    except OSError as error:
        if error.errno in {errno.EIO, errno.EMFILE, errno.ENFILE, errno.ENOMEM}:
            raise
        plan.errors.append(str(error))
        return plan
    except (ValueError, zipfile.BadZipFile, NotImplementedError) as error:
        plan.errors.append(str(error))
        return plan
    for name, present in sorted(by_case.items()):
        missing = set(ARTIFACT_NAMES) - present
        if missing:
            plan.incomplete[name] = (
                "incomplete artifact folder; missing " + ", ".join(sorted(missing))
            )
        else:
            plan.cases.append(name)
    return plan


@contextmanager
def extracted_case(archive_path: Path, scenario_id: str) -> Iterator[Path]:
    """Read only three exact members with bounded streaming, CRC checks and cleanup."""
    with tempfile.TemporaryDirectory(prefix="competition-artifacts-") as temporary:
        directory = Path(temporary)
        total = 0
        with zipfile.ZipFile(archive_path) as archive:
            for filename in ARTIFACT_NAMES:
                member = archive.getinfo(f"solutions/{scenario_id}/{filename}")
                copied = 0
                with archive.open(member) as source, (directory / filename).open("wb") as target:
                    while chunk := source.read(COPY_CHUNK_BYTES):
                        copied += len(chunk)
                        total += len(chunk)
                        if copied > member.file_size or total > MAX_CASE_BYTES:
                            raise ArtifactError("artifact stream exceeds declared or case size limit")
                        target.write(chunk)
                if copied != member.file_size:
                    raise ArtifactError("artifact stream length differs from its declared size")
        yield directory
