#!/usr/bin/env python3
"""Version management script for Lumice.

Usage:
    python scripts/version.py check              # Compare CMakeLists.txt version with latest git tag
    python scripts/version.py check --tag v4.0.0 # Compare CMakeLists.txt version with specified tag (CI mode)
    python scripts/version.py set 4.1.0          # Update CMakeLists.txt version
"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

CMAKELISTS = Path(__file__).resolve().parent.parent / "CMakeLists.txt"
CMAKE_VERSION_RE = re.compile(r"(project\s*\(\s*Lumice\s+VERSION\s+)(\d+\.\d+\.\d+)(\s*\))")
SEMVER_RE = re.compile(r"^\d+\.\d+\.\d+$")


def read_cmake_version() -> str:
    """Read version from CMakeLists.txt."""
    text = CMAKELISTS.read_text()
    m = CMAKE_VERSION_RE.search(text)
    if not m:
        print(f"Error: cannot extract version from {CMAKELISTS}", file=sys.stderr)
        print("Expected pattern: project(Lumice VERSION X.Y.Z)", file=sys.stderr)
        sys.exit(1)
    return m.group(2)


def write_cmake_version(version: str) -> None:
    """Write version to CMakeLists.txt. Exits if replacement count != 1."""
    text = CMAKELISTS.read_text()
    new_text, count = CMAKE_VERSION_RE.subn(rf"\g<1>{version}\g<3>", text)
    if count != 1:
        print(f"Error: expected exactly 1 replacement, got {count}", file=sys.stderr)
        sys.exit(1)
    CMAKELISTS.write_text(new_text)


def read_git_tag_version() -> str:
    """Read version from the latest reachable git tag (strips 'v' prefix).

    Uses ``git describe --tags --abbrev=0``, which returns the most recent tag
    reachable from HEAD. This may differ from the chronologically newest tag
    if the current branch has diverged.
    """
    try:
        tag = subprocess.check_output(
            ["git", "describe", "--tags", "--abbrev=0"],
            stderr=subprocess.PIPE,
            text=True,
        ).strip()
    except subprocess.CalledProcessError:
        print("Error: no git tags found", file=sys.stderr)
        sys.exit(1)
    return tag.removeprefix("v")


def strip_tag_prefix(tag: str) -> str:
    """Strip 'v' prefix from a tag string."""
    return tag.removeprefix("v")


def cmd_check(args: argparse.Namespace) -> None:
    """Check that CMakeLists.txt version matches the git tag."""
    cmake_ver = read_cmake_version()

    if args.tag:
        tag_ver = strip_tag_prefix(args.tag)
        source = f"--tag {args.tag}"
    else:
        tag_ver = read_git_tag_version()
        source = f"git describe (v{tag_ver})"

    if cmake_ver == tag_ver:
        print(f"OK: CMakeLists.txt version ({cmake_ver}) matches {source}")
    else:
        print(f"MISMATCH: CMakeLists.txt version ({cmake_ver}) != {source} ({tag_ver})", file=sys.stderr)
        sys.exit(1)


def cmd_set(args: argparse.Namespace) -> None:
    """Set version in CMakeLists.txt."""
    version = args.version
    if not SEMVER_RE.match(version):
        print(f"Error: '{version}' is not a valid semver (expected X.Y.Z)", file=sys.stderr)
        sys.exit(1)

    old_ver = read_cmake_version()
    write_cmake_version(version)
    print(f"Updated CMakeLists.txt version: {old_ver} -> {version}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Lumice version management")
    sub = parser.add_subparsers(dest="command", required=True)

    check_parser = sub.add_parser("check", help="Check version consistency")
    check_parser.add_argument("--tag", help="Compare against this tag (CI mode, e.g. v4.0.0)")

    set_parser = sub.add_parser("set", help="Set version in CMakeLists.txt")
    set_parser.add_argument("version", help="Version to set (X.Y.Z)")

    args = parser.parse_args()
    if args.command == "check":
        cmd_check(args)
    elif args.command == "set":
        cmd_set(args)


if __name__ == "__main__":
    main()
