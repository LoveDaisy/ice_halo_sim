"""Lumice binary discovery and execution."""

import os
import subprocess
from pathlib import Path

DEFAULT_TIMEOUT = 120  # seconds


def get_project_root() -> Path:
    """Return the project root directory (parent of e2e/)."""
    return Path(__file__).resolve().parent.parent


def find_lumice_binary() -> Path:
    """Find the Lumice executable.

    Search order:
      1. $LUMICE_BIN environment variable
      2. build/cmake_install/Lumice (release build output)

    Raises FileNotFoundError with actionable message if not found.
    """
    env_bin = os.environ.get("LUMICE_BIN")
    if env_bin:
        p = Path(env_bin)
        if p.is_file() and os.access(p, os.X_OK):
            return p
        raise FileNotFoundError(f"LUMICE_BIN={env_bin} is not a valid executable")

    root = get_project_root()
    candidate = root / "build" / "cmake_install" / "Lumice"
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return candidate

    raise FileNotFoundError(
        f"Lumice binary not found at {candidate}. "
        "Build it first: ./build.sh -j release"
    )


def run_lumice(args, timeout=DEFAULT_TIMEOUT):
    """Run Lumice with the given arguments.

    Args:
        args: List of command-line arguments (excluding the binary itself).
        timeout: Timeout in seconds (default 120). Raises subprocess.TimeoutExpired on timeout.

    Returns:
        subprocess.CompletedProcess with stdout and stderr captured as strings.
    """
    binary = find_lumice_binary()
    return subprocess.run(
        [str(binary)] + list(args),
        capture_output=True,
        text=True,
        timeout=timeout,
    )
