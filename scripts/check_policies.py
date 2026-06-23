#!/usr/bin/env python3
"""Deterministic engineering-policy checks for Lumice.

This is the *executable* half of the policy documents under doc/: it verifies the
code itself, not a human's promise that they followed a rule. Run by the CI lint
job and by the local pre-commit hook (scripts/hooks/pre-commit). Exit code is 0
when clean, 1 when any violation is found (each printed as file:line).

Checks:
  1. env-getenv-centralization — std::getenv("LUMICE_...") may appear ONLY in
     src/util/env_knobs.cpp. Everything else must route through util/env_knobs
     (doc/env-var-policy.md).
  2. env-knob-registration — every LUMICE_* name read in env_knobs.cpp must be
     documented in doc/env-var-policy.md, so the knob is discoverable.
  3. gui-api-boundary — src/gui/** must not #include "core/..." or "config/...";
     the GUI talks to core only through the C API (src/include/lumice.h).
  4. no-using-namespace — `using namespace` is banned in src/ (AGENTS.md).

Add a new check as a function returning a list of Violation and append it to
CHECKS. Keep each check deterministic and artifact-inspecting.
"""
from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
ENV_KNOBS_CPP = SRC / "util" / "env_knobs.cpp"
POLICY_DOC = REPO_ROOT / "doc" / "env-var-policy.md"

CXX_SUFFIXES = {".cpp", ".cc", ".hpp", ".h", ".mm", ".inl"}


@dataclass
class Violation:
    path: Path
    line: int
    rule: str
    message: str

    def render(self) -> str:
        rel = self.path.relative_to(REPO_ROOT)
        return f"  {rel}:{self.line}: [{self.rule}] {self.message}"


def cxx_sources(root: Path):
    for p in sorted(root.rglob("*")):
        if p.suffix in CXX_SUFFIXES and p.is_file():
            yield p


def strip_comments(text: str) -> str:
    """Replace // and /* */ comments with spaces, preserving offsets and newlines.

    Respects string and char literals so that `//` or `/*` inside a literal (e.g.
    a "LUMICE_..." env name or a URL) is NOT mistaken for a comment. Newlines are
    kept so line numbers stay accurate.
    """
    out = []
    i, n = 0, len(text)
    state = "code"  # code | line_comment | block_comment | string | char
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                state = "line_comment"
                out.append("  ")
                i += 2
                continue
            if c == "/" and nxt == "*":
                state = "block_comment"
                out.append("  ")
                i += 2
                continue
            if c == '"':
                state = "string"
                out.append(c)
            elif c == "'":
                state = "char"
                out.append(c)
            else:
                out.append(c)
            i += 1
        elif state == "line_comment":
            out.append("\n" if c == "\n" else " ")
            if c == "\n":
                state = "code"
            i += 1
        elif state == "block_comment":
            out.append("\n" if c == "\n" else " ")
            if c == "*" and nxt == "/":
                out.append(" ")
                i += 2
                state = "code"
                continue
            i += 1
        elif state in ("string", "char"):
            out.append(c)
            if c == "\\":  # escape: copy next char verbatim
                if nxt:
                    out.append(nxt)
                    i += 2
                    continue
            elif (state == "string" and c == '"') or (state == "char" and c == "'"):
                state = "code"
            i += 1
    return "".join(out)


def code_lines(path: Path):
    """Yield (lineno, original_line, code_only_line) with comments blanked out."""
    text = path.read_text(encoding="utf-8", errors="replace")
    stripped = strip_comments(text)
    originals = text.splitlines()
    codes = stripped.splitlines()
    for idx, code in enumerate(codes):
        original = originals[idx] if idx < len(originals) else ""
        yield idx + 1, original, code


GETENV_LUMICE = re.compile(r'getenv\s*\(\s*"(LUMICE_[A-Z0-9_]+)"')
GUI_FORBIDDEN_INCLUDE = re.compile(r'#\s*include\s*"(core|config)/')
USING_NAMESPACE = re.compile(r"\busing\s+namespace\b")


def check_getenv_centralization() -> list[Violation]:
    out: list[Violation] = []
    for path in cxx_sources(SRC):
        if path == ENV_KNOBS_CPP:
            continue
        for lineno, _orig, code in code_lines(path):
            if GETENV_LUMICE.search(code):
                out.append(
                    Violation(
                        path,
                        lineno,
                        "env-getenv-centralization",
                        'std::getenv("LUMICE_...") is only allowed in '
                        "src/util/env_knobs.cpp; route this knob through util/env_knobs "
                        "(doc/env-var-policy.md).",
                    )
                )
    return out


def check_env_knob_registration() -> list[Violation]:
    out: list[Violation] = []
    if not ENV_KNOBS_CPP.exists():
        return out
    doc_text = POLICY_DOC.read_text(encoding="utf-8") if POLICY_DOC.exists() else ""
    for lineno, _orig, code in code_lines(ENV_KNOBS_CPP):
        for m in GETENV_LUMICE.finditer(code):
            name = m.group(1)
            if name not in doc_text:
                out.append(
                    Violation(
                        ENV_KNOBS_CPP,
                        lineno,
                        "env-knob-registration",
                        f"{name} is read here but not documented in "
                        "doc/env-var-policy.md; register it in the A-class table.",
                    )
                )
    return out


def check_gui_api_boundary() -> list[Violation]:
    out: list[Violation] = []
    gui = SRC / "gui"
    if not gui.exists():
        return out
    for path in cxx_sources(gui):
        for lineno, _orig, code in code_lines(path):
            m = GUI_FORBIDDEN_INCLUDE.search(code)
            if m:
                out.append(
                    Violation(
                        path,
                        lineno,
                        "gui-api-boundary",
                        f'src/gui/ must not #include "{m.group(1)}/..."; use the C '
                        "API (src/include/lumice.h) instead.",
                    )
                )
    return out


def check_no_using_namespace() -> list[Violation]:
    out: list[Violation] = []
    for path in cxx_sources(SRC):
        for lineno, _orig, code in code_lines(path):
            if USING_NAMESPACE.search(code):
                out.append(
                    Violation(
                        path,
                        lineno,
                        "no-using-namespace",
                        "`using namespace` is banned in src/ (AGENTS.md).",
                    )
                )
    return out


CHECKS = [
    check_getenv_centralization,
    check_env_knob_registration,
    check_gui_api_boundary,
    check_no_using_namespace,
]


def main() -> int:
    if "--help" in sys.argv or "-h" in sys.argv:
        print(__doc__)
        return 0
    violations: list[Violation] = []
    for check in CHECKS:
        violations.extend(check())
    if violations:
        print("Policy check FAILED:\n", file=sys.stderr)
        for v in sorted(violations, key=lambda x: (str(x.path), x.line)):
            print(v.render(), file=sys.stderr)
        print(
            f"\n{len(violations)} violation(s). See doc/env-var-policy.md and AGENTS.md.",
            file=sys.stderr,
        )
        return 1
    print("Policy check passed (env centralization, knob registration, GUI API boundary, using-namespace).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
