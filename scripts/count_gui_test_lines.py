#!/usr/bin/env python3
"""Count non-comment, non-blank code lines in the GUI test suite.

Why this exists: a GUI test suite rebuild effort needs a single, self-certifying ruler for
"how big is the suite" before any wave can write an AC against it. A prior, unmerged attempt at
the same idea used the same state-machine design but never handled raw string literals as a
distinct parser state -- a documented latent gap. This script keeps the design but adds that
state, because a corpus with JSON payloads embedded as `R"(...)"` literals is exactly where `//`
or `/*` inside string content would otherwise be misread as a real comment marker.

Domain is defined by FILE PATH, not by CMake target membership: every tracked `.cpp`/`.hpp`
under `test/gui/` and `test/unit-correctness/gui/`. This is a deliberate choice, not an
oversight -- GUI test cases are split across three CMake targets (`gui_test`, `gui_unit_test`,
`unit_correctness_test`; see `test/CMakeLists.txt`), and a target-scoped denominator has twice
silently dropped files that live in `test/unit-correctness/gui/` but link into
`unit_correctness_test` rather than `gui_unit_test`. Counting by path sidesteps that failure
mode entirely: it does not matter which target compiles a file, only where it sits.

Usage:
    python3 scripts/count_gui_test_lines.py                  # total + per-directory subtotal
    python3 scripts/count_gui_test_lines.py --per-file        # + one line per file
    python3 scripts/count_gui_test_lines.py --json            # machine-readable
    python3 scripts/count_gui_test_lines.py --selfcheck        # run the built-in known-positive
                                                                 # sample suite instead of counting
    python3 scripts/count_gui_test_lines.py FILE [FILE ...]    # explicit files, domain ignored
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The two directories that hold the GUI test suite's sources, by filesystem location. Kept as a
# literal pair (matching the equivalent list in count_gui_test_cases.py) rather than derived from
# CMake, because the domain is explicitly NOT "whatever some target's source list says" -- see the
# module docstring.
DOMAIN_DIRS = [
    os.path.join("test", "gui"),
    os.path.join("test", "unit-correctness", "gui"),
]
DOMAIN_EXTS = (".cpp", ".hpp")

# Parser states.
CODE, LINE_COMMENT, BLOCK_COMMENT, STRING, CHAR, RAW_STRING = range(6)

# Raw string delimiters are capped at 16 characters by the standard; this is just a runaway guard,
# not an enforced correctness bound -- a delimiter this long or longer simply fails to parse as a
# raw string and falls back to treating `R` as an ordinary code character (see strip_comments).
_MAX_RAW_DELIM_LEN = 16
_RAW_DELIM_STOP_CHARS = set("()\\\t\n\v\f\r ")


def _is_ident_char(c: str) -> bool:
    return c.isalnum() or c == "_"


def strip_comments(src: str) -> str:
    """Blank out comments, preserving newlines so line numbering is unchanged.

    Six states: plain code, `//` line comment, `/* */` block comment, `"..."` string literal,
    `'...'` char literal, and `R"delim(...)delim"` raw string literal. String, char and raw-string
    content is walked through intact -- a `//` or `/*` inside any of them is literal text, not a
    comment marker, and must survive into the code-line count untouched.

    Known limitation: only the bare `R"` spelling is recognized as a raw string opener, not the
    prefixed forms (`u8R"`, `uR"`, `UR"`, `LR"`). As of this writing the scanned GUI test
    directories contain no prefixed raw string literals (checked via grep), so today's totals are
    unaffected -- this is a latent gap, not a currently-tripped one, exactly like the raw-string
    gap this script exists to close for its own predecessor. If a prefixed raw string is ever
    added, that file's count should be checked by hand against `--per-file`.
    """
    out: list[str] = []
    i = 0
    n = len(src)
    state = CODE
    raw_delim = ""
    prev_char = ""
    while i < n:
        c = src[i]
        d = src[i + 1] if i + 1 < n else ""
        if state == CODE:
            if c == "R" and d == '"' and not _is_ident_char(prev_char):
                j = i + 2
                delim_chars: list[str] = []
                while (
                    j < n
                    and src[j] not in _RAW_DELIM_STOP_CHARS
                    and len(delim_chars) < _MAX_RAW_DELIM_LEN
                ):
                    delim_chars.append(src[j])
                    j += 1
                if j < n and src[j] == "(":
                    raw_delim = "".join(delim_chars)
                    out.append(src[i : j + 1])
                    i = j + 1
                    state = RAW_STRING
                    prev_char = "("
                    continue
                out.append(c)
                prev_char = c
                i += 1
                continue
            if c == "/" and d == "/":
                state = LINE_COMMENT
                i += 2
                prev_char = ""
                continue
            if c == "/" and d == "*":
                state = BLOCK_COMMENT
                i += 2
                prev_char = ""
                continue
            if c == '"':
                state = STRING
                out.append(c)
                prev_char = c
                i += 1
                continue
            if c == "'":
                # C++14 digit separator (`1'000'000`, `0x1234'ABCD`), not a char-literal open --
                # it appears only immediately after a digit, with another digit/hex-letter right
                # after. A real char literal never follows a digit with no space or operator
                # between, so this check cannot misfire on one.
                if prev_char.isdigit() and _is_ident_char(d):
                    out.append(c)
                    prev_char = c
                    i += 1
                    continue
                state = CHAR
                out.append(c)
                prev_char = c
                i += 1
                continue
            out.append(c)
            prev_char = c
            i += 1
        elif state == LINE_COMMENT:
            if c == "\n":
                state = CODE
                out.append(c)
                prev_char = ""
            i += 1
        elif state == BLOCK_COMMENT:
            if c == "*" and d == "/":
                state = CODE
                i += 2
                prev_char = ""
                continue
            if c == "\n":
                out.append(c)
            i += 1
        elif state == STRING:
            out.append(c)
            if c == "\\" and d:
                out.append(d)
                prev_char = d
                i += 2
                continue
            prev_char = c
            if c == '"':
                state = CODE
            i += 1
        elif state == CHAR:
            out.append(c)
            if c == "\\" and d:
                out.append(d)
                prev_char = d
                i += 2
                continue
            prev_char = c
            if c == "'":
                state = CODE
            i += 1
        else:  # RAW_STRING
            closer = ")" + raw_delim + '"'
            if src.startswith(closer, i):
                out.append(closer)
                i += len(closer)
                state = CODE
                prev_char = '"'
                raw_delim = ""
                continue
            out.append(c)
            prev_char = c
            i += 1
    return "".join(out)


def count_text(src: str) -> int:
    return sum(1 for line in strip_comments(src).split("\n") if line.strip())


def count_file(path: str) -> int:
    with open(path, "r", encoding="utf-8") as fh:
        return count_text(fh.read())


def default_scope() -> list[str]:
    """Every tracked file under the two GUI test directories, filtered to DOMAIN_EXTS.

    `git ls-files`, not `os.walk`: a stale build artifact or an untracked scratch copy left in the
    tree must not inflate a number that gets quoted as the suite's size.
    """
    try:
        out = subprocess.run(
            ["git", "ls-files", "--"] + DOMAIN_DIRS,
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        raise RuntimeError(
            "count_gui_test_lines.py: `git ls-files` failed, refusing to fall back to a "
            f"directory walk: {exc}"
        ) from exc
    paths = [p for p in out.split("\n") if p.endswith(DOMAIN_EXTS)]
    return sorted(paths)


def top_dir(path: str) -> str:
    """The domain directory (test/gui or test/unit-correctness/gui) a path falls under."""
    for d in DOMAIN_DIRS:
        if path == d or path.startswith(d + os.sep):
            return d
    return os.path.dirname(path)


# ----------------------------------------------------------------------------------------------
# --selfcheck: known-positive samples, each asserted against a hand-counted expected total.

def _selfcheck_samples() -> list[tuple[str, str, int]]:
    samples: list[tuple[str, str, int]] = []

    samples.append((
        "pure-line-comment",
        "// this is a comment\nint x = 1;\n",
        1,
    ))
    samples.append((
        "trailing-line-comment",
        "int x = 1; // trailing\n",
        1,
    ))
    samples.append((
        "block-comment-spanning-lines",
        "int a = 1;\n/* comment\n   spans\n   lines */\nint b = 2;\n",
        2,
    ))
    samples.append((
        "block-comment-closed-same-line",
        "int a = /* mid */ 2;\nint b = 3;\n",
        2,
    ))
    samples.append((
        "string-literal-contains-double-slash",
        'const char* url = "http://example.com";\n',
        1,
    ))
    samples.append((
        # Decisive: a naive stripper that does not track string state would open a (never-closing)
        # block comment here and blank every line to end-of-sample -- this is the failure mode the
        # predecessor script's raw-string gap generalizes from.
        "string-literal-contains-slash-star",
        'const char* trick = "value /* not a comment";\nint line_two = 1;\nint line_three = 2;\n',
        3,
    ))
    samples.append((
        # Decisive for the raw-string state specifically: every physical line inside the raw
        # string is real source text and must count, even though it contains both `//` and `/*`.
        "raw-string-contains-comment-markers",
        'const char* json = R"(\n'
        "line with // not a comment\n"
        "line with /* also not a comment\n"
        'end)";\n'
        "int after_raw = 4;\n",
        5,
    ))
    samples.append((
        "char-literal-slash",
        "char c = '/';\nint x = 5;\n",
        2,
    ))
    samples.append((
        # Without the digit-separator guard, an ODD count of separator quotes (3, here) desyncs
        # the CHAR-state parser: the third `'` opens a spurious char literal that finds no closing
        # `'` on this line and carries into the next. CHAR state echoes content verbatim (it does
        # not blank anything -- see strip_comments), so the danger is not lost code but a
        # suppressed comment: once inside the bogus span, a genuine `//` on line 2 is no longer
        # recognized as a comment marker and survives into the code-line count. The comment line
        # below is what makes guard-on and guard-off diverge -- a payload with only code lines
        # after the literal cannot tell the two apart, because CHAR's verbatim echo keeps every
        # code line intact either way (measured: an earlier version of this sample had only code
        # lines here and passed identically with the guard short-circuited).
        "digit-separator-odd-segment-count",
        "size_t rb = 5'000'000'000ull;\n// must not be counted\nint x = 1;\n",
        2,
    ))
    samples.append((
        "digit-separator-even-segment-count",
        "constexpr int kN = 3'000'000;\nint after_literal = 7;\n",
        2,
    ))
    samples.append((
        "blank-lines-and-brace-only-lines",
        "int Foo() {\n}\n\nint Bar() {\n    return 1;\n}\n",
        5,
    ))
    return samples


def selfcheck() -> int:
    failures = 0
    for name, src, expected in _selfcheck_samples():
        actual = count_text(src)
        status = "ok" if actual == expected else "FAIL"
        if actual != expected:
            failures += 1
        print(f"  [{status}] {name}: expected {expected}, got {actual}", file=sys.stderr)
    total = len(_selfcheck_samples())
    if failures:
        print(f"selfcheck: {failures}/{total} sample(s) FAILED", file=sys.stderr)
        return 1
    print(f"selfcheck: OK — {total}/{total} samples passed", file=sys.stderr)
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("paths", nargs="*", help="files to count; default = the GUI test suite domain")
    ap.add_argument("--per-file", action="store_true", help="also print a per-file breakdown")
    ap.add_argument("--json", action="store_true", help="emit JSON")
    ap.add_argument(
        "--selfcheck",
        action="store_true",
        help="run the built-in known-positive sample suite and exit (ignores other flags/paths)",
    )
    args = ap.parse_args()

    if args.selfcheck:
        return selfcheck()

    paths = args.paths if args.paths else default_scope()

    per_file: dict[str, int] = {}
    missing: list[str] = []
    for p in paths:
        full = p if os.path.isabs(p) else os.path.join(REPO_ROOT, p)
        if not os.path.isfile(full):
            missing.append(p)
            continue
        per_file[p] = count_file(full)
    if missing:
        for p in missing:
            print(f"error: no such file: {p}", file=sys.stderr)
        return 1

    per_dir: dict[str, int] = {}
    for p, n in per_file.items():
        d = top_dir(p)
        per_dir[d] = per_dir.get(d, 0) + n

    total = sum(per_file.values())

    if args.json:
        result = {"code_lines": total, "files": len(per_file), "per_directory": per_dir}
        if args.per_file:
            result["per_file"] = per_file
        print(json.dumps(result, indent=1))
        return 0

    if args.per_file:
        width = max((len(p) for p in per_file), default=0)
        for p, n in sorted(per_file.items()):
            print(f"{p.ljust(width)}  {n}")
        print("-" * (width + 2 + len(str(total))))
    for d, n in sorted(per_dir.items()):
        print(f"{d}: {n}")
    print(f"TOTAL: {total} ({len(per_file)} files)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
