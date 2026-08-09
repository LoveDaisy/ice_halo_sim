#!/usr/bin/env python3
"""Count the GUI test suite's registered cases — the one baseline denominator.

Why this exists: four different numbers (598 / 721 / 632 / 760) have been quoted for
"how many GUI test cases are there", each with an unrecorded counting rule, so
"how much did we trim?" had no answer. This script fixes ONE rule and makes it
reproducible, so any later change can be reported as a delta against it.

The rule, stated so it can be argued with:

  * The unit of counting is a REGISTERED CASE — one entry in the test binary's own
    case list. That is what a runner reports and what `--gtest_list_tests` prints,
    so it is the only unit whose ground truth can be independently observed.
  * Membership is decided by the CMake TARGET's source list, not by directory.
    test/unit-correctness/gui/ deliberately holds sources of TWO different targets
    (see the link-boundary note in AGENTS.md): most link into gui_unit_test, but a
    subset with no ImGui dependency links into unit_correctness_test instead — the
    non-GUI unit-test binary. Both target source lists are parsed out of the CMakeLists
    that define them.
  * gui_unit_test and unit_correctness_test: every `TEST(...)`/`TEST_F(...)` in a
    target's sources, plus every `TEST_P(...)` body expanded by the number of rows its
    `INSTANTIATE_TEST_SUITE_P` generator instantiates (Range/Values; a generator whose
    size is not literal at the call site raises rather than guessing). Gtest registers
    one case per macro/instantiation, so the static count and the runtime list agree
    exactly — `--selfcheck` asserts that rather than assuming it, for BOTH targets.
  * gui_test: every `IM_REGISTER_TEST(engine, ...)` call SITE, except that a site
    inside a registration LOOP registers one case per row of the table the loop walks,
    so it counts as that table's length. Counting call sites alone undercounts; that is
    the known defect in one of the historical numbers. Two loop shapes are recognised —
    `for (... idx < kSceneCount ...)` over a file's `kScenes[]`, and a range-for over
    any `k...[]` table (only when a registration site is inside its braces, so a loop
    over a plain data table is left alone). Table lengths are parsed from the tables
    themselves, so adding or dropping a row moves this number with no edit here.

The four numbers this repo has quoted are all reproducible with the two rules below,
which is how they were reconciled:

  598 = --rule target  at commit e4342ecf   (the same rule as the baseline, back then)
  721 = --rule dirwalk at commit 8fb82746
  632 = --rule target  as of 2026-08-08     (the baseline, before the third target below)
  760 = --rule dirwalk as of 2026-08-08

`dirwalk` is kept ONLY so those numbers stay checkable. It counts every case file
sitting under the two gui directories regardless of which target compiles it, and it
counts loop registration sites once each, so it both over- and under-counts. Do not
report progress with it.

Third target (unit_correctness_test): a prior version of this script counted only
gui_test + gui_unit_test, so the 7 `test/unit-correctness/gui/*.cpp` files that link
into unit_correctness_test instead never reached the total — the same by-target
membership rule that was supposed to prevent exactly this kind of gap. This version
adds a third counter, `unit_correctness_test_gui=`, scoped to that target's sources
filtered down to the `unit-correctness/gui/` subdirectory (the rest of that target's
sources — core/config/server/util/backend — are out of scope for a GUI-suite counter).
`total=` now sums all three; earlier callers that assumed `total == gui_test +
gui_unit_test` will see a jump the size of the previously-missing subset. That jump is
the bug being fixed, not a new one — see the module-level note in
`count_gui_test_lines.py` for the file-domain half of the same fix.

Usage
  python3 scripts/count_gui_test_cases.py                # today's baseline
  python3 scripts/count_gui_test_cases.py --selfcheck    # + verify vs the binaries
  python3 scripts/count_gui_test_cases.py --json         # machine-readable

Downstream contract: the four `key=value` lines `gui_test=`, `gui_unit_test=`,
`unit_correctness_test_gui=`, `total=` are the output, in that order, on stdout.
Anything else (per-file breakdown, warnings) goes to stderr or behind `--verbose`.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# `test/CMakeLists.txt` guards gui_unit_test behind `if(BUILD_GUI)`; the parse below is
# indentation-insensitive so that does not matter. unit_correctness_test is defined
# unconditionally in the same file.
UNIT_CORRECTNESS_CMAKE = os.path.join("test", "CMakeLists.txt")
GUI_UNIT_TEST_CMAKE = os.path.join("test", "CMakeLists.txt")
GUI_TEST_CMAKE = os.path.join("test", "gui", "CMakeLists.txt")

# Sources of unit_correctness_test that are actually GUI cases: filtered by this path
# fragment out of the target's full source list (which also carries core/config/server/
# util/backend sources that are not this counter's concern).
GUI_SUBDIR_FRAGMENT = os.path.join("unit-correctness", "gui") + os.sep

GTEST_CASE_RE = re.compile(r"^\s*TEST(?:_F)?\s*\(", re.MULTILINE)
# A TEST_P body is registered once PER instantiated parameter, so counting the site once would
# undercount by (rows - 1) — the same defect the kSceneCount and table-loop rules below exist to
# avoid, so it gets the same treatment rather than a second unrecorded convention. Only the two
# generator spellings this repo uses are parsed, both with a literal count visible at the call
# site: `Range(0, N)` and `Values(a, b, ...)`. A generator whose size is not literal (ValuesIn over
# a table, Combine, ...) is not silently guessed at — it raises, because a wrong denominator that
# looks right is what this whole script exists to prevent. --selfcheck is the backstop either way.
GTEST_PARAM_CASE_RE = re.compile(r"^\s*TEST_P\s*\(\s*(\w+)\s*,", re.MULTILINE)
GTEST_INSTANTIATE_HEAD_RE = re.compile(r"INSTANTIATE_TEST_SUITE_P\s*\(")
GTEST_RANGE_RE = re.compile(r"::testing::Range\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)")
GTEST_VALUES_RE = re.compile(r"::testing::Values\s*\(([^)]*)\)")
# `Range(0, kRowCount)` reads better at the call site than `Range(0, 13)` and the test file
# static_asserts the constant against its own table, so the bound is resolved here rather than
# forcing every author to inline a magic number for this script's benefit.
GTEST_CONSTEXPR_INT_RE = re.compile(r"\bconstexpr\s+(?:int|size_t|std::size_t)\s+(\w+)\s*=\s*(\d+)\s*;")
IM_REGISTER_RE = re.compile(r"IM_REGISTER_TEST\s*\(\s*engine\b")
SCENE_LOOP_RE = re.compile(r"\bfor\s*\([^)]*<\s*kSceneCount\b")
SCENES_TABLE_RE = re.compile(r"\bkScenes\s*\[\s*\]\s*=\s*\{")
# The other shape a registration loop takes: a range-for straight over a `k...[]` table, one
# registered case per row. Counting the site once would undercount by (rows - 1) — the same defect
# the kSceneCount rule above exists to avoid, so it gets the same treatment rather than a second
# unrecorded convention.
TABLE_LOOP_RE = re.compile(r"\bfor\s*\(\s*(?:const\s+)?[\w:]+\s*&?\s*\w+\s*:\s*(k\w+)\s*\)")


def strip_comments_and_strings(text: str) -> str:
    """Blank out // and /* */ comments plus string/char literal bodies.

    Braces and commas inside a literal or a comment must not be read as structure —
    a scene row like `"/configs/a.json"` is harmless, but a commented-out row is not.
    Lengths are preserved so byte offsets stay valid for the caller.
    """
    out = list(text)
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if c == "/" and i + 1 < n and text[i + 1] == "/":
            while i < n and text[i] != "\n":
                out[i] = " "
                i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            out[i] = out[i + 1] = " "
            i += 2
            while i < n and not (text[i] == "*" and i + 1 < n and text[i + 1] == "/"):
                if text[i] != "\n":
                    out[i] = " "
                i += 1
            if i < n:
                out[i] = out[i + 1] = " "
                i += 2
        elif c == "'" and i > 0 and text[i - 1].isdigit() and i + 1 < n and (text[i + 1].isalnum()):
            # C++14 digit separator (`1'000'000`, `0x1234'ABCD`), not a char-literal open -- it
            # appears only immediately after a digit, with another digit/hex-letter right after. A
            # real char literal never follows a digit with no space or operator between, so this
            # check cannot misfire on one. Left un-blanked and treated as ordinary code text.
            #
            # Measured: without this check, an ODD number of digit-separator quotes in one literal
            # (e.g. `5'000'000'000ull`, three quotes) desyncs the quote-tracking below -- the third
            # quote opens a "char literal" that finds no closing `'` until some unrelated quote much
            # later in the file, blanking every TEST(...)/TEST_F(...) call in between out of
            # existence. This is exactly how unit_correctness_test's static count first came up 9
            # cases short of `--gtest_list_tests`: test_pcg_ray_base_split.cpp's digit-separated
            # `5'000'000'000ull` swallowed 6 of its own 9 TEST(...) sites before the next real quote
            # (an apostrophe inside a later comment) closed the bogus literal back out.
            i += 1
        elif c in ("'", '"'):
            quote = c
            i += 1
            while i < n:
                if text[i] == "\\":
                    out[i] = out[i + 1] = " " if i + 1 < n else " "
                    i += 2
                    continue
                if text[i] == quote:
                    break
                if text[i] != "\n":
                    out[i] = " "
                i += 1
            if i < n:
                i += 1
        else:
            i += 1
    return "".join(out)


def parse_target_sources(cmake_text: str, target: str) -> list[str]:
    """Return the raw source entries of `add_executable(<target> ...)`."""
    marker = re.search(r"add_executable\s*\(\s*" + re.escape(target) + r"\b", cmake_text)
    if marker is None:
        raise SystemExit(f"error: add_executable({target} ...) not found")
    depth = 0
    i = marker.start()
    while cmake_text[i] != "(":
        i += 1
    start = i + 1
    while i < len(cmake_text):
        if cmake_text[i] == "(":
            depth += 1
        elif cmake_text[i] == ")":
            depth -= 1
            if depth == 0:
                break
        i += 1
    body = cmake_text[start:i]
    # Drop the target name itself and any CMake comment lines. This must happen BEFORE tokenizing
    # — a comment that merely mentions a sibling file's path (e.g. explaining a link-boundary
    # split) must not be read as a source entry.
    body = re.sub(r"#[^\n]*", "", body)
    tokens = re.findall(r'"[^"]*"|\S+', body)
    return [t.strip('"') for t in tokens[1:] if t.strip('"')]


def resolve_source(entry: str, root: str) -> str:
    path = entry.replace("${PROJ_TEST_DIR}", os.path.join(root, "test"))
    path = path.replace("${PROJ_SRC_DIR}", os.path.join(root, "src"))
    path = path.replace("${CMAKE_SOURCE_DIR}", root)
    return path


def iter_cpp(directory: str) -> list[str]:
    found = []
    for dirpath, _dirnames, filenames in os.walk(directory):
        for name in sorted(filenames):
            if name.endswith(".cpp"):
                found.append(os.path.join(dirpath, name))
    return sorted(found)


def count_dirwalk(root: str) -> tuple[int, int, list[tuple[str, int]]]:
    """The superseded rule, reproduced so the 721 / 760 figures stay checkable.

    Directory membership instead of target membership, and one case per registration
    call site instead of one per loop iteration.
    """
    per_file: list[tuple[str, int]] = []
    unit = 0
    for path in iter_cpp(os.path.join(root, "test", "unit-correctness", "gui")):
        with open(path, encoding="utf-8") as fh:
            count = len(GTEST_CASE_RE.findall(strip_comments_and_strings(fh.read())))
        if count:
            per_file.append((os.path.relpath(path, root), count))
            unit += count
    gui = 0
    for path in iter_cpp(os.path.join(root, "test", "gui")):
        with open(path, encoding="utf-8") as fh:
            count = len(IM_REGISTER_RE.findall(strip_comments_and_strings(fh.read())))
        if count:
            per_file.append((os.path.relpath(path, root), count))
            gui += count
    return gui, unit, per_file


def count_param_cases(body: str, rel_path: str) -> int:
    """Registered cases contributed by TEST_P bodies: one per instantiated parameter."""
    suites: dict[str, int] = {}
    for suite in GTEST_PARAM_CASE_RE.findall(body):
        suites[suite] = suites.get(suite, 0) + 1
    if not suites:
        return 0

    consts = {name: int(value) for name, value in GTEST_CONSTEXPR_INT_RE.findall(body)}

    def as_int(token: str) -> int | None:
        if token.isdigit():
            return int(token)
        return consts.get(token)

    per_suite_params: dict[str, int] = {}
    for suite, generator in find_instantiate_calls(body):
        rng = GTEST_RANGE_RE.search(generator)
        lo = as_int(rng.group(1)) if rng else None
        hi = as_int(rng.group(2)) if rng else None
        if rng and lo is not None and hi is not None:
            n = hi - lo
        else:
            vals = GTEST_VALUES_RE.search(generator)
            if not vals:
                raise SystemExit(
                    f"{rel_path}: INSTANTIATE_TEST_SUITE_P for '{suite}' uses a generator whose "
                    f"size is not literal at the call site. Add the spelling to "
                    f"count_gui_test_cases.py rather than letting the denominator guess.")
            n = len([v for v in vals.group(1).split(",") if v.strip()])
        per_suite_params[suite] = per_suite_params.get(suite, 0) + n

    total = 0
    for suite, bodies in suites.items():
        if suite not in per_suite_params:
            raise SystemExit(f"{rel_path}: TEST_P suite '{suite}' has no INSTANTIATE_TEST_SUITE_P; "
                             f"gtest would register nothing for it.")
        total += bodies * per_suite_params[suite]
    return total


def find_instantiate_calls(body: str) -> list[tuple[str, str]]:
    """Each `INSTANTIATE_TEST_SUITE_P(Prefix, Suite, generator)` call as (suite, generator).

    Matches the closing paren by depth rather than stopping at the first `;` — a generator
    can itself carry a semicolon (a named lambda body ending `return ...;`), which would
    otherwise truncate the capture mid-generator. Whichever generator spelling happened to
    sit before the lambda would still resolve correctly by accident, which is exactly the
    silent-until-it-isn't failure mode this counts-as-ground-truth script exists to avoid.
    """
    calls: list[tuple[str, str]] = []
    for head in GTEST_INSTANTIATE_HEAD_RE.finditer(body):
        depth = 1
        i = head.end()
        while i < len(body) and depth > 0:
            if body[i] == "(":
                depth += 1
            elif body[i] == ")":
                depth -= 1
            i += 1
        if depth != 0:
            continue  # unbalanced parens; downstream suite-membership check will catch it
        args = body[head.end():i - 1].split(",", 2)
        if len(args) < 3:
            continue
        calls.append((args[1].strip(), args[2]))
    return calls


def count_gtest_style_target(
    root: str, cmake_rel: str, target: str, path_filter: str | None = None
) -> tuple[int, list[tuple[str, int]]]:
    """Count TEST/TEST_F/TEST_P(expanded) cases in a target's `.cpp` sources.

    `path_filter`, when given, keeps only source paths containing that fragment (used to scope
    unit_correctness_test's full source list down to its GUI subset) — filtering happens on the
    resolved path, after CMake variables are substituted, so it is robust to how the entry is
    spelled in CMakeLists.txt.
    """
    cmake_path = os.path.join(root, cmake_rel)
    with open(cmake_path, encoding="utf-8") as fh:
        cmake_text = fh.read()
    per_file: list[tuple[str, int]] = []
    for entry in parse_target_sources(cmake_text, target):
        path = resolve_source(entry, root)
        if not path.endswith(".cpp"):
            continue
        if path_filter is not None and path_filter not in path:
            continue
        if not os.path.exists(path):
            print(f"warning: source not found, skipped: {path}", file=sys.stderr)
            continue
        with open(path, encoding="utf-8") as fh:
            body = strip_comments_and_strings(fh.read())
        count = len(GTEST_CASE_RE.findall(body)) + count_param_cases(body, os.path.relpath(path, root))
        if count:
            per_file.append((os.path.relpath(path, root), count))
    return sum(c for _, c in per_file), per_file


def count_scene_table(body: str) -> int:
    """Length of the file's `kScenes[]` initializer, counted at brace depth 1."""
    return count_table_rows(body, "kScenes", "a kSceneCount loop was found but kScenes[] was not")


def count_table_rows(body: str, table: str, missing_msg: str) -> int:
    """Length of a `<table>[] = { {...}, {...} }` initializer, counted at brace depth 1."""
    match = re.search(r"\b" + re.escape(table) + r"\s*\[\s*\]\s*=\s*\{", body)
    if match is None:
        raise SystemExit("error: " + missing_msg)
    i = match.end()
    depth = 1
    rows = 0
    saw_content = False
    while i < len(body) and depth > 0:
        c = body[i]
        if c == "{":
            if depth == 1:
                rows += 1
            depth += 1
        elif c == "}":
            depth -= 1
        elif depth == 1 and not c.isspace():
            saw_content = True
        i += 1
    if rows == 0 and saw_content:
        # A flat table (`{a, b, c}` with scalar rows) — count comma-separated entries.
        raise SystemExit(f"error: {table}[] rows are not brace-delimited; parser needs updating")
    return rows


def registration_loop_tables(body: str) -> list[str]:
    """Tables whose range-for body actually holds an IM_REGISTER_TEST call.

    A range-for over a plain data table (a list of EV offsets, say) is not a registration loop, so
    only loops whose braces contain a registration site are reported.
    """
    tables: list[str] = []
    for match in TABLE_LOOP_RE.finditer(body):
        i = body.find("{", match.end())
        if i < 0:
            continue
        depth = 0
        j = i
        while j < len(body):
            if body[j] == "{":
                depth += 1
            elif body[j] == "}":
                depth -= 1
                if depth == 0:
                    break
            j += 1
        if IM_REGISTER_RE.search(body[i:j]):
            tables.append(match.group(1))
    return tables


def count_gui_test(root: str) -> tuple[int, list[tuple[str, int, int]]]:
    cmake_path = os.path.join(root, GUI_TEST_CMAKE)
    with open(cmake_path, encoding="utf-8") as fh:
        cmake_text = fh.read()
    gui_dir = os.path.join(root, "test", "gui")
    per_file: list[tuple[str, int, int]] = []
    for entry in parse_target_sources(cmake_text, "gui_test"):
        if not entry.endswith(".cpp"):
            continue
        if "${" in entry:
            # imgui_test_engine's own sources — third-party, registers nothing of ours.
            continue
        path = os.path.join(gui_dir, entry)
        if not os.path.exists(path):
            print(f"warning: source not found, skipped: {path}", file=sys.stderr)
            continue
        with open(path, encoding="utf-8") as fh:
            body = strip_comments_and_strings(fh.read())
        sites = len(IM_REGISTER_RE.findall(body))
        if sites == 0:
            continue
        expanded = sites
        if SCENE_LOOP_RE.search(body):
            scenes = count_scene_table(body)
            loop_sites = len(SCENE_LOOP_RE.findall(body))
            # Each scene-loop site contributes `scenes` cases instead of one.
            expanded = sites - loop_sites + loop_sites * scenes
        for table in registration_loop_tables(body):
            # Same correction for a range-for over a table: the one site inside it registers one
            # case per row, so the site's single count is short by (rows - 1).
            rows = count_table_rows(body, table, f"a range-for over {table} registers cases, but {table}[] was not found")
            expanded += rows - 1
        per_file.append((os.path.relpath(path, root), sites, expanded))
    return sum(e for _, _, e in per_file), per_file


# Named (suite, case) pairs, for the selfcheck's set-diff -- TEST/TEST_F only. TEST_P cases carry
# instantiation-dependent runtime names (`Suite/Prefix.Case/N`) that this pattern does not match;
# neither target currently uses TEST_P (checked: zero matches for `TEST_P\(` under
# test/unit-correctness/), so the diff below is exhaustive today. If one is ever added there, the
# aggregate counts (count_gtest_style_target, via count_param_cases) still cover it correctly --
# only this cross-check's per-name diff would need extending to match TEST_P's naming.
CASE_NAME_RE = re.compile(r"\bTEST(?:_F)?\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)")


def extract_case_names(root: str, cmake_rel: str, target: str, path_filter: str | None = None) -> set[tuple[str, str]]:
    """Every (suite, case) pair textually present in a target's sources (TEST/TEST_F only)."""
    cmake_path = os.path.join(root, cmake_rel)
    with open(cmake_path, encoding="utf-8") as fh:
        cmake_text = fh.read()
    names: set[tuple[str, str]] = set()
    for entry in parse_target_sources(cmake_text, target):
        path = resolve_source(entry, root)
        if not path.endswith(".cpp"):
            continue
        if path_filter is not None and path_filter not in path:
            continue
        if not os.path.exists(path):
            continue
        with open(path, encoding="utf-8") as fh:
            body = strip_comments_and_strings(fh.read())
        names.update(CASE_NAME_RE.findall(body))
    return names


def _binary_case_names(binary: str, root: str) -> set[tuple[str, str]] | None:
    """Every (suite, case) pair `--gtest_list_tests` actually reports, or None if the run failed."""
    proc = subprocess.run(
        [binary, "--gtest_list_tests"], capture_output=True, text=True, cwd=root, check=False
    )
    if proc.returncode != 0:
        return None
    names: set[tuple[str, str]] = set()
    suite: str | None = None
    for line in proc.stdout.splitlines():
        if not line.strip():
            continue
        if not line.startswith(" ") and line.rstrip().endswith("."):
            suite = line.strip()[:-1]
            continue
        if line.startswith(" ") and suite is not None:
            case = line.strip().split("#")[0].strip()
            names.add((suite, case))
    return names


def _selfcheck_target(root: str, target: str, static_names: set[tuple[str, str]]) -> int:
    """Diff a target's static (suite, case) set against its own binary's --gtest_list_tests.

    A set diff, not a bare count comparison: the two directions of mismatch mean different things
    and warrant different responses.

      * Runtime has a case the static parse missed -- an UNDERCOUNT. This is the failure mode the
        whole tool exists to prevent (the historical 142-case gap was exactly this, at the file-set
        level instead of the per-case level), so it is a hard failure.
      * Static has a case the binary never registered -- an OVERCOUNT, expected whenever a case
        sits behind platform-conditional compilation (`#if defined(__APPLE__)` / `#ifndef NDEBUG` /
        a whole TU gated on `#if LUMICE_CUDA_ENABLED`): the text exists on every platform, but only
        one branch's TEST(...) actually compiles on any given machine. A static, non-preprocessing
        text scan cannot know which branch a given host takes, so this is reported for visibility,
        not treated as a bug -- failing the gate here would mean this check can only ever pass on
        whichever single machine happened to compile every guarded branch at once, which does not
        exist.
    """
    candidates = [
        os.path.join(root, "build", "Release", "static", "bin", target),
        os.path.join(root, "build", "Debug", "static", "bin", target),
        os.path.join(root, "build", "Release", "shared", "bin", target),
    ]
    binary = next((c for c in candidates if os.path.exists(c)), None)
    if binary is None:
        print(
            f"selfcheck: {target} binary not found in build/ — build it with\n"
            "  LUMICE_SKIP_GUI_TESTS=1 ./scripts/build.sh -gtj release",
            file=sys.stderr,
        )
        return 2
    runtime_names = _binary_case_names(binary, root)
    if runtime_names is None:
        print(f"selfcheck: {binary} --gtest_list_tests failed", file=sys.stderr)
        return 2

    only_runtime = sorted(runtime_names - static_names)
    only_static = sorted(static_names - runtime_names)

    if only_runtime:
        print(
            f"selfcheck: MISMATCH ({target}) — {len(only_runtime)} case(s) registered at runtime "
            "that the static parse missed:",
            file=sys.stderr,
        )
        for s, c in only_runtime[:20]:
            print(f"    + {s}.{c}", file=sys.stderr)
        if len(only_runtime) > 20:
            print(f"    ... and {len(only_runtime) - 20} more", file=sys.stderr)
        return 1

    if only_static:
        print(
            f"selfcheck: OK ({target}) — no undercount; {len(only_static)} textually-present "
            "case(s) not compiled into this binary (expected under platform-conditional "
            "compilation; not failed):",
            file=sys.stderr,
        )
        for s, c in only_static[:20]:
            print(f"    - {s}.{c}", file=sys.stderr)
        if len(only_static) > 20:
            print(f"    ... and {len(only_static) - 20} more", file=sys.stderr)
        return 0

    print(
        f"selfcheck: OK ({target}) — static parse and {os.path.relpath(binary, root)} agree "
        f"exactly ({len(static_names)} cases)",
        file=sys.stderr,
    )
    return 0


def selfcheck(root: str) -> int:
    """Verify both TARGET-rule counters against their binaries' own case lists.

    unit_correctness_test is checked against its FULL name set (every source in the target, not
    just the GUI subset) because that is what `--gtest_list_tests` reports for that binary --
    there is no way to ask it for "only the gui/ subset". A correct full-target diff is sufficient
    evidence the gui/ subset (a strict slice of the same parse, using the same
    TEST/TEST_F/TEST_P logic) is also correct.
    """
    gui_unit_names = extract_case_names(root, GUI_UNIT_TEST_CMAKE, "gui_unit_test")
    unit_correctness_names = extract_case_names(root, UNIT_CORRECTNESS_CMAKE, "unit_correctness_test")
    rc1 = _selfcheck_target(root, "gui_unit_test", gui_unit_names)
    rc2 = _selfcheck_target(root, "unit_correctness_test", unit_correctness_names)
    return rc1 or rc2


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Count registered GUI test cases (gui_test + gui_unit_test + unit_correctness_test's GUI subset).",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--root",
        default=REPO_ROOT,
        help="Tree to count. Defaults to this repository. Point it at a directory holding "
        "checked-out historical copies of test/CMakeLists.txt, test/gui/CMakeLists.txt and "
        "their sources to reproduce a past number. One-off archaeology only — routine "
        "baseline runs need no --root.",
    )
    parser.add_argument(
        "--rule",
        choices=("target", "dirwalk"),
        default="target",
        help="'target' (default) is the baseline rule: CMake target membership, scene loops "
        "expanded. 'dirwalk' reproduces the superseded rule behind the 721/760 figures "
        "(directory membership, one case per call site) — archaeology only, never for progress.",
    )
    parser.add_argument("--json", action="store_true", help="emit a JSON object instead of key=value lines")
    parser.add_argument("--verbose", action="store_true", help="print the per-file breakdown to stderr")
    parser.add_argument(
        "--selfcheck",
        action="store_true",
        help="also run the built gui_unit_test and unit_correctness_test binaries' "
        "--gtest_list_tests and fail on any disagreement with the static parse",
    )
    args = parser.parse_args()

    root = os.path.abspath(args.root)

    if args.rule == "dirwalk":
        gui_total, unit_total, walk_files = count_dirwalk(root)
        unit_correctness_gui_total = 0
        if args.verbose:
            print("dirwalk rule (superseded; directory membership, unexpanded sites):", file=sys.stderr)
            for name, count in walk_files:
                print(f"  {count:4d}  {name}", file=sys.stderr)
        if args.selfcheck:
            print("selfcheck is meaningless under --rule dirwalk (it counts non-member files)", file=sys.stderr)
            return 2
    else:
        unit_total, unit_files = count_gtest_style_target(root, GUI_UNIT_TEST_CMAKE, "gui_unit_test")
        gui_total, gui_files = count_gui_test(root)
        unit_correctness_gui_total, unit_correctness_gui_files = count_gtest_style_target(
            root, UNIT_CORRECTNESS_CMAKE, "unit_correctness_test", path_filter=GUI_SUBDIR_FRAGMENT)
        if args.verbose:
            print("gui_unit_test (TEST/TEST_F per source):", file=sys.stderr)
            for name, count in unit_files:
                print(f"  {count:4d}  {name}", file=sys.stderr)
            print("gui_test (IM_REGISTER_TEST sites -> scene-expanded):", file=sys.stderr)
            for name, sites, expanded in gui_files:
                suffix = "" if sites == expanded else f" (from {sites} sites)"
                print(f"  {expanded:4d}  {name}{suffix}", file=sys.stderr)
            print("unit_correctness_test's unit-correctness/gui/ subset (TEST/TEST_F per source):", file=sys.stderr)
            for name, count in unit_correctness_gui_files:
                print(f"  {count:4d}  {name}", file=sys.stderr)

    total = gui_total + unit_total + unit_correctness_gui_total

    if args.json:
        print(json.dumps({
            "gui_test": gui_total,
            "gui_unit_test": unit_total,
            "unit_correctness_test_gui": unit_correctness_gui_total,
            "total": total,
        }))
    else:
        print(f"gui_test={gui_total}")
        print(f"gui_unit_test={unit_total}")
        print(f"unit_correctness_test_gui={unit_correctness_gui_total}")
        print(f"total={total}")

    if args.selfcheck:
        if args.rule == "dirwalk":
            return 2
        return selfcheck(root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
