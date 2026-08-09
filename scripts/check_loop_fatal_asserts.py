#!/usr/bin/env python3
"""Diff-scoped check: a `for` loop body must not contain a fatal assert directly.

The same defect shape — a table-driven `for` loop whose body calls a fatal assert
(`ASSERT_*` from gtest, or this repo's `IM_CHECK*` from ImGuiTestEngine) directly — has
repeatedly slipped past hand review of this suite's loop bodies: a reviewer skimming a large
table-driven test file tends to check the first row's assertion and move on, and each pass
misses a different remaining instance. Manual per-file review is not a repeatable detector;
this script is. Per this repo's own precedent — "the checker is the rule, not an approximation
of it" (AGENTS.md, said originally of check_new_refs.py / check_new_gui_tests.py, adopted here
by the same reasoning) — 0 hits from this script is the evidence a change is clean, not a
supplement to a hand-written list.

The defect
----------
A fatal assert macro expands to `return;` (or `return value;`) in the ENCLOSING FUNCTION — not a
`break` out of the current block. So a fatal assert anywhere in a `for` loop's body — including
nested inside an `if`/`while`/`switch`/nested `for` block, all of which are transparent to a
`return` — aborts the rest of that loop's iterations, and the function that called it, the moment
one iteration's early rows disagree. For a loop walking an independent-row table (the shape this
codebase's GUI test suite is full of: "for each of N unrelated fields/slots/cases, assert
something"), that means the first row to fail hides every row after it — a genuine correctness gap
in the test, not just a style complaint (see AGENTS.md's `check_new_gui_tests.py` docstring for the
sibling case of a mechanically-enforced test-authoring rule in this codebase).

The one construct that DOES stop a `return` from propagating out of the loop is a lambda: a fatal
assert called from inside a lambda body returns only that lambda, not the loop. This is why a
per-row callback (`row.check(...)`, `case.expect(...)`) is the standard escape hatch already used
throughout this suite — its body lives wherever the callback is *defined*, and if that definition
sits outside the loop (the overwhelmingly common shape: a `Row`/`Case` struct's lambda member is
built once, before the loop starts), the assert inside it never enters this scan's field of view in
the first place. This scanner special-cases only the narrower, harder case: a lambda written
literally *inside* the loop body (e.g. passed inline to `std::any_of`).

What is flagged
----------------
Every `for (...)  { ... }` (range-for or indexed, brace body required — see limitations) under the
scan root, where the body contains an `ASSERT_*` / `IM_CHECK*` call not shielded by an intervening
lambda. "Not shielded": walking outward from the assert through its enclosing brace scopes, the
nearest `for`-body or lambda-body frame (skipping transparent `if`/`while`/`switch`/plain-block
frames) is a `for`, not a `lambda`.  A doubly-nested `for A { for B { ASSERT } }` is reported
against the innermost loop B only — B's own remaining rows are the most direct casualty, and B's
failure already implies A's is short-circuited too; reporting both would double-count one defect.

What is NOT flagged, and why that is accepted
-----------------------------------------------
  - A fatal assert inside a lambda, wherever that lambda is written (inline in the loop, or built
    earlier and invoked via `row.check(...)`) — that is the sanctioned per-row-callback shape this
    codebase's own fix commits (b044aa14, cf12fe9b) converge on when a callback IS wanted; when it
    is not, the accepted fix is `if (...) { ADD_FAILURE() << ...; continue; }`, which this scanner
    also does not flag (`ADD_FAILURE` is non-fatal by construction).
  - A `for` loop with no brace body (`for (...) Stmt;`) — this text-only scanner requires a `{` to
    delimit the body it walks; a single-statement for-loop is rare in this codebase's formatted
    style (clang-format keeps braces on any multi-line body) and is a documented gap, not a claimed
    guarantee.
  - A fatal assert in a `for` loop's own header (init/condition/increment) — those run once per
    iteration outside any "row" semantics and are not the shape this rule targets.
  - `T x[N]{...}` direct-list-initialization is textually indistinguishable from a lambda's
    `[capture]{ body }` to this regex-only scanner (see `LAMBDA_RE`'s docstring) and is treated as
    a lambda scope; the risk is a false NEGATIVE (an assert inside such an initializer would be
    hidden), and no such initializer contains an assert call anywhere in this tree today.

Scope
-----
Default mode is diff-scoped, the same shape as check_new_refs.py and
check_new_gui_tests.py: only files under `test/` that the diff actually touches are read, and
only a hit whose assert line is one the diff added is reported. A whole-tree gate could not
start green here — a full sweep of this tree finds hits outside this rule's own domain — and
scoping to added lines means an unrelated edit elsewhere in an already-flagged file is never
blocked by code nobody touched. `--root PATH` bypasses the diff entirely and scans a whole
subtree instead, for a manual or diagnostic full sweep.

Usage
-----
  python3 scripts/check_loop_fatal_asserts.py               # diff-scoped: staged (pre-commit default)
  python3 scripts/check_loop_fatal_asserts.py --staged      # same, explicit
  python3 scripts/check_loop_fatal_asserts.py --range A..B  # CI: PR vs merge-base
  python3 scripts/check_loop_fatal_asserts.py --root PATH   # whole-subtree diagnostic scan
  python3 scripts/check_loop_fatal_asserts.py --selfcheck   # prove detection power, no scan

Exit code 0 when clean (or --selfcheck passes), 1 when any hit is found (or --selfcheck fails).
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_new_refs import added_lines, blob  # noqa: E402
from check_policies import REPO_ROOT, Violation, strip_comments  # noqa: E402

import re

RULE = "loop-fatal-assert"

SCAN_ROOT = REPO_ROOT / "test"
SCAN_SUFFIXES = (".cpp", ".hpp")

# Diff-scoped mode restricts to the same domain as the whole-tree default
# (`SCAN_ROOT`), expressed as a path prefix because added_lines() reports
# repo-relative paths rather than filesystem ones.
SCOPE_PREFIX = "test/"

FOR_RE = re.compile(r"\bfor\s*\(")

# gtest's fatal family (ASSERT_*) plus this repo's ImGuiTestEngine fatal macro (IM_CHECK and its
# IM_CHECK_* siblings). Both `return` out of the enclosing function on failure.
FATAL_ASSERT_RE = re.compile(r"\b(?:ASSERT_[A-Z0-9_]+|IM_CHECK\w*)\s*\(")

# A lambda signature: capture list, optional parameter list, optional `mutable`/`noexcept`,
# optional trailing return type, ending at the opening brace of its body. Parameter list and
# trailing-return-type patterns assume no nested parens/braces within them (true of every lambda
# in this codebase today); a lambda whose parameter type itself takes a parenthesized argument
# (e.g. a function-pointer parameter) would not match here and so would NOT be recognised as a
# shielding scope — a false negative, same direction of failure as the list-init gap above.
LAMBDA_RE = re.compile(
    r"\[[^\[\]]*\]"
    r"(?:\s*\([^()]*\))?"
    r"(?:\s*mutable\b)?"
    r"(?:\s*noexcept\b)?"
    r"(?:\s*->\s*[^{;]+?)?"
    r"\s*\{"
)


def _paren_close(code: str, after_open: int) -> int | None:
    """Index of the ')' matching the '(' whose content starts at `after_open`."""
    depth = 1
    i, n = after_open, len(code)
    while i < n:
        c = code[i]
        if c == "(":
            depth += 1
        elif c == ")":
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return None


def _for_body_opens(code: str) -> dict[int, int]:
    """Open-brace index -> 1-based line of the `for` keyword, for every `for (...)`
    immediately (modulo whitespace) followed by a `{` body.
    """
    out: dict[int, int] = {}
    for m in FOR_RE.finditer(code):
        open_paren = m.end() - 1
        close_paren = _paren_close(code, open_paren + 1)
        if close_paren is None:
            continue
        j = close_paren + 1
        while j < len(code) and code[j] in " \t\r\n":
            j += 1
        if j < len(code) and code[j] == "{":
            out[j] = code.count("\n", 0, m.start()) + 1
    return out


def _lambda_body_opens(code: str) -> set[int]:
    return {m.end() - 1 for m in LAMBDA_RE.finditer(code)}


@dataclass
class _Hit:
    lineno: int
    for_line: int


def find_violations_in_text(code: str) -> list[_Hit]:
    """Scan already comment-stripped `code`. See module docstring for the algorithm."""
    for_opens = _for_body_opens(code)
    lambda_opens = _lambda_body_opens(code)
    assert_starts = {m.start() for m in FATAL_ASSERT_RE.finditer(code)}

    hits: list[_Hit] = []
    stack: list[tuple[str, int | None]] = []  # (kind, for_line) ; kind in for/lambda/other
    state = "code"  # code | string | char
    i, n = 0, len(code)
    while i < n:
        c = code[i]
        if state == "code":
            if c == '"':
                state = "string"
                i += 1
                continue
            if c == "'":
                state = "char"
                i += 1
                continue
            if c == "{":
                if i in for_opens:
                    stack.append(("for", for_opens[i]))
                elif i in lambda_opens:
                    stack.append(("lambda", None))
                else:
                    stack.append(("other", None))
                i += 1
                continue
            if c == "}":
                if stack:
                    stack.pop()
                i += 1
                continue
            if i in assert_starts:
                nearest_for_line: int | None = None
                for kind, extra in reversed(stack):
                    if kind == "lambda":
                        break  # shielded: return stops here, never reaches a `for`
                    if kind == "for":
                        nearest_for_line = extra
                        break
                if nearest_for_line is not None:
                    hits.append(_Hit(code.count("\n", 0, i) + 1, nearest_for_line))
            i += 1
        elif state == "string":
            if c == "\\":
                i += 2
                continue
            if c == '"':
                state = "code"
            i += 1
        elif state == "char":
            if c == "\\":
                i += 2
                continue
            if c == "'":
                state = "code"
            i += 1
    return hits


def _violations_for(path: Path, hits: list[_Hit]) -> list[Violation]:
    return [
        Violation(
            path,
            hit.lineno,
            RULE,
            f"fatal assert directly in the body of the `for` loop starting at line "
            f"{hit.for_line}: one row's early failure `return`s out of the function, "
            "silently skipping every row after it. Use `if (...) { ADD_FAILURE() << "
            "...; continue; }` (non-fatal) instead, or move the assert into a lambda "
            "the loop calls per row.",
        )
        for hit in hits
    ]


# Excluded from a whole-subtree `--root` scan: CPM's vendored third-party sources land under
# build/ (e.g. build/cpm_cache/imgui_test_engine/.../*.cpp), which is neither this repo's code nor
# addressable via Violation.render()'s REPO_ROOT-relative path (a relative `--root` walk into it
# raised ValueError before the `root.resolve()` below — the root cause was two bugs stacked: a
# relative root produces relative rglob results, and nothing excluded vendored code from the walk
# in the first place).
BUILD_DIR_NAME = "build"


def check(root: Path) -> list[Violation]:
    root = root.resolve()
    out: list[Violation] = []
    for path in sorted(root.rglob("*")):
        if path.suffix not in SCAN_SUFFIXES or not path.is_file():
            continue
        if BUILD_DIR_NAME in path.relative_to(root).parts[:-1]:
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        code = strip_comments(text)
        out.extend(_violations_for(path, find_violations_in_text(code)))
    return out


def check_diff(diff_args: list[str], content_ref: str) -> list[Violation]:
    """Diff-scoped counterpart of `check()`: only files the diff touches under
    `SCOPE_PREFIX`, and only hits whose assert line the diff added — see the
    module docstring's "Scope" section for why line-level, not file-level.
    """
    out: list[Violation] = []
    for rel, added in sorted(added_lines(diff_args).items()):
        if not rel.startswith(SCOPE_PREFIX) or Path(rel).suffix not in SCAN_SUFFIXES:
            continue
        text = blob(content_ref, rel)
        if not text:
            continue
        code = strip_comments(text)
        hits = [hit for hit in find_violations_in_text(code) if hit.lineno in added]
        out.extend(_violations_for(REPO_ROOT / rel, hits))
    return out


# -- selfcheck -------------------------------------------------------------------
#
# Fixtures below reproduce two real hits this scanner found in this codebase
# (test_gui_widget_rules.cpp:261, test_state_reconcile.cpp:127) as known positives, plus negative
# controls proving the lambda-shield and non-loop cases are NOT flagged. `--selfcheck` fails loudly
# if any expectation does not hold — a scanner with zero detection power passing silently is the
# exact failure this repo has hit five times before (see AGENTS.md's geometry-randomization note),
# so proving detection power against real known-positive shapes, not just synthetic ones, is a
# hardening this scanner commits to over just trusting a clean scan result.

_POSITIVE_INDEXED_FOR = """
TEST(ShapeScalarDomain, EverySlotHasANonEmptyDomain) {
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(slot);
    EXPECT_LT(d.min_value, d.max_value) << "slot " << slot;
    ASSERT_NE(d.fmt, nullptr) << "slot " << slot;
    EXPECT_EQ(d.fmt[0], '%') << "slot " << slot;
  }
}
"""

_POSITIVE_HELPER_FUNCTION_FOR = """
void CheckRows(const Row* rows, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const Row& row = rows[i];
    GuiState s = MakeBaselineState();
    ASSERT_EQ(ReconcileGuiEffects(s), GuiEffects{}) << "the baseline is not quiet";
    row.mutate(s);
    const GuiEffects e = ReconcileGuiEffects(s);
    EXPECT_EQ(e.need_resim, row.resim);
  }
}
"""

_NEGATIVE_LAMBDA_SHIELDED = """
TEST(Widget, EveryCaseChecksItself) {
  for (const Case& c : kCases) {
    auto expect = [&](const Case& row) {
      ASSERT_NE(row.fmt, nullptr) << row.name;
    };
    expect(c);
  }
}
"""

_NEGATIVE_NON_FATAL_GUARD = """
TEST(ShapeScalarDomain, EverySlotHasANonEmptyDomain) {
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    const ShapeScalarDomain& d = ShapeScalarDomainFor(slot);
    if (d.fmt == nullptr) {
      ADD_FAILURE() << "slot " << slot << ": fmt is null";
      continue;
    }
    EXPECT_EQ(d.fmt[0], '%') << "slot " << slot;
  }
}
"""

_NEGATIVE_NOT_IN_LOOP = """
TEST(GuiStateReconcile, WithNoCommittedBaselineNothingIsReported) {
  GuiState s;
  ASSERT_FALSE(s.last_committed_state.has_value());
  EXPECT_EQ(ReconcileGuiEffects(s), GuiEffects{});
}
"""

_NEGATIVE_CALLBACK_DEFINED_BEFORE_LOOP = """
const Row kInertRows[] = {
  { "renderer.fov", [](GuiState& s) { ASSERT_TRUE(false); s.renderer.fov = 45.0f; } },
};
void CheckRows(const Row* rows, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    rows[i].mutate();
  }
}
"""


def _selfcheck_root_relative_path() -> bool:
    """`--root` with a relative path must not crash, and build/ vendored third-party sources under
    it must be skipped rather than flagged. Reproduces the ValueError this scanner raised on
    `--root .` before `check()` resolved its root and excluded build/.
    """
    import os
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        tmp_root = Path(tmp)
        (tmp_root / "build" / "cpm_cache" / "vendor").mkdir(parents=True)
        (tmp_root / "build" / "cpm_cache" / "vendor" / "third_party.cpp").write_text(
            "void F() { for (int i = 0; i < 1; ++i) { ASSERT_TRUE(true); } }\n", encoding="utf-8"
        )
        (tmp_root / "real.cpp").write_text(
            "void G() { for (int i = 0; i < 1; ++i) { ASSERT_TRUE(true); } }\n", encoding="utf-8"
        )
        old_cwd = Path.cwd()
        try:
            os.chdir(tmp_root)
            violations = check(Path("."))
        except Exception as exc:  # noqa: BLE001 - exactly the crash this guards against
            print(f"[FAIL] --root relative path: unexpected exception {exc!r}", file=sys.stderr)
            return False
        finally:
            os.chdir(old_cwd)
        got = sorted(v.path.name for v in violations)
        want = ["real.cpp"]
        ok = got == want
        print(
            f"[{'ok' if ok else 'FAIL'}] --root relative path: no crash, build/ excluded "
            f"(want={want} got={got})",
            file=sys.stderr,
        )
        return ok


def _selfcheck() -> int:
    cases: list[tuple[str, str, bool]] = [
        ("known-positive: indexed for, EverySlotHasANonEmptyDomain shape",
         _POSITIVE_INDEXED_FOR, True),
        ("known-positive: helper-function for, CheckRows shape",
         _POSITIVE_HELPER_FUNCTION_FOR, True),
        ("negative: assert shielded by an inline lambda", _NEGATIVE_LAMBDA_SHIELDED, False),
        ("negative: non-fatal if-guard (the accepted fix shape)", _NEGATIVE_NON_FATAL_GUARD, False),
        ("negative: assert not inside any for loop", _NEGATIVE_NOT_IN_LOOP, False),
        ("negative: per-row callback lambda defined before the loop",
         _NEGATIVE_CALLBACK_DEFINED_BEFORE_LOOP, False),
    ]
    failed = False
    for label, src, want_hit in cases:
        hits = find_violations_in_text(strip_comments(src))
        got_hit = len(hits) > 0
        status = "ok" if got_hit == want_hit else "FAIL"
        if got_hit != want_hit:
            failed = True
        print(
            f"[{status}] {label}: expected_hit={want_hit} got_hit={got_hit} "
            f"(n={len(hits)})",
            file=sys.stderr,
        )
    if not _selfcheck_root_relative_path():
        failed = True
    if failed:
        print(
            "\nSELFCHECK FAILED: detection power not proven — do not trust a clean "
            "tree-scan result until every case above is [ok].",
            file=sys.stderr,
        )
        return 1
    print(f"SELFCHECK passed ({len(cases)} fixtures, known positives + negative controls).")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument(
        "--staged",
        action="store_true",
        help="check the staged diff; this is what happens with no arguments too, "
        "so passing it is documentation rather than a switch",
    )
    group.add_argument("--range", dest="rng", metavar="BASE..HEAD", help="check a commit range")
    group.add_argument(
        "--root",
        type=Path,
        nargs="?",
        const=SCAN_ROOT,
        default=None,
        metavar="PATH",
        help=f"whole-subtree diagnostic scan, ignoring the diff (bare --root uses {SCAN_ROOT})",
    )
    parser.add_argument(
        "--selfcheck",
        action="store_true",
        help="run detection-power fixtures instead of scanning",
    )
    args = parser.parse_args()

    if args.selfcheck:
        return _selfcheck()

    if args.root is not None:
        violations = check(args.root)
    elif args.rng is not None:
        # Same range parsing as check_new_refs.py --range: reject the missing
        # ".." case explicitly, and the three-dot merge-base form, for the same
        # reasons documented there.
        if ".." not in args.rng:
            parser.error("--range expects BASE..HEAD")
        if "..." in args.rng:
            parser.error(
                "--range expects the two-dot form BASE..HEAD; for merge-base "
                "semantics pass `$(git merge-base A B)..B` explicitly"
            )
        base, head = args.rng.split("..", 1)
        if not base or not head:
            parser.error("--range expects BASE..HEAD")
        violations = check_diff([base, head], head)
    else:
        # --staged, passed or defaulted to. Empty content_ref means the index.
        violations = check_diff(["--cached"], "")

    if violations:
        print("Loop-fatal-assert check FAILED:\n", file=sys.stderr)
        for v in sorted(violations, key=lambda x: (str(x.path), x.line)):
            print(v.render(), file=sys.stderr)
        print(
            f"\n{len(violations)} fatal assert(s) found directly in a `for` loop body. "
            "See the rule in this script's module docstring.",
            file=sys.stderr,
        )
        return 1
    print("Loop-fatal-assert check passed (0 hits).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
