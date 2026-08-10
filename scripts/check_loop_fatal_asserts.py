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

The non-fatal half: IM_ERRORF / ADD_FAILURE cascades
------------------------------------------------------
Everything above is about the FATAL family. A non-fatal report (`IM_ERRORF`, gtest's
`ADD_FAILURE`) routes through the same check machinery and sets the case's error status just the
same, but does not `return` on its own — see test/gui/test_gui_shared.hpp's "A table-driven loop
stops at its first failing row" comment for the full mechanism. Every `ImGuiTestContext` action
(`ItemClick`, `ItemInputValue`, `ItemInfo`, ...) opens with `if (IsError()) return;`, so once a
non-fatal report has fired, anything that keeps driving `ctx` in that same test case reads a UI
state nothing actually put it in — and reports THAT as a second, third, ... failure, all of them
echoes of the one real cause. Two shapes reproduce this, both found and fixed by hand across
several rounds of review before this rule existed to catch them mechanically:

  1. `for-loop-nonfatal-cascade`: inside a single `for`- or `while`-loop body, a non-fatal report
     and a ctx-driving call — a `NAME->` member call for any identifier this file's own signatures
     bind as an `ImGuiTestContext*` parameter (see `_ctx_param_names`; not a hardcoded `ctx`
     literal — round 24's real miss was a lambda whose parameter was named `c`), or a call into a
     locally-defined ctx-wrapper helper such as this suite's own `FilterTo`/`RowIsChecked` (any
     function whose body itself drives its own `ImGuiTestContext*` parameter, discovered per-file
     rather than off a hardcoded name list) — not shielded by an inner lambda, with no
     `break`/`continue`/`return` in between. Order within one textual pass of the driving call and
     the report does not decide this on its own: the loop body runs again next iteration, so a
     driving call BEFORE the loop's only report is still at risk — it is the first thing the next
     iteration executes, on whatever error state this iteration's unguarded report left behind
     (round 23's real miss, reproduced in this rule's own selfcheck fixtures). Mirrors the fatal
     rule above, one macro family over — except the fatal rule stays `for`-only by its own
     documented scope; this half also walks `while`.
  2. `lambda-call-cascade`: a named lambda whose body both drives a ctx call and reports
     non-fatally, called back-to-back two or more times with nothing but whitespace between the
     calls — the same machine, one level of indirection. The header comment states it plainly:
     "The trigger is REPETITION, not the `for` keyword." A loop is not required; `verify(a);
     verify(b);` is exactly as dangerous as a loop calling `verify` once per row.

Both are text-only heuristics with the same accepted-gap philosophy as the fatal rule above: a
non-whitespace statement between two calls (the `if (ctx->IsError()) { return; }` guard this rule
wants) is read as "something is there" and suppresses the hit, without checking that the something
IS that guard — proving absence of any short-circuit, not correctness of the one present. A
`NAME->IsError()` call itself is never counted as a "driving" call (it is the query the guard is
built from, not an action `IsError()`'s own no-op-on-error prelude would need to protect).

Round 24 found two more instances of the same "literal judgment call, walked around by the next
author" pattern living in this rule's OWN machinery rather than in the code it scans:

  - The wrap-around branch's suggested fix text used to offer `continue` as an option alongside
    `break`/`return`. It does not work for that branch: a wrap-around hit means the loop wraps
    around to re-run the driving call at the top of the NEXT iteration regardless of what happens
    after the report in THIS one, so a `continue` placed after the report is a no-op against this
    specific cascade — only `break`/`return` actually stop it. `_CascadeHit.wrap_around` now
    distinguishes the two cases and `_for_cascade_violations_for` gives each its own fix text.
  - `ctx->` was matched as a literal identifier; a lambda whose `ImGuiTestContext*` parameter is
    named anything else was invisible to the scan (test_file_ops.cpp's `check` lambda names it
    `c`). `_ctx_param_names`/`_ctx_call_re` now build the driving-call pattern from every
    `ImGuiTestContext*` parameter name this file's OWN signatures actually declare — the same
    registry-over-literal approach `_ctx_wrapper_helper_names` already used for helper functions,
    now applied to the parameter name itself.
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
WHILE_RE = re.compile(r"\bwhile\s*\(")

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

# `auto NAME = <lambda>` — the shape every named-lambda cascade candidate in this codebase uses.
AUTO_LAMBDA_RE = re.compile(r"\b(?:const\s+)?auto\s+(\w+)\s*=\s*" + LAMBDA_RE.pattern)

# Non-fatal report macros: same routing as a fatal assert (both go through ImGuiTestEngine_Check
# and set the case's status to Error) but do NOT `return` from the enclosing function on their own.
NONFATAL_RE = re.compile(r"\b(?:IM_ERRORF|ADD_FAILURE)\s*\(")

# An `ImGuiTestContext*` parameter of ANY lambda or function signature — `[](ImGuiTestContext*
# NAME)` and `Foo(ImGuiTestContext* NAME, ...)` both match (only the parenthesized parameter list
# itself; a lambda's leading `[...]` capture list is not part of this pattern). Used to build a
# PER-FILE set of identifier names that denote a live ImGuiTestContext*, generalizing the
# historical hardcoded `ctx` literal — round 24's real miss (test_file_ops.cpp's `check` lambda
# binds the parameter as `c`) is a renamed-parameter instance of the same underlying pattern this
# whole rule keeps re-encountering: a literal-name judgment call gets walked around by the next
# author who happens to pick a different name (this rule's own history: it was first written
# catching one syntactic shape, then extended to non-fatal reports, then to reports the driving
# call precedes textually, and now to a renamed driving-call parameter — four independent
# "same real rule, different literal disguise" generalizations of one underlying principle: in a
# scope that runs more than once, a non-fatal report must be followed by a short-circuit before
# the context is driven again).
CTX_PARAM_RE = re.compile(r"\(\s*ImGuiTestContext\s*\*\s*(\w+)\b")

# A statement that could plausibly short-circuit the cascade. Intentionally permissive — this is a
# text-only scanner that proves the ABSENCE of any such statement, not the correctness of the one
# present (mirrors LAMBDA_RE's docstring on the same tradeoff).
CONTROL_FLOW_RE = re.compile(r"\b(break|continue|return)\b")

# The signature shape of a locally-defined ctx-wrapper helper: a named function taking an
# `ImGuiTestContext*` as its first parameter, under ANY parameter name (generalized from the
# historical hardcoded `ctx` literal for the same reason as `CTX_PARAM_RE` above). Matches the
# call-site's own name, e.g. `RowIsChecked(ImGuiTestContext* ctx, ...)`. Used by
# `_ctx_wrapper_helper_names` to find helpers whose BODY drives that parameter directly — see that
# function's docstring for why a registry beats a hardcoded name list.
CTX_HELPER_DEF_RE = re.compile(r"\b(\w+)\s*\(\s*ImGuiTestContext\s*\*\s*\w+\b")


def _ctx_param_names(code: str) -> set[str]:
    """Every distinct identifier this file's own signatures bind as an `ImGuiTestContext*`
    parameter — `ctx` whenever it appears literally, plus whatever else an author picked (`c`,
    `tc`, ...). A per-file set beats a hardcoded literal for the same reason
    `_ctx_wrapper_helper_names` already builds its helper registry from callee bodies rather than a
    name list: the next rename is always one commit away.
    """
    return {m.group(1) for m in CTX_PARAM_RE.finditer(code)}


def _ctx_call_re(names: set[str]) -> re.Pattern[str]:
    """A `NAME->` member-call regex over `names`, excluding `IsError` the same way the historical
    single-literal `ctx->` pattern did (`IsError` is the query the short-circuit guard is built
    from, not a driving action that needs the guard). Falls back to the literal `ctx` when a
    file's signatures contain no `ImGuiTestContext*` parameter at all for `_ctx_param_names` to
    find — a name-set search that starts from zero candidates would otherwise silently match
    nothing, which is a worse failure mode than the historical single-literal default.
    """
    alt = "|".join(re.escape(n) for n in sorted(names)) if names else "ctx"
    return re.compile(r"\b(?:" + alt + r")->(?!IsError\b)")


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


def _brace_close(code: str, open_index: int) -> int | None:
    """Index of the '}' matching the '{' at `open_index`, respecting string/char literals."""
    depth = 0
    i, n = open_index, len(code)
    state = "code"
    while i < n:
        c = code[i]
        if state == "code":
            if c == '"':
                state = "string"
            elif c == "'":
                state = "char"
            elif c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    return i
        elif state == "string":
            if c == "\\":
                i += 1
            elif c == '"':
                state = "code"
        elif state == "char":
            if c == "\\":
                i += 1
            elif c == "'":
                state = "code"
        i += 1
    return None


def _ctx_wrapper_helper_names(code: str, ctx_call_re: re.Pattern[str]) -> set[str]:
    """Names of locally-defined functions that take an `ImGuiTestContext*` as their first
    parameter and whose body itself drives it directly (e.g. `FilterTo`/`RowIsChecked`/`ToggleRow`
    in this suite's own per-file test helpers) — the "ctx-wrapper helper" shape. A call to one of
    these, passing the caller's own ctx variable as its first argument, drives it exactly as a
    literal `NAME->` call would, but is textually invisible to `ctx_call_re` on its own.

    A registry built from the callee's OWN body (does it drive the parameter directly?) is used
    instead of a hardcoded name list (`FilterTo`/`RowIsChecked`/`ToggleRow`/...): the list would
    need updating by hand every time this suite's authors write a new per-file wrapper, which is
    exactly the "next round, a new shape" failure mode this rule's own history warns about. A
    function that merely accepts the parameter without driving it (rare, but not impossible) is
    correctly excluded by the body check. `ctx_call_re` is the caller's per-file regex from
    `_ctx_call_re(_ctx_param_names(code))` — generalized past a hardcoded `ctx` literal the same
    way this registry itself already is.
    """
    names: set[str] = set()
    for m in CTX_HELPER_DEF_RE.finditer(code):
        open_paren = code.find("(", m.start())
        if open_paren == -1:
            continue
        close_paren = _paren_close(code, open_paren + 1)
        if close_paren is None:
            continue
        j = close_paren + 1
        while j < len(code) and code[j] in " \t\r\n":
            j += 1
        if j >= len(code) or code[j] != "{":
            continue  # declaration only (no body), e.g. a forward decl or a header prototype
        body_close = _brace_close(code, j)
        if body_close is None:
            continue
        if ctx_call_re.search(code, j, body_close):
            names.add(m.group(1))
    return names


def _ctx_wrapper_call_starts(code: str, helper_names: set[str], ctx_names: set[str]) -> set[int]:
    """Call-site start positions for `NAME(<ctx-name>, ...)` where NAME is a registered
    ctx-wrapper helper (see `_ctx_wrapper_helper_names`) and the first argument is one of this
    file's known `ImGuiTestContext*` identifiers (`ctx_names`, from `_ctx_param_names`) — the same
    driving-call semantics as a `ctx_call_re` match, one level of indirection removed. Generalized
    from a literal `ctx` first-argument match for the same reason as the rest of this rule's
    naming: a caller whose own parameter is named `c` still passes `c`, not `ctx`.
    """
    if not helper_names:
        return set()
    names = ctx_names if ctx_names else {"ctx"}
    pattern = re.compile(
        r"\b(?:" + "|".join(re.escape(n) for n in sorted(helper_names)) + r")\s*\(\s*(?:"
        + "|".join(re.escape(n) for n in sorted(names)) + r")\b"
    )
    return {m.start() for m in pattern.finditer(code)}


def _enclosing_scope_open(code: str, position: int) -> int | None:
    """Index of the `{` opening the innermost brace scope containing `position`, or None at
    file scope. Used to bound a named lambda's call-site search to the block it was DEFINED in:
    two unrelated test cases in one file can both name a local `auto verify = [...]`, and without
    this a call in the second case would be (mis)attributed to the first case's definition too.
    """
    stack: list[int] = []
    state = "code"
    i, n = 0, min(position, len(code))
    while i < n:
        c = code[i]
        if state == "code":
            if c == '"':
                state = "string"
            elif c == "'":
                state = "char"
            elif c == "{":
                stack.append(i)
            elif c == "}":
                if stack:
                    stack.pop()
        elif state == "string":
            if c == "\\":
                i += 1
            elif c == '"':
                state = "code"
        elif state == "char":
            if c == "\\":
                i += 1
            elif c == "'":
                state = "code"
        i += 1
    return stack[-1] if stack else None


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


def _while_body_opens(code: str) -> dict[int, int]:
    """Open-brace index -> 1-based line of the `while` keyword, for every `while (...)`
    immediately (modulo whitespace) followed by a `{` body. Mirrors `_for_body_opens`; used only
    by the non-fatal-cascade detector (round 23's requirement 1 covers `for` and `while` alike —
    the fatal-assert rule above stays `for`-only, its own documented scope).
    """
    out: dict[int, int] = {}
    for m in WHILE_RE.finditer(code):
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


@dataclass
class _CascadeHit:
    lineno: int
    marker_lineno: int
    loop_kind: str = "for"
    wrap_around: bool = False


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


def find_for_cascade_in_text(code: str) -> list[_CascadeHit]:
    """Shape 1 of the non-fatal half (see module docstring): inside a single `for`- or
    `while`-loop body (round 23's requirement 1 covers both alike; the fatal-assert rule above
    stays `for`-only, its own separately documented scope), a non-fatal report
    (IM_ERRORF/ADD_FAILURE) and a ctx-driving call (a `NAME->` member call for any of this file's
    own `ImGuiTestContext*` parameter names — see `_ctx_param_names` — or a call to a
    locally-defined ctx-wrapper helper, see `_ctx_wrapper_helper_names`), with no
    `break`/`continue`/`return` in between.

    `armed` is tracked per loop-frame (`for` or `while`, identically) on the walk's own stack: a
    qualifying report call sets it, a qualifying driving call while set is a same-iteration hit,
    and `break`/`continue` clear only the nearest enclosing loop frame while `return` clears every
    loop frame up to (not past) the nearest enclosing lambda boundary — the same reach a `return`
    has in the real function it appears in.

    Order within one textual pass is not the whole story: a loop body runs again next iteration,
    so a driving call that appears BEFORE the only report in the body is still at risk — it is the
    first thing the NEXT iteration executes, on a ctx the PREVIOUS iteration may have left in an
    error state. `first_ctx_line` records the first driving call seen in the frame (whichever
    order it comes in); when the frame's closing `}` is reached still armed with no guard since the
    last report, and that first driving call is textually earlier than the report that left it
    armed, a wrap-around hit is reported there (`hit.wrap_around=True` — see
    `_for_cascade_violations_for` for why its suggested fix differs from a same-iteration hit's).
    The `<` guard against double-reporting a same-iteration hit already caught by the forward-order
    check above (round 23's real miss: test/gui/functional/test_defaults_panel.cpp's
    `reset_all_unchecks_every_row_...` case, where a helper call drives before the loop's own
    unguarded report, not after; round 24's real miss: test/gui/functional/test_file_ops.cpp's
    `check` lambda, the identical shape with its ctx parameter renamed to `c` — caught now because
    `_ctx_param_names` collects `c` instead of assuming the literal `ctx`).
    """
    for_opens = _for_body_opens(code)
    while_opens = _while_body_opens(code)
    lambda_opens = _lambda_body_opens(code)
    nonfatal_starts = {m.start() for m in NONFATAL_RE.finditer(code)}
    ctx_names = _ctx_param_names(code)
    ctx_call_re = _ctx_call_re(ctx_names)
    wrapper_names = _ctx_wrapper_helper_names(code, ctx_call_re)
    ctx_starts = {m.start() for m in ctx_call_re.finditer(code)} | _ctx_wrapper_call_starts(
        code, wrapper_names, ctx_names
    )
    control_flow_starts = {m.start(): m.group(1) for m in CONTROL_FLOW_RE.finditer(code)}

    hits: list[_CascadeHit] = []
    # Each frame: [kind, loop_line_or_None, armed, marker_line_or_None, first_ctx_line_or_None]
    # kind is "for", "while", "lambda", or "other"; "for" and "while" are treated identically
    # below except for the string tag a hit records.
    stack: list[list] = []
    state = "code"
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
                    stack.append(["for", for_opens[i], False, None, None])
                elif i in while_opens:
                    stack.append(["while", while_opens[i], False, None, None])
                elif i in lambda_opens:
                    stack.append(["lambda", None, False, None, None])
                else:
                    stack.append(["other", None, False, None, None])
                i += 1
                continue
            if c == "}":
                if stack:
                    frame = stack.pop()
                    if (
                        frame[0] in ("for", "while")
                        and frame[2]
                        and frame[4] is not None
                        and frame[4] < frame[3]
                    ):
                        hits.append(
                            _CascadeHit(frame[4], frame[3], loop_kind=frame[0], wrap_around=True)
                        )
                i += 1
                continue
            if i in nonfatal_starts:
                for frame in reversed(stack):
                    if frame[0] == "lambda":
                        break
                    if frame[0] in ("for", "while"):
                        frame[2] = True
                        frame[3] = code.count("\n", 0, i) + 1
                        break
                i += 1
                continue
            if i in ctx_starts:
                for frame in reversed(stack):
                    if frame[0] == "lambda":
                        break
                    if frame[0] in ("for", "while"):
                        if frame[4] is None:
                            frame[4] = code.count("\n", 0, i) + 1
                        if frame[2]:
                            hits.append(
                                _CascadeHit(
                                    code.count("\n", 0, i) + 1, frame[3], loop_kind=frame[0]
                                )
                            )
                        break
                i += 1
                continue
            if i in control_flow_starts:
                kw = control_flow_starts[i]
                if kw == "return":
                    for frame in reversed(stack):
                        if frame[0] == "lambda":
                            break
                        if frame[0] in ("for", "while"):
                            frame[2] = False
                else:  # break / continue: only the nearest enclosing loop
                    for frame in reversed(stack):
                        if frame[0] == "lambda":
                            break
                        if frame[0] in ("for", "while"):
                            frame[2] = False
                            break
                i += 1
                continue
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


def find_lambda_call_cascade_in_text(code: str) -> list[_CascadeHit]:
    """Shape 2 of the non-fatal half (see module docstring): a named lambda (`auto NAME = [...]
    { ... }`) whose body drives a ctx call and reports non-fatally, called back-to-back two or
    more times with nothing but whitespace between the calls.

    Deliberately conservative: any non-whitespace text between two calls to the same name — the
    `if (ctx->IsError()) { return; }` guard this rule wants, but also anything else — suppresses
    the hit for that pair, same accepted-gap stance as the rest of this scanner.
    """
    ctx_call_re = _ctx_call_re(_ctx_param_names(code))
    hits: list[_CascadeHit] = []
    for def_match in AUTO_LAMBDA_RE.finditer(code):
        name = def_match.group(1)
        body_open = def_match.end() - 1
        body_close = _brace_close(code, body_open)
        if body_close is None:
            continue
        body = code[body_open:body_close]
        if not (ctx_call_re.search(body) and NONFATAL_RE.search(body)):
            continue

        scope_open = _enclosing_scope_open(code, def_match.start())
        scope_close = _brace_close(code, scope_open) if scope_open is not None else len(code)

        call_re = re.compile(r"\b" + re.escape(name) + r"\s*\(")
        call_spans: list[tuple[int, int]] = []
        for m in call_re.finditer(code):
            if m.start() <= body_close:
                continue  # inside the lambda's own definition/body, not a call site
            if scope_close is not None and m.start() >= scope_close:
                continue  # outside the block this lambda was defined in — a different case's own
            close_paren = _paren_close(code, m.end())
            if close_paren is None:
                continue
            stmt_end = close_paren + 1
            while stmt_end < len(code) and code[stmt_end] in " \t":
                stmt_end += 1
            if stmt_end < len(code) and code[stmt_end] == ";":
                stmt_end += 1
            call_spans.append((m.start(), stmt_end))

        for (prev_start, prev_end), (cur_start, _cur_end) in zip(call_spans, call_spans[1:]):
            if code[prev_end:cur_start].strip() == "":
                hits.append(_CascadeHit(
                    code.count("\n", 0, cur_start) + 1,
                    code.count("\n", 0, prev_start) + 1,
                ))
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


FOR_CASCADE_RULE = "loop-nonfatal-cascade"
LAMBDA_CASCADE_RULE = "lambda-call-cascade"


def _for_cascade_violations_for(path: Path, hits: list[_CascadeHit]) -> list[Violation]:
    out: list[Violation] = []
    for hit in hits:
        if hit.wrap_around:
            # The driving call at `hit.lineno` is textually BEFORE the report it is dangling
            # after (the loop wraps around to run it again next iteration). `continue` placed
            # after the report does not help here: it jumps to the top of THIS SAME loop body,
            # which is exactly where the driving call already sits, so it runs unconditionally
            # either way. Only `break` (stop the loop) or `return` (stop the function) actually
            # prevent the next iteration's driving call from running on the errored ctx — the
            # round 24 review's Major #1: this branch's suggested fix used to offer `continue` as
            # if it were interchangeable with the same-iteration case below, and it is not.
            fix = (
                "Add `if (ctx->IsError()) { break; }` (or `return`) between them — `continue` "
                "does NOT help here: the loop wraps around and re-runs this driving call at the "
                "top of the next iteration regardless, so `continue` right after the report is a "
                "no-op against this specific cascade."
            )
        else:
            fix = "Add `if (ctx->IsError()) { break; }` (or `continue`/`return`) between them."
        out.append(
            Violation(
                path,
                hit.lineno,
                FOR_CASCADE_RULE,
                "ctx-> call left dangling after a non-fatal report at line "
                f"{hit.marker_lineno} in the same `{hit.loop_kind}` body: once that report fires, "
                "this call silently no-ops (every ImGuiTestContext action opens with `if "
                f"(IsError()) return;`) and whatever it reports next is an echo of the real "
                f"failure, not a new one. {fix}",
            )
        )
    return out


def _lambda_cascade_violations_for(path: Path, hits: list[_CascadeHit]) -> list[Violation]:
    return [
        Violation(
            path,
            hit.lineno,
            LAMBDA_CASCADE_RULE,
            "named lambda called again here with nothing but whitespace since the call at line "
            f"{hit.marker_lineno} — if that one reported a non-fatal failure, every ctx-> action "
            "in this call silently no-ops and whatever it reports is an echo, not a new failure. "
            "Add `if (ctx->IsError()) { return; }` between the calls.",
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
        out.extend(_for_cascade_violations_for(path, find_for_cascade_in_text(code)))
        out.extend(_lambda_cascade_violations_for(path, find_lambda_call_cascade_in_text(code)))
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
        for_cascade_hits = [
            hit for hit in find_for_cascade_in_text(code) if hit.lineno in added
        ]
        out.extend(_for_cascade_violations_for(REPO_ROOT / rel, for_cascade_hits))
        lambda_cascade_hits = [
            hit for hit in find_lambda_call_cascade_in_text(code) if hit.lineno in added
        ]
        out.extend(_lambda_cascade_violations_for(REPO_ROOT / rel, lambda_cascade_hits))
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

# -- selfcheck fixtures: the non-fatal half -------------------------------------
#
# Shape 1 (for-loop-nonfatal-cascade) reproduces the real, fixed
# test_view_display_controls.cpp:519-536 shape (a non-fatal report followed by more `ctx->`
# driving in the same `for` body); shape 2 (lambda-call-cascade) reproduces the real
# test_edit_modal.cpp shape this rule exists to catch — a `verify` lambda called back-to-back
# (reset_view_lands_on_the_same_pose_the_card_thumbnail_uses and
# an_axis_preset_button_resets_the_modal_preview) with no `IsError()` guard between calls.

_POSITIVE_FOR_NONFATAL_CASCADE = """
TEST(ViewDisplayControls, EveryFullSkyLensZeroesTheStoredAngles) {
  for (const int lens : kFullSkyLensTypes) {
    if (gui::g_state.renderer.roll != 0.0f) {
      IM_ERRORF("full-sky lens %d kept a nonzero roll", lens);
    }
    ctx->ItemClick("**/Reset##view");
  }
}
"""

_NEGATIVE_FOR_NONFATAL_GUARDED = """
TEST(ViewDisplayControls, EveryFullSkyLensZeroesTheStoredAngles) {
  for (const int lens : kFullSkyLensTypes) {
    if (gui::g_state.renderer.roll != 0.0f) {
      IM_ERRORF("full-sky lens %d kept a nonzero roll", lens);
    }
    if (ctx->IsError()) {
      break;
    }
    ctx->ItemClick("**/Reset##view");
  }
}
"""

# Round 23's real miss (test/gui/functional/test_defaults_panel.cpp,
# reset_all_unchecks_every_row_and_writes_nothing_until_save): a driving call BEFORE the loop's
# only report, with nothing after it. This used to be a negative control on the theory that a
# report with nothing driving `ctx` afterward in the same textual pass can't cascade — wrong in a
# loop, which runs its body again: the driving call at the top of the NEXT iteration executes on
# whatever error state THIS iteration's unguarded report left behind. See the "wrap-around" note in
# find_for_cascade_in_text's docstring.
_POSITIVE_FOR_CTX_BEFORE_REPORT_WRAP_AROUND = """
TEST(ViewDisplayControls, ReportsAfterDriving) {
  for (const int lens : kFullSkyLensTypes) {
    ctx->ItemClick("**/Reset##view");
    if (gui::g_state.renderer.roll != 0.0f) {
      IM_ERRORF("full-sky lens %d kept a nonzero roll", lens);
    }
  }
}
"""

# The exact shape of round 23's real miss, this time with the driving calls going through
# locally-defined ctx-wrapper helpers (`FilterTo`/`RowIsChecked`) the way test_defaults_panel.cpp's
# real case does, rather than a literal `ctx->` call — exercising `_ctx_wrapper_helper_names` /
# `_ctx_wrapper_call_starts` together with the wrap-around fix above, on the actual code shape that
# slipped past round 23's review.
_POSITIVE_FOR_CTX_WRAPPER_HELPER_BEFORE_REPORT = """
void FilterTo(ImGuiTestContext* ctx, const char* text) {
  ctx->ItemInputValue("**/###defaults_search", text);
}

bool RowIsChecked(ImGuiTestContext* ctx, const std::string& key_path) {
  return ctx->ItemIsChecked(key_path.c_str());
}

TEST(DefaultsPanel, reset_all_unchecks_every_row_and_writes_nothing_until_save) {
  for (const auto& row : rows) {
    FilterTo(ctx, row.key_path.c_str());
    if (RowIsChecked(ctx, row.key_path)) {
      IM_ERRORF("row '%s' survived Reset all still checked", row.key_path.c_str());
    }
  }
}
"""

_NEGATIVE_FOR_REPORT_IN_INLINE_LAMBDA = """
TEST(ViewDisplayControls, ReportInsideInlineLambda) {
  for (const int lens : kFullSkyLensTypes) {
    auto check = [&]() { IM_ERRORF("lens %d bad", lens); };
    check();
    ctx->ItemClick("**/Reset##view");
  }
}
"""

# Round 24's real miss (test/gui/functional/test_file_ops.cpp, the `check` lambda at
# lines 385-395): the identical wrap-around shape as
# `_POSITIVE_FOR_CTX_BEFORE_REPORT_WRAP_AROUND` above, but the lambda's `ImGuiTestContext*`
# parameter is named `c`, not `ctx` — invisible to a scan hardcoded to the literal `ctx->`.
# Reproduces the real code (trimmed to the shape that matters) as a known positive per this rule's
# own G3 discipline: the sample must actually turn red once `_ctx_param_names` is in place, or it
# does not count as proof of detection power.
_POSITIVE_FOR_RENAMED_CTX_PARAM_WRAP_AROUND = """
TEST(FileOps, TopBarSaveMenuGatesMatchRunState) {
  const auto check = [](ImGuiTestContext* c, const char* scenario, const Item* items, int n) {
    c->ItemClick("##TopBar/Save");
    c->SetRef("//$FOCUSED");
    for (int i = 0; i < n; ++i) {
      const bool disabled = IsDisabled(c->ItemInfo(items[i].path));
      if (disabled == items[i].expect_enabled) {
        IM_ERRORF("%s: '%s' is %s and should not be", scenario, items[i].path,
                  disabled ? "unreachable" : "reachable");
      }
    }
  };
}
"""

_NEGATIVE_FOR_RENAMED_CTX_PARAM_GUARDED = """
TEST(FileOps, TopBarSaveMenuGatesMatchRunState) {
  const auto check = [](ImGuiTestContext* c, const char* scenario, const Item* items, int n) {
    c->ItemClick("##TopBar/Save");
    c->SetRef("//$FOCUSED");
    for (int i = 0; i < n; ++i) {
      const bool disabled = IsDisabled(c->ItemInfo(items[i].path));
      if (disabled == items[i].expect_enabled) {
        IM_ERRORF("%s: '%s' is %s and should not be", scenario, items[i].path,
                  disabled ? "unreachable" : "reachable");
      }
      if (c->IsError()) {
        break;
      }
    }
  };
}
"""

# Round 23's requirement 1 covers `while` the same as `for` (the fatal-assert rule above stays
# `for`-only, its own separately documented scope); these mirror
# `_POSITIVE_FOR_NONFATAL_CASCADE`/`_NEGATIVE_FOR_NONFATAL_GUARDED` one keyword over.
_POSITIVE_WHILE_NONFATAL_CASCADE = """
TEST(ViewDisplayControls, EveryFullSkyLensZeroesTheStoredAngles) {
  int lens_index = 0;
  while (lens_index < kFullSkyLensTypeCount) {
    const int lens = kFullSkyLensTypes[lens_index];
    if (gui::g_state.renderer.roll != 0.0f) {
      IM_ERRORF("full-sky lens %d kept a nonzero roll", lens);
    }
    ctx->ItemClick("**/Reset##view");
    ++lens_index;
  }
}
"""

_NEGATIVE_WHILE_NONFATAL_GUARDED = """
TEST(ViewDisplayControls, EveryFullSkyLensZeroesTheStoredAngles) {
  int lens_index = 0;
  while (lens_index < kFullSkyLensTypeCount) {
    const int lens = kFullSkyLensTypes[lens_index];
    if (gui::g_state.renderer.roll != 0.0f) {
      IM_ERRORF("full-sky lens %d kept a nonzero roll", lens);
    }
    if (ctx->IsError()) {
      break;
    }
    ctx->ItemClick("**/Reset##view");
    ++lens_index;
  }
}
"""

_POSITIVE_LAMBDA_CASCADE_RESET_VIEW = """
TEST(EditModal, reset_view_lands_on_the_same_pose_the_card_thumbnail_uses) {
  auto verify = [&](const char* label) {
    ResetTestState();
    ctx->ItemClick("**/Edit##cr");
    ctx->ItemClick("**/Reset View##modal");
    if (observed != expected) {
      IM_ERRORF("preset=%s mismatch", label);
    }
  };

  verify("Plate");
  verify("Column");
}
"""

_POSITIVE_LAMBDA_CASCADE_AXIS_PRESET = """
TEST(EditModal, an_axis_preset_button_resets_the_modal_preview) {
  auto verify = [&](const char* label) {
    ResetTestState();
    ctx->ItemClick("**/Edit##cr");
    ctx->ItemClick(("**/" + std::string(label)).c_str());
    if (observed != expected) {
      IM_ERRORF("preset=%s mismatch", label);
    }
  };

  verify("Column");
  verify("Plate");
}
"""

_NEGATIVE_LAMBDA_CASCADE_GUARDED = """
TEST(EntryManagement, cancelling_pick_mode_does_not_reopen_the_modal) {
  auto cancel_by = [ctx](bool by_escape) {
    ResetTestState();
    ctx->ItemClick("**/Edit##cr");
    if (gui::g_state.pick_link_source.has_value()) {
      IM_ERRORF("%s did not cancel", by_escape ? "Escape" : "a click off the cards");
    }
  };

  cancel_by(/*by_escape=*/true);

  if (ctx->IsError()) {
    return;
  }
  cancel_by(/*by_escape=*/false);
}
"""

_NEGATIVE_LAMBDA_CASCADE_NO_CTX_OR_REPORT = """
void MakeRows() {
  auto add_row = [&](int value) { total += value; };
  add_row(1);
  add_row(2);
}
"""

_NEGATIVE_LAMBDA_CASCADE_SINGLE_CALL = """
TEST(EditModal, single_call_has_no_repetition) {
  auto verify = [&](const char* label) {
    ctx->ItemClick("**/Edit##cr");
    if (bad) {
      IM_ERRORF("bad %s", label);
    }
  };
  verify("Plate");
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


def _selfcheck_wrap_around_fix_text() -> bool:
    """Round 24's Major #1: the wrap-around branch's suggested fix must not offer `continue` (it
    does not stop the cascade there — see `_for_cascade_violations_for`'s docstring comment), while
    the same-iteration branch's fix text must keep offering it (it DOES work there; regression
    guard against overcorrecting the message for every hit).
    """
    ok = True

    # The distinguishing substrings are the ENUMERATED options in each branch's fix sentence, not
    # bare "continue" — the wrap-around branch's message legitimately says the word once, in the
    # explanation of why `continue` does not help there (that explanation is the point of Major
    # #1's fix); what must NOT appear is `continue` offered as an option alongside break/return.
    wrap_hits = find_for_cascade_in_text(strip_comments(_POSITIVE_FOR_CTX_BEFORE_REPORT_WRAP_AROUND))
    wrap_violations = _for_cascade_violations_for(Path("dummy_wrap.cpp"), wrap_hits)
    wrap_ok = bool(wrap_violations) and all(
        "(or `return`)" in v.message and "(or `continue`/`return`)" not in v.message
        for v in wrap_violations
    )
    print(
        f"[{'ok' if wrap_ok else 'FAIL'}] wrap-around fix text does not offer `continue` as an "
        f"option (n_violations={len(wrap_violations)})",
        file=sys.stderr,
    )
    ok = ok and wrap_ok

    same_iter_hits = find_for_cascade_in_text(strip_comments(_POSITIVE_FOR_NONFATAL_CASCADE))
    same_iter_violations = _for_cascade_violations_for(Path("dummy_same.cpp"), same_iter_hits)
    same_iter_ok = bool(same_iter_violations) and all(
        "(or `continue`/`return`)" in v.message for v in same_iter_violations
    )
    print(
        f"[{'ok' if same_iter_ok else 'FAIL'}] same-iteration fix text still offers `continue` "
        f"(n_violations={len(same_iter_violations)})",
        file=sys.stderr,
    )
    ok = ok and same_iter_ok

    return ok


def _selfcheck() -> int:
    cases: list[tuple[str, str, bool, object]] = [
        ("known-positive: indexed for, EverySlotHasANonEmptyDomain shape",
         _POSITIVE_INDEXED_FOR, True, find_violations_in_text),
        ("known-positive: helper-function for, CheckRows shape",
         _POSITIVE_HELPER_FUNCTION_FOR, True, find_violations_in_text),
        ("negative: assert shielded by an inline lambda", _NEGATIVE_LAMBDA_SHIELDED, False,
         find_violations_in_text),
        ("negative: non-fatal if-guard (the accepted fix shape)", _NEGATIVE_NON_FATAL_GUARD, False,
         find_violations_in_text),
        ("negative: assert not inside any for loop", _NEGATIVE_NOT_IN_LOOP, False,
         find_violations_in_text),
        ("negative: per-row callback lambda defined before the loop",
         _NEGATIVE_CALLBACK_DEFINED_BEFORE_LOOP, False, find_violations_in_text),
        # -- non-fatal half: for-loop-nonfatal-cascade -------------------------
        ("known-positive: for-loop-nonfatal-cascade, EveryFullSkyLensZeroesTheStoredAngles shape",
         _POSITIVE_FOR_NONFATAL_CASCADE, True, find_for_cascade_in_text),
        ("negative: for-loop-nonfatal-cascade guarded by IsError()/break (the accepted fix shape, "
         "also regression-guards the ctx->IsError() exclusion)",
         _NEGATIVE_FOR_NONFATAL_GUARDED, False, find_for_cascade_in_text),
        ("known-positive: round-23 wrap-around, ctx-> drives BEFORE the loop's only report "
         "(next iteration inherits the error)",
         _POSITIVE_FOR_CTX_BEFORE_REPORT_WRAP_AROUND, True, find_for_cascade_in_text),
        ("known-positive: round-23 real miss, ctx-wrapper helper (FilterTo/RowIsChecked) drives "
         "before the loop's only report (reset_all_unchecks_every_row_and_writes_nothing_until_save)",
         _POSITIVE_FOR_CTX_WRAPPER_HELPER_BEFORE_REPORT, True, find_for_cascade_in_text),
        ("negative: report shielded by an inline lambda inside the for body",
         _NEGATIVE_FOR_REPORT_IN_INLINE_LAMBDA, False, find_for_cascade_in_text),
        ("known-positive: round-24 real miss, ctx parameter renamed to `c` "
         "(test_file_ops.cpp's `check` lambda) — proves _ctx_param_names catches a non-`ctx` name",
         _POSITIVE_FOR_RENAMED_CTX_PARAM_WRAP_AROUND, True, find_for_cascade_in_text),
        ("negative: the same renamed-parameter shape, guarded by IsError()/break",
         _NEGATIVE_FOR_RENAMED_CTX_PARAM_GUARDED, False, find_for_cascade_in_text),
        ("known-positive: while-loop-nonfatal-cascade, same shape as the for-loop case "
         "(round 23 requirement 1: for/while alike)",
         _POSITIVE_WHILE_NONFATAL_CASCADE, True, find_for_cascade_in_text),
        ("negative: while-loop-nonfatal-cascade guarded by IsError()/break",
         _NEGATIVE_WHILE_NONFATAL_GUARDED, False, find_for_cascade_in_text),
        # -- non-fatal half: lambda-call-cascade -----------------------------
        ("known-positive: round-21 lambda-call-cascade shape 1 "
         "(reset_view_lands_on_the_same_pose_the_card_thumbnail_uses)",
         _POSITIVE_LAMBDA_CASCADE_RESET_VIEW, True, find_lambda_call_cascade_in_text),
        ("known-positive: round-21 lambda-call-cascade shape 2 "
         "(an_axis_preset_button_resets_the_modal_preview)",
         _POSITIVE_LAMBDA_CASCADE_AXIS_PRESET, True, find_lambda_call_cascade_in_text),
        ("negative: lambda-call-cascade guarded by IsError()/return (the accepted fix shape, "
         "cancel_by)",
         _NEGATIVE_LAMBDA_CASCADE_GUARDED, False, find_lambda_call_cascade_in_text),
        ("negative: repeated named lambda with neither ctx-> nor a non-fatal report",
         _NEGATIVE_LAMBDA_CASCADE_NO_CTX_OR_REPORT, False, find_lambda_call_cascade_in_text),
        ("negative: single call site, no repetition to cascade",
         _NEGATIVE_LAMBDA_CASCADE_SINGLE_CALL, False, find_lambda_call_cascade_in_text),
    ]
    failed = False
    for label, src, want_hit, detector in cases:
        hits = detector(strip_comments(src))
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
    if not _selfcheck_wrap_around_fix_text():
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
