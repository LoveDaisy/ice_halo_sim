#!/usr/bin/env python3
"""Diff-scoped policy check: a newly registered gui_test case must actually use its ImGuiTestContext.

Third entry point next to check_policies.py and check_new_refs.py, and a
deliberate one. check_policies.py asks "what state is the repo in?" (whole-tree).
check_new_refs.py asks "what did you just write?" over prose. This asks the same
"what did you just write?" question, but over a different domain entirely: C++
test structure — a lambda signature and its brace-balanced body. Folding it into
check_new_refs.py's PATTERNS table would put a C++ structural parser behind a
name and a docstring that promise a prose regex battery.

The rule
--------
The GUI tests are split across two targets on a *link* boundary:

  - gui_test      needs a window, a GL context and the ImGui test engine, so CI
                  cannot run it (the runners have no display) — it is build-only
                  there. Its cases really do execute only on a developer desktop.
  - gui_unit_test links the same lumice_gui_obj with no window, no GL context and
                  no test engine, so its cases run in CI on every platform that
                  builds the GUI.

A test that needs no rendered frame therefore buys nothing by living in gui_test
and loses continuous cross-platform coverage. But the split is only a convention,
and the path of least resistance runs the wrong way: adding to gui_test is the
muscle memory, and it always compiles. This gate makes the boundary hold on new
cases.

What is rejected
----------------
A `TestFunc = [](ImGuiTestContext* ...)` lambda whose signature line is ADDED by
this diff, in a file under test/gui/ ending in .cpp, when either:

  1. the parameter is anonymous — `[](ImGuiTestContext*)`, or the equivalent
     `[](ImGuiTestContext* /*ctx*/)` where the name is commented out; or
  2. the parameter is named and the body calls `IM_UNUSED(<name>)` while
     containing no other mention of `<name>` at all.

Both are an explicit statement that the case does not drive the GUI — one made
by the compiler (an unnamed parameter cannot be referenced), one made by the
author's own hand. Neither is an inference of ours, which is why the false
positive rate is what it is.

What is NOT rejected, and why that is accepted
----------------------------------------------
  - `t->TestFunc = SomeNamedFunction;` — no lambda signature to read. Detecting
    it would mean resolving a symbol to its definition, which is a compiler's
    job, not a regex's.
  - A named parameter never referenced and never marked `IM_UNUSED` — it
    compiles because the build is -Wall -Wextra without -Werror, so an unused
    parameter is a warning rather than an error. Catching this needs the "body
    mentions the name nowhere" inference, with no explicit author or compiler
    statement behind it, and inference is exactly what makes a gate misfire.
  - Pre-existing cases of any shape. The gate reads added lines only, so it
    starts green with no baseline to maintain — same reasoning as its
    prose-scanning sibling, and the same reason the alternative (a whole-tree
    gate) is not on the table: the existing body cannot be zeroed out first.

There is no inline exemption, by design. If a new case genuinely needs a frame
but never happens to touch ctx, the fix is to make it drive ctx (`ctx->Yield()`
is the frame pump), which both states the dependency and passes this gate. If
some case really cannot be written that way, the rule is wrong and the rule
should change — here, in this file.

Why "mentions the name" and not "calls name->"
----------------------------------------------
The narrower `name->` reading was tried first and is wrong. Passing ctx bare to
a helper — `RunSomeScenario(ctx, ...)` — is a legitimate and common use that
never writes `->`; over the current tree the narrow reading misjudges 17 such
cases as unused against 4 genuine ones. A gate that blocks commits cannot be
built on a predicate with that ratio, so the bare-identifier reading is the one
implemented.

Usage
-----
  python3 scripts/check_new_gui_tests.py --staged          # pre-commit: the index
  python3 scripts/check_new_gui_tests.py --range A..B      # CI: PR vs merge-base

Exit code 0 when clean, 1 when any newly added case is rejected.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path, PurePosixPath

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_new_refs import added_lines, blob  # noqa: E402
from check_policies import REPO_ROOT, Violation, strip_comments  # noqa: E402

RULE = "gui-test-unused-ctx"

# Only the display-requiring target. gui_unit_test's sources live under
# test/unit-correctness/gui/ and do not use IM_REGISTER_TEST at all, so they are
# out of scope by construction; pinning the prefix here keeps it that way even if
# that ever changes, rather than letting the rule silently widen.
SCOPE_PREFIX = "test/gui/"
SCOPE_SUFFIX = ".cpp"

# The capture list is matched as a literal `[]` rather than `\[[^\]]*\]` because
# ImGuiTest::TestFunc is a plain function pointer (`void (*)(ImGuiTestContext*)`).
# A capturing lambda does not convert to one, so it cannot be assigned here — the
# empty capture is enforced by the type system, not by convention, and widening
# the pattern would only buy shapes that do not compile.
#
# \s matches newlines, so a signature broken across lines is still found; the
# violation is anchored at the line the match STARTS on, i.e. the `TestFunc` line.
# That is one deterministic anchor. Testing every line the match spans instead
# would fire when a later reformat touched only the closing paren, which is not
# a newly registered case.
#
# The parameter name is a capture group rather than a hardcoded `ctx` so that
# renaming the parameter cannot silently disable the rule.
SIGNATURE_RE = re.compile(
    r"TestFunc\s*=\s*\[\s*\]\s*\(\s*ImGuiTestContext\s*\*\s*([A-Za-z_]\w*)?\s*\)\s*\{"
)

FIX_HINT = (
    "If it needs no rendered frame, move it to gui_unit_test "
    "(test/unit-correctness/gui/), which runs in CI on every platform. "
    "If it does need one — a real frame, a GL texture, a driven widget — drive "
    "the context explicitly instead (ctx->Yield() is the frame pump), which "
    "states that dependency and satisfies this rule."
)


def _brace_inner(tail_after_open_brace: str) -> str:
    """Text between an already-consumed `{` and its depth-matched `}`.

    The brace-counting sibling of check_policies._paren_inner, and it inherits
    that function's one limitation: a brace inside a string literal is counted.
    The consequence is bounded and lands on the safe side. An unbalanced literal
    makes the scan run past the real closing brace, so the "body" swells to
    include later code — which then almost certainly mentions the parameter name,
    and the case is cleared. The failure mode is a miss, not a block, which is
    the direction this gate is meant to fail in.
    """
    depth = 1
    for i, ch in enumerate(tail_after_open_brace):
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return tail_after_open_brace[:i]
    return tail_after_open_brace


def _uses(name: str, body: str) -> bool:
    """True if `body` mentions `name` outside of its own IM_UNUSED(name) call(s).

    Every IM_UNUSED(name) is removed first, not just a leading one: the marker is
    the statement being tested for, so leaving any occurrence in would make the
    body look like it uses the parameter it just declared unused.
    """
    without_marker = re.sub(rf"IM_UNUSED\s*\(\s*{re.escape(name)}\s*\)", "", body)
    return re.search(rf"\b{re.escape(name)}\b", without_marker) is not None


def _is_marked_unused(name: str, body: str) -> bool:
    return re.search(rf"IM_UNUSED\s*\(\s*{re.escape(name)}\s*\)", body) is not None


def check(diff_args: list[str], content_ref: str) -> tuple[list[Violation], int]:
    """Returns (violations, number of newly added TestFunc signatures scanned)."""
    violations: list[Violation] = []
    scanned = 0
    for rel, linenos in sorted(added_lines(diff_args).items()):
        p = PurePosixPath(rel)
        if not rel.startswith(SCOPE_PREFIX) or p.suffix != SCOPE_SUFFIX:
            continue
        text = blob(content_ref, rel)
        if not text:
            continue
        # Comments are blanked to spaces with offsets preserved, which does two
        # jobs at once: `[](ImGuiTestContext* /*ctx*/)` reduces to an anonymous
        # parameter — which is what that shape means — and a body that merely
        # talks about ctx in a comment no longer reads as using it. Line numbers
        # survive because strip_comments keeps every newline.
        code = strip_comments(text)
        for m in SIGNATURE_RE.finditer(code):
            lineno = code.count("\n", 0, m.start()) + 1
            if lineno not in linenos:
                continue  # pre-existing case; this gate only reads what you added
            scanned += 1
            name = m.group(1)
            if not name:
                violations.append(
                    Violation(
                        REPO_ROOT / rel,
                        lineno,
                        RULE,
                        "this newly registered gui_test case takes an anonymous "
                        "ImGuiTestContext*, so it cannot drive the GUI at all. "
                        + FIX_HINT,
                    )
                )
                continue
            body = _brace_inner(code[m.end() :])
            if _is_marked_unused(name, body) and not _uses(name, body):
                violations.append(
                    Violation(
                        REPO_ROOT / rel,
                        lineno,
                        RULE,
                        f"this newly registered gui_test case marks "
                        f"`{name}` IM_UNUSED and never mentions it again, so it "
                        f"does not drive the GUI. " + FIX_HINT,
                    )
                )
    return violations, scanned


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
    args = parser.parse_args()

    # Presence, not truthiness: an empty --range is a malformed range, and
    # testing the string itself would quietly demote it to a staged scan.
    if args.rng is not None:
        if ".." not in args.rng:
            parser.error("--range expects BASE..HEAD")
        # The three-dot form is rejected rather than passed through. Splitting it
        # on the first ".." silently yields a head of ".HEAD", and git's eventual
        # complaint about that is not one anybody can act on. Its meaning is also
        # not this tool's: A...B asks about the merge base, which the caller is
        # better placed to resolve into the two-dot range it wants scanned.
        if "..." in args.rng:
            parser.error(
                "--range expects the two-dot form BASE..HEAD; for merge-base "
                "semantics pass `$(git merge-base A B)..B` explicitly"
            )
        base, head = args.rng.split("..", 1)
        if not base or not head:
            parser.error("--range expects BASE..HEAD")
        diff_args, content_ref = [base, head], head
    else:
        # --staged, passed or defaulted to. Empty content_ref means the index.
        diff_args, content_ref = ["--cached"], ""

    violations, scanned = check(diff_args, content_ref)
    if violations:
        print("New-gui-test check FAILED:\n", file=sys.stderr)
        for v in sorted(violations, key=lambda x: (str(x.path), x.line)):
            print(v.render(), file=sys.stderr)
        print(
            f"\n{len(violations)} newly registered gui_test case(s) do not use "
            "their ImGuiTestContext. See the rule in AGENTS.md; pre-existing "
            "cases are out of scope.",
            file=sys.stderr,
        )
        return 1
    print(f"New-gui-test check passed ({scanned} newly registered case(s) scanned).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
