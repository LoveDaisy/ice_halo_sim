"""Regression net for `scripts/check_new_gui_tests.py`.

The gate rejects a newly registered gui_test case that cannot drive the GUI, and
it fails in exactly the silent direction its sibling nets were written for: a
signature regex that stops matching reports success, a brace scan that runs past
its closing brace reports success, and a diff parser that attributes lines to the
wrong file reports success. None of those crash. The gate stays green while
enforcing nothing — which is the failure a gate is least able to reveal about
itself, and the reason this file exists rather than more careful reading.

Two kinds of case are pinned below and they are not interchangeable. The invented
ones state one mechanism each in isolation, so a failure names its own cause. The
last one takes a real tree file whose exact shape once made the narrow reading of
this rule misfire, and runs the gate over it as though every line were new; it is
the only test here that can catch a divergence between what the rule is believed
to do and what real GUI test code looks like.

Invented payloads use `Widget`/`Scenario`-style identifiers. The gate reads C++
structure rather than prose, so these are out of its siblings' scope by
construction, but they are kept fictional anyway.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "scripts"))

import check_new_gui_tests  # noqa: E402
import check_new_refs  # noqa: E402
import check_policies  # noqa: E402

GUI_TEST_FILE = "test/gui/functional/test_gui_example.cpp"

HEADER = """#include "gui_test_common.hpp"

namespace lumice::test {

void RegisterExampleTests(ImGuiTestEngine* engine) {
"""

FOOTER = """}

}  // namespace lumice::test
"""

# HEADER is five lines and each case() opens with its own `  {`, so the first
# signature lands on line 8. Spelled out rather than derived from the fixture
# text so the expectation stays an independent oracle: a helper that searched for
# "TestFunc" would locate it the same way the checker does, and agree with it
# even when both were wrong.
FIRST_SIG_LINE = 8


def case(name: str, signature: str, body: str) -> str:
    """One IM_REGISTER_TEST block, in the shape the real suites use.

    The receiver is `t` in every case, scoped by its own braces, because that is
    what the real files do — and it is load-bearing for the aliasing test below.
    Naming the variable after the test would make every signature line unique,
    which is exactly the property real code does not have and the property whose
    absence lets a newly written signature be paired with a deleted one.
    """
    return (
        "  {\n"
        f'    ImGuiTest* t = IM_REGISTER_TEST(engine, "example", "{name}");\n'
        f"    t->TestFunc = {signature} {{\n{body}    }};\n  }}\n\n"
    )


def source(*cases: str) -> str:
    return HEADER + "".join(cases) + FOOTER


class Repo:
    """A throwaway git repo the checker is pointed at via its REPO_ROOT globals."""

    def __init__(self, root: Path):
        self.root = root

    def _git(self, *args: str) -> str:
        return subprocess.run(
            ["git", *args], cwd=self.root, capture_output=True, check=True, text=True
        ).stdout

    def write(self, rel: str, text: str) -> None:
        p = self.root / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")

    def commit(self, msg: str = "wip") -> str:
        self._git("add", "-A")
        self._git("commit", "-q", "-m", msg)
        return self._git("rev-parse", "HEAD").strip()

    def stage(self) -> None:
        self._git("add", "-A")

    def run(self) -> tuple[list[check_policies.Violation], int]:
        """Run the gate over the staged diff, as the pre-commit hook does."""
        self.stage()
        return check_new_gui_tests.check(["--cached"], "", "HEAD")

    def run_range(self, base: str) -> tuple[list[check_policies.Violation], int]:
        """Run the gate over a commit range, as CI does."""
        return check_new_gui_tests.check([base, "HEAD"], "HEAD", base)

    def hits(self) -> list[tuple[str, int]]:
        violations, _ = self.run()
        return sorted((str(v.path.relative_to(self.root)), v.line) for v in violations)

    def range_hits(self, base: str) -> list[tuple[str, int]]:
        violations, _ = self.run_range(base)
        return sorted((str(v.path.relative_to(self.root)), v.line) for v in violations)

    def scanned(self) -> int:
        return self.run()[1]


@pytest.fixture
def repo(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Repo:
    r = Repo(tmp_path)
    r._git("init", "-q", "-b", "main")
    r._git("config", "user.email", "t@example.com")
    r._git("config", "user.name", "T")
    # All three bindings. The git helpers and the blob reader resolve the copy
    # imported into check_new_refs; Violation construction resolves the copy
    # imported into check_new_gui_tests; Violation.render() resolves the name in
    # the module that defines it.
    monkeypatch.setattr(check_new_refs, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(check_new_gui_tests, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(check_policies, "REPO_ROOT", tmp_path)
    r.write("seed.txt", "seed\n")
    r.commit("seed")
    return r


# --- the two rejected shapes -------------------------------------------------


def test_anonymous_parameter_is_rejected(repo: Repo) -> None:
    """Shape 1: the compiler's own statement that the case cannot touch the GUI.

    An unnamed parameter is unreferenceable, so no reading of the body can
    overturn it — which is why this shape needs no body analysis at all.
    """
    repo.write(
        GUI_TEST_FILE,
        source(case("t1", "[](ImGuiTestContext*)", "    IM_CHECK(1 + 1 == 2);\n")),
    )
    assert repo.hits() == [(GUI_TEST_FILE, FIRST_SIG_LINE)]


def test_commented_out_parameter_name_is_rejected(repo: Repo) -> None:
    """`/*ctx*/` is the anonymous shape wearing a name, and must read as anonymous.

    It is unreferenceable for the same reason as the bare form. This is not a
    separate branch in the checker: comments are blanked before matching, so the
    signature reduces to shape 1. The test pins that reduction, because the
    obvious alternative implementation — matching the raw text — would let this
    spelling through while every other test here still passed.
    """
    repo.write(
        GUI_TEST_FILE,
        source(case("t1", "[](ImGuiTestContext* /*ctx*/)", "    IM_CHECK(1 + 1 == 2);\n")),
    )
    assert repo.hits() == [(GUI_TEST_FILE, FIRST_SIG_LINE)]


def test_im_unused_with_no_other_mention_is_rejected(repo: Repo) -> None:
    """Shape 2: the author's own statement, written by hand, that ctx is not used."""
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* ctx)",
                "    IM_UNUSED(ctx);\n    IM_CHECK(1 + 1 == 2);\n",
            )
        ),
    )
    assert repo.hits() == [(GUI_TEST_FILE, FIRST_SIG_LINE)]


def test_parameter_name_is_read_from_the_signature(repo: Repo) -> None:
    """The name is captured, not assumed to be `ctx`.

    Every case in the tree today names it `ctx`, so a hardcoded literal would
    pass the whole suite and then quietly stop enforcing anything the day someone
    renamed the parameter.
    """
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* context)",
                "    IM_UNUSED(context);\n    IM_CHECK(1 + 1 == 2);\n",
            )
        ),
    )
    assert repo.hits() == [(GUI_TEST_FILE, FIRST_SIG_LINE)]


# --- what must NOT be rejected -----------------------------------------------


def test_im_unused_but_still_driving_ctx_is_accepted(repo: Repo) -> None:
    """The false positive this rule was nearly built with.

    `IM_UNUSED(ctx)` opening a body does not mean the body is done with ctx —
    real cases mark it and then use `ctx->Yield()` further down as a frame pump.
    A checker that stopped at the marker would reject working GUI tests, and a
    gate that rejects correct code gets bypassed, which is worse than no gate.
    """
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* ctx)",
                "    IM_UNUSED(ctx);\n"
                "    IM_CHECK(1 + 1 == 2);\n"
                "    ctx->Yield();\n",
            )
        ),
    )
    assert repo.hits() == []


def test_ctx_passed_bare_to_a_helper_is_accepted(repo: Repo) -> None:
    """`Helper(ctx, ...)` is a use, even though it never writes `->`.

    This is why the predicate is "mentions the name" rather than "calls name->".
    Over the current tree the narrow reading misjudges 17 cases of this exact
    shape as unused, against 4 that genuinely never use the parameter.
    """
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* ctx)",
                "    IM_UNUSED(ctx);\n    RunWidgetScenario(ctx, 3);\n",
            )
        ),
    )
    assert repo.hits() == []


def test_ordinary_case_driving_ctx_is_accepted(repo: Repo) -> None:
    """The overwhelmingly common shape: named, unmarked, actually used."""
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* ctx)",
                '    ctx->SetRef("Example");\n    ctx->ItemClick("Apply");\n',
            )
        ),
    )
    assert repo.hits() == []


def test_mention_in_a_comment_does_not_count_as_use(repo: Repo) -> None:
    """Prose about ctx is not a use of ctx.

    Comments are blanked before the body is read, so a body whose only remaining
    occurrence of the name sits in `// ctx is unnecessary here` still reports the
    parameter as unused. Without that blanking the rule would be trivially
    defeated by the very comment an author writes to explain the omission.
    """
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* ctx)",
                "    IM_UNUSED(ctx);  // ctx is unnecessary: no frame is rendered\n"
                "    IM_CHECK(1 + 1 == 2);\n",
            )
        ),
    )
    assert repo.hits() == [(GUI_TEST_FILE, FIRST_SIG_LINE)]


def test_unmarked_never_used_parameter_is_accepted(repo: Repo) -> None:
    """The documented gap, pinned so it stays a decision rather than an accident.

    A named parameter that is never referenced and never marked IM_UNUSED
    compiles (the build is -Wall -Wextra without -Werror) and four such cases
    exist in the tree. Catching it needs an inference with no compiler or author
    statement behind it, which is the class of predicate that makes gates
    misfire. If that trade is ever revisited, this test is what changes — and it
    fails loudly rather than silently widening.
    """
    repo.write(
        GUI_TEST_FILE,
        source(case("t1", "[](ImGuiTestContext* ctx)", "    IM_CHECK(1 + 1 == 2);\n")),
    )
    assert repo.hits() == []


def test_named_function_testfunc_is_accepted(repo: Repo) -> None:
    """The other documented gap: `TestFunc = SomeFunction;` has no signature here.

    Resolving the symbol to its definition is a compiler's job. Pinned for the
    same reason as above — so the gap is legible, not discovered.
    """
    repo.write(
        GUI_TEST_FILE,
        HEADER
        + '  ImGuiTest* t1 = IM_REGISTER_TEST(engine, "example", "t1");\n'
        + "  t1->TestFunc = RunExampleTest;\n\n"
        + FOOTER,
    )
    assert repo.hits() == []


# --- scoping: which lines, and which paths -----------------------------------


def test_pre_existing_case_is_out_of_scope(repo: Repo) -> None:
    """The gate reads added lines only, so the existing body never blocks anyone.

    26 cases in the tree match a rejected shape today. A whole-tree reading could
    not start green, and the alternative — a frozen baseline — is an asset
    someone has to keep correct forever. Scanning the diff starts green for free
    and matches how the rule actually gets broken.
    """
    violating = source(case("t1", "[](ImGuiTestContext*)", "    IM_CHECK(1 + 1 == 2);\n"))
    repo.write(GUI_TEST_FILE, violating)
    repo.commit("pre-existing")
    repo.write("unrelated.txt", "touched\n")
    assert repo.hits() == []
    assert repo.scanned() == 0


def test_editing_around_a_pre_existing_case_does_not_bill_it(repo: Repo) -> None:
    """Touching a file that holds a rejected shape is not the same as adding one.

    The anchor is the signature line itself, so an edit elsewhere in the file
    leaves the old case unbilled. Without this the gate would demand unrelated
    cleanup as the price of any edit to these files — the burden it is scoped to
    avoid.
    """
    violating = source(case("t1", "[](ImGuiTestContext*)", "    IM_CHECK(1 + 1 == 2);\n"))
    repo.write(GUI_TEST_FILE, violating)
    repo.commit("pre-existing")
    repo.write(GUI_TEST_FILE, violating.replace("IM_CHECK(1 + 1 == 2);", "IM_CHECK(2 + 2 == 4);"))
    assert repo.hits() == []


def test_rule_is_confined_to_test_gui(repo: Repo) -> None:
    """Only the display-requiring target is in scope.

    gui_unit_test's sources do not use IM_REGISTER_TEST at all, so nothing there
    can match today — which is precisely why the path prefix has to be pinned by
    a test. A widening would be invisible until it started rejecting code in a
    tree this rule was never argued for.
    """
    outside = "test/unit-correctness/gui/test_gui_unit_example.cpp"
    repo.write(outside, source(case("t1", "[](ImGuiTestContext*)", "    IM_CHECK(1 == 1);\n")))
    assert repo.hits() == []


def test_second_case_in_the_same_file_is_reported_separately(repo: Repo) -> None:
    """Body extraction must stop at its own closing brace.

    A brace scan that overran would swallow the following case, and the two would
    collapse into one report — or, worse, the first case would be cleared by the
    second one's use of ctx.
    """
    repo.write(
        GUI_TEST_FILE,
        source(
            case("t1", "[](ImGuiTestContext*)", "    IM_CHECK(1 + 1 == 2);\n"),
            case("t2", '[](ImGuiTestContext* ctx)', '    ctx->SetRef("Example");\n'),
            case(
                "t3",
                "[](ImGuiTestContext* ctx)",
                "    IM_UNUSED(ctx);\n    IM_CHECK(3 + 3 == 6);\n",
            ),
        ),
    )
    # Each single-statement case() block spans seven lines, so the third case's
    # signature sits fourteen lines below the first.
    assert repo.hits() == [(GUI_TEST_FILE, FIRST_SIG_LINE), (GUI_TEST_FILE, FIRST_SIG_LINE + 14)]


def test_nested_braces_in_the_body_do_not_end_it_early(repo: Repo) -> None:
    """A block or initializer inside the body is not its closing brace.

    A naive first-`}` scan would cut the body at the inner block and miss the
    `ctx->Yield()` beyond it, turning a working GUI test into a rejection.
    """
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* ctx)",
                "    IM_UNUSED(ctx);\n"
                "    for (int i = 0; i < 2; ++i) {\n"
                "      IM_CHECK(i < 2);\n"
                "    }\n"
                "    ctx->Yield();\n",
            )
        ),
    )
    assert repo.hits() == []


# --- "new" is an identity question, not a line-number one --------------------
#
# Both tests below describe the same commit shape — some cases deleted, the file
# rewritten around what remains — because that is what a migration looks like and
# it is the shape this gate will meet most often. Read line-wise it defeats the
# gate in both directions at once, so each direction gets its own test.


def test_new_case_is_caught_even_when_its_signature_line_aliases_a_deleted_one(
    repo: Repo,
) -> None:
    """A newly added case must be caught even if git never reports its line as added.

    The rejected signatures are boilerplate: every anonymous one in the tree is
    the same 44 characters. So when a change deletes cases carrying that exact
    line and adds a new one, git's line matching pairs the new line with a
    deleted one and the added-line set never mentions it. The gate then reports
    success on a genuinely violating change — which is how it behaved against
    real history before identity became the predicate.

    This is the single most important test in the file: it is the one that fails
    if the gate reverts to asking where a line is rather than whether a case is
    new.
    """
    kept = case("t_kept", '[](ImGuiTestContext* ctx)', '      ctx->SetRef("Example");\n')
    repo.write(
        GUI_TEST_FILE,
        source(
            kept,
            case("t_gone_a", "[](ImGuiTestContext*)", "      IM_CHECK(1 + 1 == 2);\n"),
            case("t_gone_b", "[](ImGuiTestContext*)", "      IM_CHECK(2 + 2 == 4);\n"),
        ),
    )
    base = repo.commit("existing suite")
    # Delete the two anonymous cases and register one more, exactly as a
    # migration-plus-one-new-test commit does.
    repo.write(
        GUI_TEST_FILE,
        source(kept, case("t_new", "[](ImGuiTestContext*)", "      IM_CHECK(4 + 4 == 8);\n")),
    )
    repo.commit("migrate two out, add one")

    added = check_new_refs.added_lines([base, "HEAD"]).get(GUI_TEST_FILE, set())
    sig_line = FIRST_SIG_LINE + 7
    assert sig_line not in added, (
        "fixture no longer reproduces the aliasing it exists to pin: git now "
        "reports the new signature line as added, so this test would pass even "
        "with a line-based predicate"
    )
    assert repo.range_hits(base) == [(GUI_TEST_FILE, sig_line)]


def test_untouched_case_is_not_billed_when_a_sibling_case_is_deleted(repo: Repo) -> None:
    """...and the repair for the above must not bill cases that merely shifted.

    The IM_REGISTER_TEST line is the one line in a case that cannot alias,
    because it carries the case's name. That makes it the obvious anchor and the
    wrong one: when a neighbouring case is deleted, git aligns the identical
    boilerplate across the two bodies and reports the name-bearing line as the
    only change, so an untouched case reads as newly registered. Two real cases
    were billed that way before identity became the predicate.
    """
    kept = case(
        "t2", "[](ImGuiTestContext* ctx)", "    IM_UNUSED(ctx);\n    ResetTestState();\n"
    )
    repo.write(
        GUI_TEST_FILE,
        source(
            case(
                "t1",
                "[](ImGuiTestContext* ctx)",
                "    IM_UNUSED(ctx);\n    ResetTestState();\n",
            ),
            kept,
        ),
    )
    base = repo.commit("two cases sharing a body shape")
    repo.write(GUI_TEST_FILE, source(kept))  # delete the first; the second shifts up
    repo.commit("delete the first case")
    assert repo.range_hits(base) == []


def test_case_moved_between_gui_files_is_not_billed(repo: Repo) -> None:
    """Identity is looked up across test/gui/, not per file.

    A case relocated to another suite file is not a case someone registered, so
    scoping the lookup to the file it now lives in would bill every such move —
    and moving cases around is what maintaining this tree consists of.
    """
    other = "test/gui/functional/test_gui_other.cpp"
    moved = case("t1", "[](ImGuiTestContext*)", "    IM_CHECK(1 + 1 == 2);\n")
    repo.write(GUI_TEST_FILE, source(moved))
    base = repo.commit("case lives here")
    repo.write(GUI_TEST_FILE, source())
    repo.write(other, source(moved))
    repo.commit("move it there")
    assert repo.range_hits(base) == []


def test_registration_split_across_lines_still_counts_as_pre_existing(repo: Repo) -> None:
    """A long test name pushes IM_REGISTER_TEST onto its own line; it still has identity.

    The "did this exist before" lookup reads `git grep` output, which yields the
    matching line alone — without the `ImGuiTest* t =` that sits on the line
    above. A pattern requiring that binding silently drops these cases from the
    "existed before" set, and a case missing from that set reads as newly
    registered. The failure is therefore towards blocking correct code, and it is
    invisible: nothing about the wrapped registration looks different.
    """
    wrapped = (
        '  ImGuiTest* t_with_a_rather_long_descriptive_name =\n'
        '      IM_REGISTER_TEST(engine, "example", "a_rather_long_descriptive_case_name");\n'
        "  t_with_a_rather_long_descriptive_name->TestFunc = [](ImGuiTestContext*) {\n"
        "    IM_CHECK(1 + 1 == 2);\n  };\n\n"
    )
    repo.write(GUI_TEST_FILE, HEADER + wrapped + FOOTER)
    base = repo.commit("pre-existing wrapped registration")
    repo.write(GUI_TEST_FILE, HEADER + wrapped + "  // an unrelated edit\n" + FOOTER)
    repo.commit("touch the file")
    assert repo.range_hits(base) == []


def test_registration_call_split_across_lines_still_counts_as_pre_existing(repo: Repo) -> None:
    """The IM_REGISTER_TEST call's own arguments wrapping is the same trap as above.

    The previous test wraps the line ABOVE the call (the `ImGuiTest* t =`
    binding). This one wraps the call itself — `IM_REGISTER_TEST(` on one line,
    `engine, "example", "name")` on the next. `git grep` still yields only the
    single matching physical line, and that line alone never contains a complete
    category/name pair, so the same silent drop-from-"existed before" failure
    applies to a shape the line-above variant does not exercise.
    """
    wrapped = (
        "  ImGuiTest* t = IM_REGISTER_TEST(\n"
        '      engine, "example", "a_case_whose_call_wraps");\n'
        "  t->TestFunc = [](ImGuiTestContext*) {\n"
        "    IM_CHECK(1 + 1 == 2);\n  };\n\n"
    )
    repo.write(GUI_TEST_FILE, HEADER + wrapped + FOOTER)
    base = repo.commit("pre-existing call-wrapped registration")
    repo.write(GUI_TEST_FILE, HEADER + wrapped + "  // an unrelated edit\n" + FOOTER)
    repo.commit("touch the file")
    assert repo.range_hits(base) == []


def test_signature_rewritten_in_place_is_billed(repo: Repo) -> None:
    """Identity alone would miss an existing case edited into a rejected shape.

    Deleting the parameter name from a case that used to drive the GUI registers
    nothing new, so the identity predicate says nothing. The added-line anchor is
    kept for exactly this, and it is safe here in a way it is not as the primary
    predicate: a signature line reported as added belongs to a case that was
    either just written or just edited, and both are this diff's to answer for.
    """
    repo.write(
        GUI_TEST_FILE,
        source(case("t1", '[](ImGuiTestContext* ctx)', '    ctx->SetRef("Example");\n')),
    )
    base = repo.commit("a case that drives the GUI")
    repo.write(
        GUI_TEST_FILE,
        source(case("t1", "[](ImGuiTestContext*)", "    IM_CHECK(1 + 1 == 2);\n")),
    )
    repo.commit("gut it")
    assert repo.range_hits(base) == [(GUI_TEST_FILE, FIRST_SIG_LINE)]


# --- why there is no real-tree case here anymore ------------------------------
#
# This file used to end with one more test: it copied a real gui_test suite in
# whole, billed every line as added, and asserted the rule stayed quiet. Its
# value over the invented payloads above was calibration — those say what
# someone believed the rule should see, that one said what the rule actually had
# to live with.
#
# The suite it pointed at held the shape that makes the naive reading fail: a
# case marking `IM_UNUSED(ctx)` and then still using ctx as a frame pump. That
# shape is no longer anywhere under test/gui, and its absence is not a defect
# being fixed — the layer's definition now is "a case here must need a real
# frame or real input", so every case drives ctx and nobody reaches for the
# mark-unused idiom. With no instance left in the tree there is nothing to
# calibrate against, so the test was removed rather than re-pointed at a file
# that does not have the shape.
#
# The shape itself stays covered, from both sides, by the pair above:
#   test_im_unused_with_no_other_mention_is_rejected  (marked, truly unused -> hit)
#   test_im_unused_but_still_driving_ctx_is_accepted  (marked, still used  -> no hit)
# Deleting the second would let a marker-only regression pass the first unseen.
#
# If the idiom ever comes back into test/gui, re-add a real-tree case anchored
# on the file that holds it; that is a stronger check than any payload here.
