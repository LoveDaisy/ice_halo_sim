"""Regression net for `check_pytest_invocation_marker` in `scripts/check_policies.py`.

Most of `check_policies.py` ships without tests on purpose (see the note at the
top of `test_check_new_refs.py`): a check that asks "is this symbol present in
this file?" is legible by reading. This one is not. It runs three different
extractors — CMake paren-depth blocks, backslash-continued shell lines, and
Markdown fence state — and then applies a two-part predicate (`.py` present,
`-m` absent) to a *slice* of each command rather than the whole text. Every one
of those pieces fails silently in the same direction: the gate stays green while
scanning nothing, or while reading a launcher's `-m` as the caller's.

That is not hypothetical. The plan for this rule specified searching the whole
command span for `-m`; implementing it that way passes the whole-tree run and
looks clean, yet misses *both* of the historical incidents the rule exists to
catch, because `python3 -m pytest` carries a literal `-m` of its own. The tests
named `..._launcher_...` below pin that specific hazard.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import check_policies  # noqa: E402

RULE = "pytest-invocation-marker"


@pytest.fixture
def tree(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    """A scratch repo with every scan root of the rule retargeted at it."""
    (tmp_path / "test").mkdir()
    (tmp_path / "scripts").mkdir()
    (tmp_path / "doc").mkdir()
    monkeypatch.setattr(check_policies, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(check_policies, "PYTEST_SCAN_CMAKE", tmp_path / "test" / "CMakeLists.txt")
    monkeypatch.setattr(check_policies, "PYTEST_SCAN_SHELL_DIR", tmp_path / "scripts")
    monkeypatch.setattr(check_policies, "PYTEST_SCAN_MD_DIRS", (tmp_path / "doc",))
    monkeypatch.setattr(check_policies, "PYTEST_SCAN_MD_FILES", (tmp_path / "AGENTS.md",))
    monkeypatch.setattr(
        check_policies,
        "PYTEST_INVOCATION_MARKER_ALLOWED",
        frozenset({tmp_path / "scripts" / "verify_crash_sentinel_detection_power.sh"}),
    )
    return tmp_path


def _cmake(tree: Path, body: str) -> list[check_policies.Violation]:
    (tree / "test" / "CMakeLists.txt").write_text(body, encoding="utf-8")
    return check_policies.check_pytest_invocation_marker()


def _shell(tree: Path, body: str, name: str = "run.sh") -> list[check_policies.Violation]:
    (tree / "scripts" / name).write_text(body, encoding="utf-8")
    return check_policies.check_pytest_invocation_marker()


def _md(tree: Path, body: str, name: str = "guide.md") -> list[check_policies.Violation]:
    (tree / "doc" / name).write_text(body, encoding="utf-8")
    return check_policies.check_pytest_invocation_marker()


# --- must stay red: a .py target with no -m of its own ----------------------


def test_cmake_add_test_without_marker_is_flagged(tree: Path) -> None:
    """Verbatim shape of test/CMakeLists.txt before 5164fd26: the target sits on
    its own continuation line, so only block-level extraction can see it next to
    the command."""
    out = _cmake(
        tree,
        'find_program(PYTEST_EXECUTABLE NAMES pytest)\n'
        'add_test(NAME "CudaMultiMsParity"\n'
        "  COMMAND ${PYTEST_EXECUTABLE} -v\n"
        '    "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_multi_ms_parity.py"\n'
        '  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")\n',
    )
    assert len(out) == 1
    assert out[0].rule == RULE
    assert out[0].line == 2  # reported at the add_test( line, not the target line


def test_shell_bare_pytest_with_py_target_is_flagged(tree: Path) -> None:
    out = _shell(tree, 'pytest -v "test/regression-sentinel/test_something.py"\n')
    assert len(out) == 1
    assert out[0].rule == RULE


def test_markdown_launcher_dash_m_does_not_count_as_the_callers_marker(tree: Path) -> None:
    """The pre-7fbe01d7 CUDA parity recipe, verbatim (brace expansion included).

    `python3 -m pytest` contains a literal `-m`. Reading that as "the caller
    passed a marker" is the single mistake that would make this whole rule miss
    the incident it was written for, and it leaves no trace: the tree stays
    green. Brace expansion needs no expanding — the literal already ends `.py`.
    """
    out = _md(
        tree,
        "Recipe:\n\n"
        "```bash\n"
        "python3 -m pytest -v "
        "test/parity-cross-backend/backend/test_cuda_{exit_seam,filter,multi_ms}_parity.py\n"
        "```\n",
    )
    assert len(out) == 1
    assert out[0].rule == RULE


def test_markdown_windows_launcher_form_is_flagged(tree: Path) -> None:
    """Same hazard through the win-builder spelling: an absolute `python.exe -m
    pytest`, backslash path separators, several targets space-separated."""
    out = _md(
        tree,
        "```bat\n"
        r"C:\lumice-test\py311\python.exe -m pytest -v "
        r"test\backend\test_a_parity.py test\backend\test_b_parity.py"
        "\n```\n",
    )
    assert len(out) == 1


def test_shell_fused_launcher_short_option_is_flagged(tree: Path) -> None:
    r"""`python -mpytest` — no whitespace between `-m` and `pytest`.

    CPython accepts the fused short option, so this launches pytest exactly like
    `python -m pytest` does. It used to match NO alternative at all: `\bpytest\b`
    cannot match inside `-mpytest` (the preceding `m` is a word char, so there is
    no boundary there), and the launcher branch required whitespace. The call
    site was therefore not scanned — a false negative that no whole-tree green
    can surface, since a miss is indistinguishable from a clean tree.
    """
    out = _shell(tree, "python -mpytest test/regression-sentinel/test_x.py\n")
    assert len(out) == 1
    assert out[0].rule == RULE


def test_fused_launcher_still_honours_the_callers_marker(tree: Path) -> None:
    """The fused spelling must not become a blanket violation: carrying its own
    `-m`, it is compliant exactly like the spaced spelling. Pins that widening
    the launcher branch did not also swallow the caller's marker."""
    out = _shell(tree, "python3 -mpytest -m slow test/regression-sentinel/test_x.py\n")
    assert out == []


@pytest.mark.parametrize("module", ["notpytest", "pytestx"])
def test_fused_launcher_does_not_swallow_a_similar_module_name(tree: Path, module: str) -> None:
    r"""The counter-example the widening rests on, pinned rather than reasoned.

    Dropping the whitespace requirement is only safe because the branch still
    demands the literal `pytest` bounded on the right: `-mnotpytest` fails it on
    the left (the `-m` is not followed by `pytest`) and `-mpytestx` on the right
    (`\b` needs a non-word char after `pytest`). Neither is a pytest run, so
    neither may be flagged. Without this case the two tests above are both
    pytest-POSITIVE, and an over-matching regex would keep them green.
    """
    assert _shell(tree, f"python -m{module} foo.py\n") == []


def test_shell_marker_on_a_continuation_line_is_seen(tree: Path) -> None:
    """`-m` and the `.py` target routinely land on different physical lines.
    Judging either line alone is wrong in both directions; this pins the folding.
    """
    out = _shell(
        tree,
        'pytest -v "test/regression-sentinel/test_something.py" \\\n  --tb=short\n',
    )
    assert len(out) == 1, "continuation line has no -m either: still a violation"

    out = _shell(
        tree,
        'pytest -v "test/regression-sentinel/test_something.py" \\\n  -m slow --tb=short\n',
        name="ok.sh",
    )
    assert [v for v in out if v.path.name == "ok.sh"] == []


def test_one_compliant_call_does_not_launder_a_bad_one(tree: Path) -> None:
    """Two invocations on one line: each owns only the text up to the next
    pytest token, so the trailing `-m slow` cannot cover for the first call."""
    out = _shell(tree, 'pytest a_test.py && pytest -m slow b_test.py\n')
    assert len(out) == 1


# --- must stay green --------------------------------------------------------


def test_cmake_add_test_with_marker_is_clean(tree: Path) -> None:
    """The post-5164fd26 shape that ships today."""
    assert (
        _cmake(
            tree,
            'add_test(NAME "CudaMultiMsParity"\n'
            "  COMMAND ${PYTEST_EXECUTABLE} -v -m slow\n"
            '    "${PROJ_TEST_DIR}/parity-cross-backend/backend/test_cuda_multi_ms_parity.py"\n'
            '  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")\n',
        )
        == []
    )


def test_shell_invocation_without_a_py_target_is_clean(tree: Path) -> None:
    """`scripts/test.sh` as it stands: no `.py` target, so addopts' fast-subset
    default is exactly what the caller wants. Flagging this would make the rule
    unlandable."""
    assert _shell(tree, "pytest -n auto\n") == []


def test_shell_directory_target_with_marker_is_clean(tree: Path) -> None:
    assert _shell(tree, 'pytest --ignore=test/performance -n 3 -m "slow and not heavy"\n') == []


def test_markdown_bare_pytest_lines_in_agents_md_are_clean(tree: Path) -> None:
    """AGENTS.md's command table: no `.py` targets. Note `pyproject.toml` in the
    trailing comment must not read as a `.py` path."""
    (tree / "AGENTS.md").write_text(
        "```bash\n"
        "pytest -v          # fast e2e only — pinned by pyproject.toml addopts\n"
        "pytest -v -m ''    # full e2e set\n"
        "pytest -v -m slow  # slow e2e only\n"
        "```\n",
        encoding="utf-8",
    )
    assert check_policies.check_pytest_invocation_marker() == []


def test_markdown_prose_outside_a_fence_is_not_scanned(tree: Path) -> None:
    """doc/ discusses pytest and this very addopts pin at length. Restricting to
    fenced blocks is what makes that a non-issue without one exclusion pattern;
    if the fence bound is ever dropped, this goes red."""
    assert (
        _md(
            tree,
            "Historically the recipe read `pytest -v test/backend/test_a_parity.py`, which\n"
            "silently collected zero tests after the addopts pin landed.\n",
        )
        == []
    )


# --- allowlist granularity --------------------------------------------------


def test_allowlisted_script_bare_invocation_is_clean(tree: Path) -> None:
    """These two scripts drive targets that are deliberately NOT slow-marked."""
    assert (
        _shell(
            tree,
            'pytest -v "test/regression-sentinel/test_face_distance_crash.py" --tb=short\n',
            name="verify_crash_sentinel_detection_power.sh",
        )
        == []
    )


def test_identical_content_under_another_filename_is_flagged(tree: Path) -> None:
    """Paired with the test above, and the pairing is the point: the exemption is
    keyed on the file's identity, not on what the command happens to look like.
    Byte-identical content, different name, must go red — otherwise someone has
    quietly turned a named exemption into a content heuristic."""
    out = _shell(
        tree,
        'pytest -v "test/regression-sentinel/test_face_distance_crash.py" --tb=short\n',
        name="not_the_allowlisted_one.sh",
    )
    assert len(out) == 1
    assert out[0].rule == RULE


# --- parse failure is observable, and is NOT a violation --------------------


def test_unterminated_fence_warns_without_failing(
    tree: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """An unterminated fence means the scanner may have read the file wrong.
    That is a different fact from "the repo has a violation" and gets a different
    exit: a stderr warning, no Violation, exit code untouched. Silence would let
    "the scanner quietly stopped scanning" persist forever — no whole-tree run
    and no fixture would ever surface it. Reporting a Violation would turn a
    prose-style change into a red gate. This pins the middle path against being
    collapsed either way.
    """
    out = _md(tree, "```bash\npytest -v test/backend/test_a_parity.py\n")
    assert out == []
    assert "unterminated fenced block" in capsys.readouterr().err


# --- the rule is registered, not merely defined ------------------------------


def test_check_is_registered_in_checks(tree: Path) -> None:
    """A rule that exists but is not in CHECKS never runs; nothing else here
    would notice."""
    assert check_policies.check_pytest_invocation_marker in check_policies.CHECKS


# --- line-continuation folding follows the shell, not intuition --------------


def test_continuation_folds_the_way_a_shell_folds_it() -> None:
    """`\\`+newline is *removed*, not turned into a space.

    `echo foo\\<nl>bar` prints `foobar` — that is the language these scanners
    read, so the fold has to match it. Joining with a space would read a
    dialect nobody writes and would split tokens the shell keeps contiguous.

    This is pinned because a reviewer proposed the space-join as a fix for the
    MISS 4 wording, on two counterexamples that are real behaviours but not
    real invocations (see the companion test below). Without a pin, the next
    reader has no way to tell the current form is a decision rather than a slip.
    """
    folded = check_policies._join_line_continuations("pytest foo.py \\\n-m slow\n")
    assert folded == [(1, "pytest foo.py -m slow")]

    tight = check_policies._join_line_continuations("pytest foo.py\\\n-m slow\n")
    assert tight == [(1, "pytest foo.py-m slow")]


def test_miss_4_is_a_broken_command_not_a_missed_invocation() -> None:
    """The MISS 4 blind spot only swallows input the shell cannot run either.

    `pytest\\<nl>foo.py` folds to `pytestfoo.py`: the token regex declines it,
    and so would the shell — there is no such executable. Pinning it keeps the
    documented miss honest; if someone ever makes this case flag, the rule has
    started reporting on commands that do not exist.
    """
    folded = check_policies._join_line_continuations("pytest\\\nfoo.py --tb=short\n")
    assert folded == [(1, "pytestfoo.py --tb=short")]
    assert check_policies._pytest_invocation_offenders(folded[0][1]) == []
