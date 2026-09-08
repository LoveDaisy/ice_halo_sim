"""Regression net for `check_no_render_in_benchmark_poll` in `scripts/check_policies.py`.

Read the note at the top of `test_check_new_refs.py` first: this repo does not
test symbol-matching checks, only the ones that would fail silently. This rule
qualifies because it owns a parser, and every way that parser can break fails
toward green.

The rule is not "does `LUMICE_AcquireResultFrame` appear in main.cpp" — the file
is full of legitimate calls to it, three of them, and one mention inside a
comment. It is "does it appear *between* the `void RunBenchmarkPass(` line and
the column-0 `}` that closes it". So the whole rule rests on finding two line
numbers, and a mistake in either direction is invisible: a start marker that
matches nothing yields an empty scan, an end marker found too early yields a
short one. Both print "Policy check passed".

The cases below therefore pin *where the window is*, not the wording of any
pattern: a call inside the function is caught, the same call before and after it
is not, and the mention that lives only in a comment is not. A rewrite that keeps
those four facts should keep this file green.

Not pinned, deliberately: the rule's own stated limitation, that renaming or
splitting `RunBenchmarkPass` silently disables it. There is no assertion that
would make that state red without also re-deciding what the function is called,
which is the judgement the rule declines to automate.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import check_policies  # noqa: E402

RULE = "no-render-in-benchmark-poll"


@pytest.fixture
def src_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    src = tmp_path / "src"
    src.mkdir()
    monkeypatch.setattr(check_policies, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(check_policies, "SRC", src)
    return src


def _violations(src_root: Path, body: str) -> list:
    (src_root / "main.cpp").write_text(body, encoding="utf-8")
    return check_policies.check_no_render_in_benchmark_poll()


def _main_cpp(before: str = "", inside: str = "", after: str = "") -> str:
    """A miniature main.cpp with the three regions the rule has to tell apart."""
    return (
        "namespace {\n"
        "\n"
        "void PrintStats(LUMICE_Server* server) {\n"
        f"{before}"
        "}\n"
        "\n"
        "void RunBenchmarkPass(const std::string& config_str, int num_workers) {\n"
        "  while (true) {\n"
        f"{inside}"
        "    LUMICE_GetSimRayCount(server, &cur_rays);\n"
        "  }\n"
        "}\n"
        "\n"
        "}  // namespace\n"
        "\n"
        "int main() {\n"
        f"{after}"
        "}\n"
    )


def test_clean_benchmark_pass_is_accepted(src_root: Path):
    assert _violations(src_root, _main_cpp()) == []


@pytest.mark.parametrize(
    "call",
    [
        "LUMICE_AcquireResultFrame(server, &raw_frame)",
        "SaveRenderResults(server, output_dir, image_format, jpeg_quality)",
        "SaveCompositeResults(server, output_dir, image_format, jpeg_quality)",
        "PrintStats(server)",
    ],
)
def test_render_trigger_inside_the_pass_is_caught(src_root: Path, call: str):
    """Each banned name is banned: the C API chokepoint and all three wrappers."""
    found = _violations(src_root, _main_cpp(inside=f"    {call};\n"))
    assert [v.rule for v in found] == [RULE], f"{call} was not reported"


def test_same_call_outside_the_pass_is_not_flagged(src_root: Path):
    """The window matters, not the file.

    `PrintStats` legitimately acquires a frame, and `main()` legitimately calls
    all three wrappers. A rule that fired on those would have to be turned off,
    which is the failure mode one step worse than not existing.
    """
    body = _main_cpp(
        before="  LUMICE_AcquireResultFrame(server, &raw_frame);\n",
        after="  SaveRenderResults(server, dir, fmt, q);\n  PrintStats(server);\n",
    )
    assert _violations(src_root, body) == []


def test_mention_in_a_comment_inside_the_pass_is_not_flagged(src_root: Path):
    """Comments are blanked before the scan.

    The constant declarations in the real main.cpp explain themselves by naming
    LUMICE_AcquireResultFrame, and the benchmark loop carries a comment saying
    precisely not to call it. A rule that read those as violations would punish
    the documentation that states it.
    """
    body = _main_cpp(inside="    // NOT LUMICE_AcquireResultFrame(server, &f) -- see above.\n")
    assert _violations(src_root, body) == []


def test_violation_points_at_the_offending_line(src_root: Path):
    """The reported line must be the call, not the function header.

    A rule that reports the window's start instead sends the reader to a
    signature that looks fine, which is how a real violation gets dismissed.
    """
    body = _main_cpp(inside="    PrintStats(server);\n")
    found = _violations(src_root, body)
    assert len(found) == 1
    reported = body.splitlines()[found[0].line - 1]
    assert "PrintStats(server);" in reported


def test_absent_main_cpp_is_not_an_error(src_root: Path):
    """An empty src/ yields no violations rather than an exception."""
    assert check_policies.check_no_render_in_benchmark_poll() == []
