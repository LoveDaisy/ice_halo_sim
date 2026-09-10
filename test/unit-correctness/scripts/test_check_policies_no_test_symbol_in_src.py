"""Regression net for `check_no_test_symbol_in_src` in `scripts/check_policies.py`.

The rule is one prefix regex run line-by-line over `code_lines()`, so by the
criterion at the top of `test_check_new_refs.py` it is on the untested side.
It is pinned anyway, and for the same reason `test_check_policies_bare_print.py`
is: the rule's interesting behaviour lives in a helper it borrows rather than in
its own text. That a comment naming `LUMICE_TEST_` (the rule's own explanation,
a header note pointing at the test surface) is not a hit comes entirely from
`strip_comments()`; a change made for another check's benefit reaches here
silently, and a false positive on the prose that documents the rule would be
the first thing a reader saw.

The other reason is the one the issue named: a policy gate's red state has to be
demonstrated, not assumed. The temporary "write a `LUMICE_TEST_` line into
lumice.h and watch the checker go red" probe is done once by hand and then
undone; this file is that probe made permanent.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import check_policies  # noqa: E402

RULE = "no-test-symbol-in-src"


@pytest.fixture
def src_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    src = tmp_path / "src"
    src.mkdir()
    monkeypatch.setattr(check_policies, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(check_policies, "SRC", src)
    return src


def _violations(src_root: Path, body: str, name: str = "scratch.cpp") -> list:
    path = src_root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(body, encoding="utf-8")
    return check_policies.check_no_test_symbol_in_src()


# --- must stay red: the test surface leaking into src/ ----------------------


def test_declaration_in_public_header_is_flagged(src_root: Path) -> None:
    """The exact shape the rule exists for: a hook declared in lumice.h."""
    out = _violations(
        src_root,
        "#ifndef LUMICE_H_\nvoid LUMICE_TEST_ComputeRenderDomainMask(void);\n#endif\n",
        name="include/lumice.h",
    )
    assert len(out) == 1
    assert out[0].rule == RULE
    assert out[0].line == 2


def test_call_in_implementation_is_flagged(src_root: Path) -> None:
    out = _violations(src_root, "void F() {\n  LUMICE_TEST_ReleaseRenderDomainMask(0);\n}\n")
    assert len(out) == 1


def test_struct_tag_is_flagged(src_root: Path) -> None:
    """The prefix, not a function shape: a mirrored struct is as much a leak as a call."""
    out = _violations(src_root, "struct LUMICE_TEST_RenderDomainMask_ {\n  int width;\n};\n")
    assert len(out) == 1


def test_every_hit_line_is_reported(src_root: Path) -> None:
    out = _violations(
        src_root,
        "int a = LUMICE_TEST_A;\nint b = 1;\nint c = LUMICE_TEST_C;\n",
    )
    assert [v.line for v in out] == [1, 3]


# --- must stay green: prose about the rule is not a violation of it --------


def test_prefix_in_comment_is_not_flagged(src_root: Path) -> None:
    """Comes from `strip_comments()`, which this check borrows and does not own."""
    out = _violations(
        src_root,
        "// LUMICE_TEST_* hooks live in test/support/lumice_test_api.h, never here.\n"
        "/* see LUMICE_TEST_ComputeRenderDomainMask */\n"
        "int x = 0;\n",
    )
    assert out == []


def test_product_prefix_is_not_flagged(src_root: Path) -> None:
    """LUMICE_* without the TEST_ segment is the product surface and is the point."""
    out = _violations(
        src_root,
        "LUMICE_ErrorCode LUMICE_ComputeAnnotationOverlay(const LUMICE_AnnotationRequest* r,\n"
        "                                                 LUMICE_AnnotationOverlay* o);\n"
        "#define LUMICE_TESTING_MODE 1\n",
    )
    assert out == []


def test_prefix_inside_a_longer_identifier_is_not_flagged(src_root: Path) -> None:
    """`\\b` ahead of the prefix: MY_LUMICE_TEST_X is not the test surface's spelling."""
    assert _violations(src_root, "int MY_LUMICE_TEST_X = 0;\n") == []


# --- scope: which files are read at all -------------------------------------


@pytest.mark.parametrize("suffix", sorted(check_policies.CXX_SUFFIXES))
def test_every_cxx_suffix_is_scanned(src_root: Path, suffix: str) -> None:
    out = _violations(src_root, "int v = LUMICE_TEST_V;\n", name=f"leak{suffix}")
    assert len(out) == 1, f"{suffix} not scanned"


def test_non_source_suffix_is_not_scanned(src_root: Path) -> None:
    assert _violations(src_root, "LUMICE_TEST_ is documented here\n", name="notes.md") == []


def test_files_outside_src_are_not_scanned(src_root: Path) -> None:
    """The test surface's own header must not trip the rule that protects it."""
    outside = src_root.parent / "test" / "support"
    outside.mkdir(parents=True)
    (outside / "lumice_test_api.h").write_text(
        "void LUMICE_TEST_ComputeRenderDomainMask(void);\n", encoding="utf-8"
    )
    assert check_policies.check_no_test_symbol_in_src() == []


def test_check_is_registered_in_checks(src_root: Path) -> None:
    assert check_policies.check_no_test_symbol_in_src in check_policies.CHECKS
