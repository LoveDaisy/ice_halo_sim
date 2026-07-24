"""Regression net for `check_no_default_constructed_crystal_slots` in
`scripts/check_policies.py`.

The rest of `check_policies.py` ships without tests on purpose (see the note
in `test_check_new_refs.py`): most of its checks ask "is this symbol present
in this file?", which is legible by reading. This one check is not — it
parses call-site arguments (comma counting, paren depth, whitespace) to tell
a hazardous single-argument `resize`/`assign`/count-ctor from a safe
multi-argument or copy-ctor form, and across two rounds of code review that
parsing logic produced four distinct silent defects on the same small
surface: parameter-count-blind resize/assign, a copy-ctor false positive,
a function-prototype false positive, and a nested-call comma miscount (in
both directions — false negative for a single nested-call argument, false
positive for a real two-argument iterator-range ctor whose first argument
contains a nested call). Every one of those failed silently: the gate stayed
green while quietly rejecting safe code or admitting unsafe code, which is
exactly the failure this gate exists to prevent.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import check_policies  # noqa: E402


@pytest.fixture
def src_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    src = tmp_path / "src"
    src.mkdir()
    monkeypatch.setattr(check_policies, "SRC", src)
    return src


def _violations(src_root: Path, body: str) -> list[check_policies.Violation]:
    (src_root / "scratch.cpp").write_text(
        f"#include <vector>\n#include \"crystal.hpp\"\n\n{body}\n", encoding="utf-8"
    )
    return check_policies.check_no_default_constructed_crystal_slots()


# --- single-argument forms materialise default Crystals: must stay red -----


def test_single_arg_resize_is_flagged(src_root: Path) -> None:
    out = _violations(
        src_root,
        "void F() {\n"
        "  std::vector<Crystal> v;\n"
        "  v.resize(4);\n"
        "}\n",
    )
    assert len(out) == 1
    assert out[0].rule == "no-default-constructed-crystal-slots"


def test_single_arg_assign_is_flagged(src_root: Path) -> None:
    out = _violations(
        src_root,
        "void F() {\n"
        "  std::vector<Crystal> v;\n"
        "  v.assign(4);\n"
        "}\n",
    )
    assert len(out) == 1


def test_count_ctor_is_flagged(src_root: Path) -> None:
    out = _violations(src_root, "std::vector<Crystal> v(4);\n")
    assert len(out) == 1


def test_nested_call_as_sole_resize_argument_is_flagged(src_root: Path) -> None:
    """`resize(ComputeCount(w, h))` has one top-level argument even though a
    naive first-`)` scan would see a comma inside it and wave it through."""
    out = _violations(
        src_root,
        "void F() {\n"
        "  std::vector<Crystal> v;\n"
        "  v.resize(ComputeCount(w, h));\n"
        "}\n",
    )
    assert len(out) == 1


# --- multi-argument / copy / range forms fill with real values: must stay green


def test_two_arg_resize_is_not_flagged(src_root: Path) -> None:
    out = _violations(
        src_root,
        "void F() {\n"
        "  std::vector<Crystal> v;\n"
        "  v.resize(4, Crystal());\n"
        "}\n",
    )
    assert out == []


def test_two_arg_assign_is_not_flagged(src_root: Path) -> None:
    out = _violations(
        src_root,
        "void F() {\n"
        "  std::vector<Crystal> v;\n"
        "  v.assign(4, Crystal());\n"
        "}\n",
    )
    assert out == []


def test_copy_ctor_from_known_name_is_not_flagged(src_root: Path) -> None:
    out = _violations(
        src_root,
        "std::vector<Crystal> src_vec;\n"
        "std::vector<Crystal> copy_ctor(src_vec);\n",
    )
    assert out == []


def test_range_ctor_with_nested_calls_is_not_flagged(src_root: Path) -> None:
    """`v(a.begin(), a.end())` is a real two-argument iterator-range ctor;
    the first argument's own `)` must not be mistaken for the outer close."""
    out = _violations(
        src_root,
        "std::vector<Crystal> src_vec;\n"
        "std::vector<Crystal> ranged(src_vec.begin(), src_vec.end());\n",
    )
    assert out == []


def test_function_prototype_is_not_flagged(src_root: Path) -> None:
    """`std::vector<Crystal> Build(int n);` parses identically to a variable
    count-ctor declaration at this text-matching depth; the `type name`
    whitespace inside the parens is what distinguishes it."""
    out = _violations(src_root, "std::vector<Crystal> Build(int n);\n")
    assert out == []


def test_no_arg_emplace_back_is_flagged(src_root: Path) -> None:
    out = _violations(
        src_root,
        "void F() {\n"
        "  std::vector<Crystal> v;\n"
        "  v.emplace_back();\n"
        "}\n",
    )
    assert len(out) == 1


def test_reserve_is_not_flagged(src_root: Path) -> None:
    out = _violations(
        src_root,
        "void F() {\n"
        "  std::vector<Crystal> v;\n"
        "  v.reserve(4);\n"
        "}\n",
    )
    assert out == []
