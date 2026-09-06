"""Regression net for `check_msvc_string_literal_limit` in `scripts/check_policies.py`.

Read the note at the top of `test_check_new_refs.py` first: this repo does not
test symbol-matching checks, only the ones that would fail silently. This rule
is on the tested side for two independent reasons.

First, the boundary is a number nobody can see. 16384 is MSVC's cap, and the
difference between `<=` and `<` at that comparison is one byte of a shader — not
something a reader of the diff would catch, and not something any other leg of
CI can report, because Clang, GCC and VS 2026 all accept the oversized literal
happily. A regression here surfaces as a failed *release*, on the one job that
builds Windows binaries, after the tag is already pushed.

Second, the rule owns a parser. The pattern has to find a delimiter, match it
against its own closing marker, and measure the body in bytes rather than
characters — and it deliberately reads raw file text instead of the shared
`strip_comments()` helper, because a shader body contains `//` runs as content.
Every one of those decisions fails toward green if it is edited carelessly: a
greedy body swallows two literals into one measurement, a stripped comment
under-counts, `len()` on a str instead of on bytes under-counts again.

Scope note: these cases pin *observable behaviour of the rule*, not the wording
of the regex. A rewrite that keeps the byte-counted body and the back-referenced
delimiter should keep this file green.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import check_policies  # noqa: E402

RULE = "msvc-string-literal-limit"
LIMIT = check_policies.MSVC_STRING_LITERAL_LIMIT


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
    return check_policies.check_msvc_string_literal_limit()


def _raw(payload: str, delim: str = "glsl") -> str:
    return f'R"{delim}({payload}){delim}"'


# --- the boundary value itself ----------------------------------------------


def test_limit_constant_is_pinned_to_msvcs_observed_c2026_trigger() -> None:
    """16384 is inside the net, not just the comparison drawn around it.

    Every other case in this file builds its payload from `LIMIT`, read back out
    of the module under test, so together they pin the *relative* behaviour of
    that `<=` and nothing at all about the number it compares against. Change
    `MSVC_STRING_LITERAL_LIMIT` to 8192, or add a digit to it, and all of them
    stay green while the gate stops agreeing with MSVC — which is the exact
    shape of the failure this rule exists to prevent: green CI, and a release
    that still dies on the Windows leg.

    The 16384 below is written out on purpose and must not be replaced by a
    reference to the constant. It is the observed C2026 trigger point, the same
    number recorded in this task's issue and plan.
    """
    assert check_policies.MSVC_STRING_LITERAL_LIMIT == 16384


# --- must stay red: a literal over the cap ----------------------------------


def test_oversized_literal_is_flagged(src_root: Path) -> None:
    out = _violations(src_root, f"const char* k =\n{_raw('x' * (LIMIT + 1))};\n")
    assert len(out) == 1
    assert out[0].rule == RULE
    assert str(LIMIT + 1) in out[0].message


def test_violation_points_at_the_opening_line(src_root: Path) -> None:
    """The line a reader has to go edit — not where MSVC's own buffer ran out.

    A multi-line body makes the two diverge: MSVC reports a line deep inside the
    literal. Anchoring on `match.start()` is what keeps this at the opener.
    """
    payload = "\n".join("y" * 64 for _ in range(LIMIT // 32))
    body = "// header\n// header\nconst char* k =\n" + _raw(payload) + ";\n"
    out = _violations(src_root, body)
    assert len(out) == 1
    assert out[0].line == 4


# --- the boundary itself ----------------------------------------------------


def test_exactly_at_the_limit_is_clean(src_root: Path) -> None:
    assert _violations(src_root, f"const char* k = {_raw('x' * LIMIT)};\n") == []


def test_one_byte_over_the_limit_is_flagged(src_root: Path) -> None:
    assert len(_violations(src_root, f"const char* k = {_raw('x' * (LIMIT + 1))};\n")) == 1


def test_body_that_desyncs_the_comment_stripper_is_still_flagged(src_root: Path) -> None:
    """Pins the reason this rule reads raw text instead of `strip_comments()`.

    That helper does not parse raw strings: `R"delim(` reads to it as an ordinary
    string opener, so an odd number of `"` in the body desyncs its state machine,
    and a `//` past that point is blanked as a comment. Blanking reaches the
    closing `)delim"`, the literal stops being findable, and an oversized shader
    reports as clean — a miss, not a wrong number, which is why a byte-count
    assertion alone would not catch the regression.

    The body below is that exact shape: one unbalanced quote, then a trailing
    `//` line. Routed through the stripper it yields zero violations; read raw it
    yields one. Verified by mutating the rule to call the helper.
    """
    body = '// 6" wide\n' + "x" * (LIMIT + 1) + "\n// trailing note "
    out = _violations(src_root, f"const char* k =\n{_raw(body)};\n")
    assert len(out) == 1


def test_limit_is_measured_in_bytes_not_characters(src_root: Path) -> None:
    """A multi-byte body under the cap in characters can be over it in bytes.

    MSVC counts the encoded bytes, so `len(str)` would let a literal through that
    the compiler rejects. Half the cap in 3-byte characters is comfortably over.
    """
    payload = "あ" * (LIMIT // 2)
    assert len(payload) < LIMIT
    out = _violations(src_root, f"const char* k = {_raw(payload)};\n")
    assert len(out) == 1


# --- must stay green: shapes the rule is not about --------------------------


def test_adjacent_literals_each_under_the_limit_are_clean(src_root: Path) -> None:
    """The end state this task produces: one literal split into two adjacent ones.

    The compiler concatenates them back into identical bytes, and MSVC applies
    its cap per literal, so the pair must read as clean even though the combined
    body is over. A greedy body pattern would merge them and report a false red.
    """
    half = "x" * (LIMIT - 1)
    body = f"const char* k =\n{_raw(half)}\n{_raw(half)};\n"
    assert _violations(src_root, body) == []


def test_plain_string_literal_is_not_scanned(src_root: Path) -> None:
    """Scope is raw literals only.

    A non-raw literal that long cannot be written on one source line anyway, and
    the rule was not asked to cover it — stated here so a later reader does not
    read the absence as an oversight.
    """
    assert _violations(src_root, 'const char* k = "' + "x" * (LIMIT + 1) + '";\n') == []


def test_distinct_delimiters_do_not_close_each_other(src_root: Path) -> None:
    """The closing marker must match the opening delimiter.

    The body opens with `)b"` — a well-formed closing marker for a *different*
    delimiter — and only then runs past the cap before its own `)a"` arrives.
    Drop the back-reference and the non-greedy body stops at that first `)b"`,
    measuring an empty literal and reporting the file clean, so the oversized
    body is missed entirely rather than mis-sized.
    """
    payload = ')b"' + "x" * (LIMIT + 1)
    body = f'const char* k = R"a({payload})a";\n'
    assert len(_violations(src_root, body)) == 1


# --- scanned file set -------------------------------------------------------


@pytest.mark.parametrize("suffix", [".cpp", ".hpp", ".cu", ".cuh", ".metal", ".mm", ".inl"])
def test_every_scanned_suffix_is_covered(src_root: Path, suffix: str) -> None:
    """A GPU backend must not be able to reopen the gap the rule closes."""
    out = _violations(src_root, f"const char* k = {_raw('x' * (LIMIT + 1))};\n", f"s{suffix}")
    assert len(out) == 1


def test_unscanned_suffix_is_ignored(src_root: Path) -> None:
    assert _violations(src_root, f"k = {_raw('x' * (LIMIT + 1))}\n", "notes.txt") == []


def test_nested_directories_are_scanned(src_root: Path) -> None:
    out = _violations(src_root, f"const char* k = {_raw('x' * (LIMIT + 1))};\n", "gui/deep/s.cpp")
    assert len(out) == 1


# --- registration -----------------------------------------------------------


def test_check_is_registered() -> None:
    """An unregistered check is a check that never runs."""
    assert check_policies.check_msvc_string_literal_limit in check_policies.CHECKS
