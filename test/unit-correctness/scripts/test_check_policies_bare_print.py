"""Regression net for `check_no_bare_print` in `scripts/check_policies.py`.

Read the note at the top of `test_check_new_refs.py` first: this repo does not
test symbol-matching checks, only the ones that would fail silently. By that
criterion `check_no_bare_print` is a borderline case, and the honest reading is
that it sits on the *untested* side — it is two flat regexes run line-by-line
over `code_lines()`, with no parser of its own.

It is pinned here anyway, for the one thing that criterion does not cover: the
rule's two interesting behaviours are not written anywhere in the rule. That
`snprintf` is not flagged is a consequence of a `\\b` sitting between `n` and
`p` in a larger alternation; that a commented-out `printf(` is not flagged comes
from `strip_comments()`, a helper this check merely borrows and does not own.
Both are legible today by someone reading closely. Neither survives a future
edit to the pattern or to the shared helper on its own — and both fail toward
green, which is the direction nobody investigates.

Scope note: these cases pin *observable behaviour of the rule*, not the wording
of the regex. A rewrite of `BARE_PRINT_CALL` that keeps the buffer/stream
distinction should keep this file green.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "scripts"))

import check_policies  # noqa: E402

RULE = "no-bare-print"


@pytest.fixture
def src_root(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    src = tmp_path / "src"
    src.mkdir()
    monkeypatch.setattr(check_policies, "REPO_ROOT", tmp_path)
    monkeypatch.setattr(check_policies, "SRC", src)
    monkeypatch.setattr(check_policies, "BARE_PRINT_ALLOWED", frozenset({src / "main.cpp"}))
    return src


def _violations(src_root: Path, body: str, name: str = "scratch.cpp") -> list:
    path = src_root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(body, encoding="utf-8")
    return check_policies.check_no_bare_print()


# --- must stay red: real logger bypasses ------------------------------------


def test_bare_printf_is_flagged(src_root: Path) -> None:
    out = _violations(src_root, 'void F() {\n  printf("hello %d\\n", 1);\n}\n')
    assert len(out) == 1
    assert out[0].rule == RULE
    assert out[0].line == 2


def test_bare_cout_is_flagged(src_root: Path) -> None:
    out = _violations(src_root, 'void F() {\n  std::cout << "hello" << std::endl;\n}\n')
    assert len(out) == 1


def test_unqualified_cerr_is_flagged(src_root: Path) -> None:
    """The `std::` is optional on purpose, and the trailing `<<` is what makes
    that safe. GPU dialects require namespace imports, so this rule cannot lean
    on the `using namespace` ban the way a host-C++-only rule could."""
    assert len(_violations(src_root, 'void F() {\n  cerr << "boom";\n}\n')) == 1


def test_fputs_and_putchar_are_flagged(src_root: Path) -> None:
    out = _violations(
        src_root, 'void F() {\n  fputs("a", stderr);\n  putchar(10);\n}\n'
    )
    assert len(out) == 2


# --- must stay green: buffer formatters are not output ----------------------


@pytest.mark.parametrize("call", ["snprintf", "vsnprintf", "sprintf"])
def test_buffer_formatters_are_not_flagged(src_root: Path, call: str) -> None:
    """These write into a caller-supplied buffer; they emit nothing, so they are
    not a logging bypass. Nothing in the rule says so — it falls out of the `\\b`
    ahead of `printf`, which finds no word boundary inside `snprintf`. The whole
    logging policy would collapse into noise the day that boundary is dropped,
    and the failure would look like a wave of false positives on correct code.
    """
    out = _violations(src_root, f'void F() {{\n  char b[8];\n  {call}(b, 8, "%d", 1);\n}}\n')
    assert out == []


def test_commented_out_print_is_not_flagged(src_root: Path) -> None:
    """Comes from `strip_comments()`, which this check borrows and does not own —
    so a change made for some other check's benefit can silently reach here."""
    out = _violations(
        src_root,
        "void F() {\n"
        '  // printf("debug %d\\n", 1);\n'
        "  /* std::cout << \"debug\"; */\n"
        "}\n",
    )
    assert out == []


def test_logger_macros_are_not_flagged(src_root: Path) -> None:
    """The sanctioned spelling must not trip the rule that recommends it."""
    out = _violations(
        src_root, 'void F() {\n  ILOG_INFO("hello {}", 1);\n  GUI_LOG_WARN("x");\n}\n'
    )
    assert out == []


# --- scope: which files are read at all -------------------------------------


@pytest.mark.parametrize("suffix", [".cpp", ".hpp", ".h", ".mm", ".cu", ".cuh", ".metal"])
def test_gpu_and_host_suffixes_are_all_scanned(src_root: Path, suffix: str) -> None:
    """PRINT_SCAN_SUFFIXES deliberately widens past CXX_SUFFIXES so a CUDA/Metal
    backend cannot reopen the stderr side channel. GPU sources were invisible to
    every policy check before that widening, so this is the one scope fact worth
    pinning per suffix rather than in aggregate."""
    out = _violations(src_root, 'void F() {\n  printf("x");\n}\n', name=f"gpu{suffix}")
    assert len(out) == 1, f"{suffix} not scanned"


def test_allowlisted_file_is_skipped(src_root: Path) -> None:
    """main.cpp's stdout IS the CLI's product output; two of its lines are parsed
    contracts. The exemption is per file and by name, never per line."""
    assert _violations(src_root, 'int main() {\n  printf("[BENCHMARK] {}");\n}\n', name="main.cpp") == []


def test_non_source_suffix_is_not_scanned(src_root: Path) -> None:
    """A .py/.md/.txt under src/ is not a translation unit; scanning it would
    flag documentation of the rule as a violation of it."""
    assert _violations(src_root, 'printf("this is prose about printf(")\n', name="notes.md") == []


def test_check_is_registered_in_checks(src_root: Path) -> None:
    assert check_policies.check_no_bare_print in check_policies.CHECKS
