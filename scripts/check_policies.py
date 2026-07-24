#!/usr/bin/env python3
"""Deterministic engineering-policy checks for Lumice.

This is the *executable* half of the policy documents under doc/: it verifies the
code itself, not a human's promise that they followed a rule. Run by the CI
`policy` job and by the local pre-commit hook (scripts/hooks/pre-commit). Exit
code is 0 when clean, 1 when any violation is found (each printed as file:line).

Checks:
  1. env-getenv-centralization — std::getenv("LUMICE_...") may appear ONLY in
     src/util/env_knobs.cpp. Everything else must route through util/env_knobs
     (doc/env-var-policy.md).
  2. env-knob-registration — every LUMICE_* name read in env_knobs.cpp must be
     documented in doc/env-var-policy.md, so the knob is discoverable.
  3. gui-api-boundary — src/gui/** must not #include "core/..." or "config/...";
     the GUI talks to core only through the C API (src/include/lumice.h).
  4. no-using-namespace — `using namespace` is banned in src/ (AGENTS.md).
  5. struct-layout-parity — canonical/mirror struct pairs (currently just
     `GenRootKernelParams` between pcg_shared.h and metal_trace_backend.mm) must
     have identical ordered (type, name) field lists. Catches same-size field
     reorders that sizeof-based `static_assert`s do not detect (scrum-328.2
     Step 6).
  6. no-config-by-value-copy — LUMICE_Config owns heap allocations via its
     raypath_color pointer (task-344, v4.8) AND each compositions[i]'s
     term_ids/term_counts pointers (task-host-abi-cpu-caps, v4.9). The check
     also covers LUMICE_ComplexComposition (each record is now an owning
     type on its own). Copying either struct by value (copy-init, direct-
     init, list-init, by-value parameter, or later-line assignment `y = x;`)
     aliases the same heap blocks and double-frees on double Release. Route
     through pointer or `const &`, and manage lifetime via the type's
     Create/Set / Release C API (or lumice::ConfigOwningGuard).
  7. gui-state-field-tier-registration — every top-level field of
     `struct GuiState` (src/gui/gui_state.hpp) must be registered in EXACTLY
     one of the two tables in src/gui/gui_state_tiers.hpp: `kFieldTierTable`
     (a governed tier — struct-hard/soft, display, view, session) or
     `kDerivedFieldsExcludeList` (runtime-derived / not governed). Adding a
     new GuiState field without touching the tiers header is a violation.
     Closes the "unregistered field silently escapes governance" loophole
     that scrum-gui-state-reconcile T0 (reconcile-foundation) was built to
     prevent (doc/gui-state-governance.md).
  8. no-msvc-unsafe-builtin — MSVC does not recognise the GCC/Clang
     `__builtin_popcount*` / `__builtin_ctz*` / `__builtin_clz*` intrinsic
     families and reports C3861. src/ and test/ must route through
     `lumice::PopCount` (src/util/bit_utils.hpp) or an equivalent portable
     implementation. Scope is intentionally narrow: only the popcount/ctz/clz
     families are pinned (task-popcount-helper), extend the regex when a new
     MSVC-unsafe `__builtin_*` family surfaces.
  9. no-default-constructed-crystal-slots — no `std::vector<Crystal>` in src/
     may be sized by `resize()`, `assign(n)`, or a count-ctor, and none may be
     grown with an argument-less `emplace_back()`. All four produce
     default-constructed `Crystal` elements: `face_cnt == 0`, no polygon faces,
     `fn_period_ == -1`, null normal/distance tables. That state is the same
     one the closed-form validity gate returns for a degenerate crystal, so
     consumers that iterate the container cannot tell "never filled in" from
     "legitimately empty", and a partially-filled container (e.g. a per-ci loop
     that `continue`s on empty populations) silently hands out untouched slots.
     Every such container must instead be filled element-by-element with real
     crystals via `push_back`/`emplace_back(args...)`, so its size and its
     populated prefix are the same thing by construction. The invariant is
     pinned here rather than in a comment because the failure is silent: a
     default slot reads as a valid empty crystal all the way to the GPU.

Add a new check as a function returning a list of Violation and append it to
CHECKS. Keep each check deterministic and artifact-inspecting.
"""
from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

# Anyone overriding this to point the checks at another tree must rebind it in
# every module that imported it, not just this one: `from check_policies import
# REPO_ROOT` copies the value, so a later assignment here is invisible to the
# importer. The two bindings then disagree silently, and the split does not
# surface where it is made — Violation.render() below resolves the name in this
# module, while an importer's own path handling resolves its private copy. Rebind
# one and everything works until the first violation reaches the renderer, which
# is the only code path taken when there is something to report.
REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
ENV_KNOBS_CPP = SRC / "util" / "env_knobs.cpp"
POLICY_DOC = REPO_ROOT / "doc" / "env-var-policy.md"

CXX_SUFFIXES = {".cpp", ".cc", ".hpp", ".h", ".mm", ".inl"}


@dataclass
class Violation:
    path: Path
    line: int
    rule: str
    message: str

    def render(self) -> str:
        rel = self.path.relative_to(REPO_ROOT)
        return f"  {rel}:{self.line}: [{self.rule}] {self.message}"


def cxx_sources(root: Path):
    for p in sorted(root.rglob("*")):
        if p.suffix in CXX_SUFFIXES and p.is_file():
            yield p


def strip_comments(text: str) -> str:
    """Replace // and /* */ comments with spaces, preserving offsets and newlines.

    Respects string and char literals so that `//` or `/*` inside a literal (e.g.
    a "LUMICE_..." env name or a URL) is NOT mistaken for a comment. Newlines are
    kept so line numbers stay accurate.

    Known limitation: C++11 raw string literals (R"delim(...)delim") are not
    parsed specially — a raw string containing `//`, `/*`, or a getenv("LUMICE_…")
    fragment could be mis-handled. The codebase currently uses none, so this is a
    documented gap, not an active false-positive source.
    """
    out = []
    i, n = 0, len(text)
    state = "code"  # code | line_comment | block_comment | string | char
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == "/" and nxt == "/":
                state = "line_comment"
                out.append("  ")
                i += 2
                continue
            if c == "/" and nxt == "*":
                state = "block_comment"
                out.append("  ")
                i += 2
                continue
            if c == '"':
                state = "string"
                out.append(c)
            elif c == "'":
                state = "char"
                out.append(c)
            else:
                out.append(c)
            i += 1
        elif state == "line_comment":
            out.append("\n" if c == "\n" else " ")
            if c == "\n":
                state = "code"
            i += 1
        elif state == "block_comment":
            out.append("\n" if c == "\n" else " ")
            if c == "*" and nxt == "/":
                out.append(" ")
                i += 2
                state = "code"
                continue
            i += 1
        elif state in ("string", "char"):
            out.append(c)
            if c == "\\":  # escape: copy next char verbatim
                if nxt:
                    out.append(nxt)
                    i += 2
                    continue
            elif (state == "string" and c == '"') or (state == "char" and c == "'"):
                state = "code"
            i += 1
    return "".join(out)


def code_lines(path: Path):
    """Yield (lineno, original_line, code_only_line) with comments blanked out."""
    text = path.read_text(encoding="utf-8", errors="replace")
    stripped = strip_comments(text)
    originals = text.splitlines()
    codes = stripped.splitlines()
    for idx, code in enumerate(codes):
        original = originals[idx] if idx < len(originals) else ""
        yield idx + 1, original, code


GETENV_LUMICE = re.compile(r'getenv\s*\(\s*"(LUMICE_[A-Z0-9_]+)"')
# Catches "core/..", <core/..>, and relative "../core/.." forms. The \b before
# (core|config) keeps it from matching substrings like "score/" or "mycore/".
GUI_FORBIDDEN_INCLUDE = re.compile(r'#\s*include\s*[<"][^">]*\b(core|config)/')
USING_NAMESPACE = re.compile(r"\busing\s+namespace\b")
# `\w*` after the family root absorbs all known length suffixes (`ll`, `l`,
# `s`, `g`) without enumerating each. Only the popcount/ctz/clz families are
# pinned: PR#182 was specifically about `__builtin_popcountll`; extend the
# alternation when a new MSVC-unsafe family surfaces.
MSVC_UNSAFE_BUILTIN = re.compile(r"\b__builtin_(?:popcount|ctz|clz)\w*")


def check_getenv_centralization() -> list[Violation]:
    out: list[Violation] = []
    for path in cxx_sources(SRC):
        if path == ENV_KNOBS_CPP:
            continue
        for lineno, _orig, code in code_lines(path):
            if GETENV_LUMICE.search(code):
                out.append(
                    Violation(
                        path,
                        lineno,
                        "env-getenv-centralization",
                        'std::getenv("LUMICE_...") is only allowed in '
                        "src/util/env_knobs.cpp; route this knob through util/env_knobs "
                        "(doc/env-var-policy.md).",
                    )
                )
    return out


def check_env_knob_registration() -> list[Violation]:
    out: list[Violation] = []
    if not ENV_KNOBS_CPP.exists():
        return out
    doc_text = POLICY_DOC.read_text(encoding="utf-8") if POLICY_DOC.exists() else ""
    for lineno, _orig, code in code_lines(ENV_KNOBS_CPP):
        for m in GETENV_LUMICE.finditer(code):
            name = m.group(1)
            if name not in doc_text:
                out.append(
                    Violation(
                        ENV_KNOBS_CPP,
                        lineno,
                        "env-knob-registration",
                        f"{name} is read here but not documented in "
                        "doc/env-var-policy.md; register it in the A-class table.",
                    )
                )
    return out


def check_gui_api_boundary() -> list[Violation]:
    out: list[Violation] = []
    gui = SRC / "gui"
    if not gui.exists():
        return out
    for path in cxx_sources(gui):
        for lineno, _orig, code in code_lines(path):
            m = GUI_FORBIDDEN_INCLUDE.search(code)
            if m:
                out.append(
                    Violation(
                        path,
                        lineno,
                        "gui-api-boundary",
                        f'src/gui/ must not #include "{m.group(1)}/..."; use the C '
                        "API (src/include/lumice.h) instead.",
                    )
                )
    return out


# reconciler-widget-include: gui_state_reconcile.cpp is the "single owner of effects" reconciler.
# T1 (task-color-migration) landed one concrete-widget dependency in it — `#include "gui/
# color_window.hpp"` to call PushDisplayState — which inverts the intended layering direction
# (widget depends on reconciler abstraction, not the reverse). That is acceptable for a SINGLE
# domain but must not accrete: the moment a second `need_X_push` display channel would add a
# second widget-header include here, the author must instead introduce a push-handler registration/
# dispatch abstraction (doc/gui-state-governance.md §4 支柱 2). The inline comment + doc were the
# only guard, i.e. a soft constraint; this gate hardens it (决策公理 a09). Any `#include "gui/..."`
# in this file outside the allowlist below fails — forcing exactly that evaluation.
RECONCILE_CPP = SRC / "gui" / "gui_state_reconcile.cpp"
RECONCILE_GUI_INCLUDE = re.compile(r'#\s*include\s*"(gui/[^"]+)"')
# Grandfathered includes. `gui_state_reconcile.hpp` = own header; `gui_state.hpp` = pure data
# struct (a tier below, not a widget); `color_window.hpp` = the single T1 reverse-include. Adding
# a NEW widget-header entry here without introducing the abstraction is the thing to stop, so
# extending this set is a deliberate reviewed act, not a silent edit.
RECONCILE_ALLOWED_GUI_INCLUDES = frozenset(
    {
        "gui/gui_state_reconcile.hpp",
        "gui/gui_state.hpp",
        "gui/color_window.hpp",
    }
)


def check_reconciler_widget_include() -> list[Violation]:
    out: list[Violation] = []
    if not RECONCILE_CPP.exists():
        return out
    for lineno, _orig, code in code_lines(RECONCILE_CPP):
        m = RECONCILE_GUI_INCLUDE.search(code)
        if m and m.group(1) not in RECONCILE_ALLOWED_GUI_INCLUDES:
            out.append(
                Violation(
                    RECONCILE_CPP,
                    lineno,
                    "reconciler-widget-include",
                    f'gui_state_reconcile.cpp must not add #include "{m.group(1)}": the '
                    "reconciler is the single owner of effects and must not reverse-depend on a "
                    "second widget domain. Introduce a push-handler registration/dispatch "
                    "abstraction instead (doc/gui-state-governance.md §4 支柱 2), or if this is a "
                    "pure data/utility header, add it to RECONCILE_ALLOWED_GUI_INCLUDES.",
                )
            )
    return out


def check_no_using_namespace() -> list[Violation]:
    out: list[Violation] = []
    for path in cxx_sources(SRC):
        for lineno, _orig, code in code_lines(path):
            if USING_NAMESPACE.search(code):
                out.append(
                    Violation(
                        path,
                        lineno,
                        "no-using-namespace",
                        "`using namespace` is banned in src/ (AGENTS.md).",
                    )
                )
    return out


# Struct-layout parity: ordered (type, name) field lists must match between the
# canonical `GenRootKernelParams` in src/core/shared/pcg_shared.h and its host
# mirror in src/core/backend/metal_trace_backend.mm. Field order/type/name drift
# in either file silently breaks the MSL-shader ↔ host binding — sizeof static
# asserts only catch size deltas, not same-size reorders.
#
# Regex approach (matches the existing check style — no AST): scan for the
# canonical struct block, then extract one `<type> <name>;` per line inside it.
# `GenRootKernelParams` is intentionally simple (POD, single-declaration lines,
# no nested structs / macros / bitfields) so this survives; if the struct
# complexifies later, harden the extractor rather than swap to a full parser.
STRUCT_FIELD_RE = re.compile(
    r"^\s*(?P<type>[A-Za-z_][A-Za-z0-9_:<>\s]*?)\s+(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*(?:=[^;]+)?;\s*$"
)

STRUCT_LAYOUT_PAIRS: list[tuple[str, Path, Path]] = [
    (
        "GenRootKernelParams",
        SRC / "core" / "shared" / "pcg_shared.h",
        SRC / "core" / "backend" / "metal_trace_backend.mm",
    ),
]


def _extract_struct_fields(path: Path, struct_name: str) -> tuple[list[tuple[str, str]], int]:
    """Return (ordered [(type, name)], line where the struct opens) — or ([], -1)
    if the struct isn't found. Uses the comment-stripped text so field lines
    inside comments do not pollute the list; strings inside the struct body are
    unlikely for a POD, so the comment stripper alone is sufficient.
    """
    text = path.read_text(encoding="utf-8")
    stripped = strip_comments(text)
    open_re = re.compile(r"\bstruct\s+" + re.escape(struct_name) + r"\s*\{")
    m = open_re.search(stripped)
    if not m:
        return [], -1
    open_line = stripped.count("\n", 0, m.start()) + 1
    # Walk braces from the opening `{` to find the matching close.
    body_start = m.end()
    depth = 1
    i = body_start
    while i < len(stripped) and depth > 0:
        c = stripped[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                break
        i += 1
    if depth != 0:
        return [], open_line
    body = stripped[body_start:i]
    fields: list[tuple[str, str]] = []
    for raw in body.splitlines():
        line = raw.strip()
        if not line or line.startswith("//") or line.startswith("#"):
            continue
        fm = STRUCT_FIELD_RE.match(line)
        if fm:
            type_str = " ".join(fm.group("type").split())  # collapse whitespace
            fields.append((type_str, fm.group("name")))
    return fields, open_line


def check_struct_layout_parity() -> list[Violation]:
    out: list[Violation] = []
    for struct_name, canon_path, mirror_path in STRUCT_LAYOUT_PAIRS:
        canon_fields, canon_line = _extract_struct_fields(canon_path, struct_name)
        mirror_fields, mirror_line = _extract_struct_fields(mirror_path, struct_name)
        if not canon_fields:
            out.append(
                Violation(
                    canon_path,
                    canon_line if canon_line > 0 else 1,
                    "struct-layout-parity",
                    f"canonical struct `{struct_name}` not found in {canon_path.name}",
                )
            )
            continue
        if not mirror_fields:
            out.append(
                Violation(
                    mirror_path,
                    mirror_line if mirror_line > 0 else 1,
                    "struct-layout-parity",
                    f"mirror struct `{struct_name}` not found in {mirror_path.name}",
                )
            )
            continue
        if len(canon_fields) != len(mirror_fields):
            out.append(
                Violation(
                    mirror_path,
                    mirror_line,
                    "struct-layout-parity",
                    f"`{struct_name}` field count differs: canonical={len(canon_fields)} "
                    f"vs mirror={len(mirror_fields)} (canonical @ {canon_path.name}:{canon_line})",
                )
            )
        for idx, (canon, mirror) in enumerate(zip(canon_fields, mirror_fields)):
            if canon != mirror:
                out.append(
                    Violation(
                        mirror_path,
                        mirror_line,
                        "struct-layout-parity",
                        (
                            f"`{struct_name}` field #{idx} differs: canonical=`{canon[0]} {canon[1]}` "
                            f"(@ {canon_path.name}:{canon_line}) vs mirror=`{mirror[0]} {mirror[1]}`"
                        ),
                    )
                )
    return out


# no-config-by-value-copy: LUMICE_Config carries an owning heap pointer
# (raypath_color) since task-344 / v4.8. Any by-value copy — copy/direct/list
# initialization, by-value parameter, or later-line assignment `y = x;` between
# two same-type locals — aliases the same heap block and double-frees at Release.
# Route through pointer or `const LUMICE_Config&`, and manage lifetime via
# LUMICE_ConfigCreateColorClasses / Release (or lumice::ConfigColorGuard).
#
# Known limitation (recorded in plan §3 design-point 6, out of scope for this
# task): does not cover container-held instances (`std::vector<LUMICE_Config>`
# etc.). Codebase audit found no such usage; extend the rule if it appears.
#
# Known limitation (code-review-01, round 1): Patterns 1a/1b/1c all require
# the RHS to be a single bare identifier, so a function-call RHS such as
# `LUMICE_Config x = SomeFactory();` is not matched. The codebase currently
# avoids this shape (test factories were converted to out-param `Fill*` style
# specifically to sidestep it), but a future by-value-returning factory would
# not be caught — extend Pattern 1a's RHS match to also accept `\w+\(...\)`
# (excluding aggregate-init syntax) if this shape reappears.
# v4.9 (task-host-abi-cpu-caps): LUMICE_ComplexComposition now owns two heap
# pointers (term_ids / term_counts) via LUMICE_CompositionSetClauses, so it
# joins the ban list on the same "aliased owning pointer → double-free"
# grounds as LUMICE_Config. Same 4 patterns run for each type.
#
# code-review-01 (round 1, Major) fix: the RHS of Patterns 1a/1b/1c originally
# matched only a bare identifier (`\w+`), so the exact risk this task's own
# issue.md/plan called out — `LUMICE_ComplexComposition rec = cfg.compositions[i];`
# aliasing the owning term_ids/term_counts heap allocation — was invisible to the
# gate. `_RHS_EXPR` below widens the RHS to also accept member (`.field`),
# pointer-member (`->field`), and subscript (`[expr]`) chains so member/subscript
# copy sources are caught too. The (still-open) function-call-RHS limitation
# above is unrelated and unchanged.
GUARDED_OWNING_TYPES = ["LUMICE_Config", "LUMICE_ComplexComposition"]

# A bare identifier optionally followed by any number of `.field`, `->field`,
# or `[expr]` accessors — e.g. `cfg`, `cfg.compositions[i]`, `arr[i]->comp`.
_RHS_EXPR = r"\w+(?:\.\w+|->\w+|\[[^\[\]]+\])*"


def _make_owning_type_patterns(type_name: str):
    """Build the (copy_init, direct_init, list_init, by_value_param, decl) regex
    tuple for one owning type. Kept as a factory so each type gets a fresh
    compiled set — the four regexes are identical in shape but bound to the
    specific type token so error messages / self-assignment guards read
    naturally."""
    copy_init = re.compile(r"\b" + type_name + r"\s+(\w+)\s*=\s*(" + _RHS_EXPR + r")\s*;")
    direct_init = re.compile(r"\b" + type_name + r"\s+(\w+)\s*\(\s*(" + _RHS_EXPR + r")\s*\)\s*;")
    list_init = re.compile(r"\b" + type_name + r"\s+(\w+)\s*\{\s*(" + _RHS_EXPR + r")\s*\}\s*;")
    by_value_param = re.compile(r"\b" + type_name + r"\s+(\w+)\s*[,)]")
    decl = re.compile(r"\b" + type_name + r"(\s*[*&]?)\s*(\w+)")
    return copy_init, direct_init, list_init, by_value_param, decl


def _is_pointer_or_ref_decl(match: "re.Match[str]") -> bool:
    return "*" in match.group(1) or "&" in match.group(1)


def check_no_config_by_value_copy() -> list[Violation]:
    out: list[Violation] = []
    for type_name in GUARDED_OWNING_TYPES:
        patterns = _make_owning_type_patterns(type_name)
        for path in cxx_sources(SRC):
            out.extend(_scan_config_copies(path, type_name, patterns))
        test_root = REPO_ROOT / "test"
        if test_root.exists():
            for path in cxx_sources(test_root):
                out.extend(_scan_config_copies(path, type_name, patterns))
    return out


def _scan_config_copies(path: Path, type_name: str, patterns) -> list[Violation]:
    copy_init_re, direct_init_re, list_init_re, by_value_param_re, decl_re = patterns
    out: list[Violation] = []
    # First pass: collect all `<type_name> <name>` value declarations by name.
    # Excludes pointer/reference declarations. Used only for Pattern 3
    # (later-line bare assignment) to keep the check anchored on same-type
    # locals rather than any arbitrary `a = b`.
    value_decls: set[str] = set()
    for _lineno, _orig, code in code_lines(path):
        for m in decl_re.finditer(code):
            if _is_pointer_or_ref_decl(m):
                continue
            value_decls.add(m.group(2))

    for lineno, _orig, code in code_lines(path):
        # Patterns 1a/1b/1c: copy/direct/list initialization from a single
        # existing identifier. All three share the "RHS is one bare
        # identifier" judgement — aggregate init `{}`, `= {}`, `{a, b}`,
        # function call `foo()` are not matched.
        for m in copy_init_re.finditer(code):
            lhs, rhs = m.group(1), m.group(2)
            if lhs == rhs:
                continue
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"copy-init `{type_name} {lhs} = {rhs};` aliases the "
                    "owning heap allocation; use a pointer or reference.",
                )
            )
        for m in direct_init_re.finditer(code):
            lhs, rhs = m.group(1), m.group(2)
            if lhs == rhs:
                continue
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"direct-init `{type_name} {lhs}({rhs});` aliases the "
                    "owning heap allocation; use a pointer or reference.",
                )
            )
        for m in list_init_re.finditer(code):
            lhs, rhs = m.group(1), m.group(2)
            if lhs == rhs:
                continue
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"list-init `{type_name} {lhs}{{{rhs}}};` aliases the "
                    "owning heap allocation; use a pointer or reference.",
                )
            )
        # Pattern 2: by-value function parameter. The `\s+` after the type
        # already excludes `<type>*name` / `<type>&name`
        # (adjacent `*`/`&` binds to the type token with no space). Explicit
        # spaced variants like `<type> * name` or `... & name` still
        # match Pattern 2 as a false positive — the plan accepts this because
        # the codebase uses adjacent `*`/`&` universally and the spaced form
        # would violate the local style anyway.
        for m in by_value_param_re.finditer(code):
            # Guard against matching a value declaration inside a function
            # body (which Patterns 1a/1b/1c already own): parameter positions
            # are directly preceded by `(` or `,` (possibly with whitespace),
            # while local declarations are typically preceded by `{`, `;`, or
            # start-of-line.
            start = m.start()
            prefix = code[:start]
            # Walk backwards past whitespace to find the nearest non-space char.
            j = len(prefix) - 1
            while j >= 0 and prefix[j] in " \t":
                j -= 1
            preceding = prefix[j] if j >= 0 else ""
            if preceding not in ("(", ","):
                continue
            # Skip trivial `<type> x` where x is followed by `[` in
            # e.g. `<type> x[]` — that's an array parameter, not a
            # by-value scalar. The regex ends on `[,)]` so an array param
            # `<type> x[]` would already not match (the `]` is not `,`
            # or `)`), no extra guard needed.
            name = m.group(1)
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"by-value parameter `{type_name} {name}` aliases the "
                    "owning heap allocation; use "
                    f"`const {type_name}&` or `{type_name}*`.",
                )
            )
        # Pattern 3: `a = b;` where both `a` and `b` are same-type locals we
        # saw declared in this file. Skips self-assignment. Skips lines that
        # already look like an initialization (would be caught by Pattern 1a).
        if "=" in code and value_decls:
            # Cheap containment prefilter; avoid a Cartesian regex sweep for
            # files with hundreds of unrelated `=` lines.
            for name_lhs in value_decls:
                for name_rhs in value_decls:
                    if name_lhs == name_rhs:
                        continue
                    assign_re = re.compile(
                        r"(?<![*&\w])\b"
                        + re.escape(name_lhs)
                        + r"\s*=\s*"
                        + re.escape(name_rhs)
                        + r"\s*;"
                    )
                    if assign_re.search(code):
                        # Filter out the declaration line itself (would double-
                        # count with Pattern 1a). Detected by checking that
                        # `<type>` does not immediately precede name_lhs
                        # on this line.
                        decl_here = re.compile(
                            r"\b" + type_name + r"\s+" + re.escape(name_lhs) + r"\b"
                        )
                        if decl_here.search(code):
                            continue
                        out.append(
                            Violation(
                                path,
                                lineno,
                                "no-config-by-value-copy",
                                f"assignment `{name_lhs} = {name_rhs};` between two "
                                f"{type_name} locals aliases the owning heap "
                                "allocation; use a pointer/reference.",
                            )
                        )
    return out


# gui-state-field-tier-registration: coverage-union check between the top-level
# fields of `struct GuiState` (src/gui/gui_state.hpp) and the two registries in
# `src/gui/gui_state_tiers.hpp` (`kFieldTierTable` + `kDerivedFieldsExcludeList`).
# This closes the "silently un-governed field" loophole — the whole point of the
# reconcile-foundation geodetics — by failing the build when a GuiState field is
# missing from BOTH tables, is registered in BOTH, or has been removed from the
# struct but is still referenced in a table (stale entry).
#
# Parser scope: `struct GuiState` is much richer than `GenRootKernelParams`
# (nested struct declarations, method bodies, enum decls, array fields with
# aggregate initializers). We can't reuse `_extract_struct_fields`. Instead we
# walk the struct body line-by-line, tracking depth so nested-struct / method
# bodies are ignored, and match top-level field lines with a bespoke regex.
GUI_STATE_HPP = SRC / "gui" / "gui_state.hpp"
GUI_STATE_TIERS_HPP = SRC / "gui" / "gui_state_tiers.hpp"

GUI_FIELD_LINE_RE = re.compile(
    r"^"
    r"\s*"
    r"(?:(?:static|inline|mutable|constexpr|volatile)\s+)*"
    r"[A-Za-z_][\w:<>\s,\*&]*?"
    r"\s"
    r"(?P<name>[A-Za-z_]\w*)"
    r"(?:\s*\[[^\]]*\])?"
    r"\s*(?:=[^;]*|\{[^;]*\})?"
    r"\s*;"
    r"\s*$"
)

GUI_FIELD_SKIP_KEYWORDS_RE = re.compile(
    r"^(struct|enum|class|union|friend|static_assert|template|typedef|using|return)\b"
)


def _extract_gui_state_top_level_fields(path: Path) -> tuple[set[str], int]:
    text = path.read_text(encoding="utf-8")
    stripped = strip_comments(text)
    m = re.search(r"\bstruct\s+GuiState\s*\{", stripped)
    if not m:
        return set(), -1
    open_line = stripped.count("\n", 0, m.start()) + 1
    body_start = m.end()
    depth = 1
    i = body_start
    while i < len(stripped) and depth > 0:
        c = stripped[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                break
        i += 1
    body = stripped[body_start:i]

    fields: set[str] = set()
    nested_depth = 0
    for raw in body.splitlines():
        line = raw.strip()
        opens = line.count("{")
        closes = line.count("}")
        pre_depth = nested_depth
        nested_depth += opens - closes
        # Only lines fully at depth 0 (both before and after) can carry a
        # top-level field. Nested struct/method-body lines are skipped.
        if pre_depth != 0 or nested_depth != 0:
            continue
        if not line or line.startswith("#"):
            continue
        if re.match(r"^(public|private|protected)\s*:", line):
            continue
        if GUI_FIELD_SKIP_KEYWORDS_RE.match(line):
            continue
        # A method declaration/definition has '(' in its declarator (before any
        # '=' default-value token, since a default parameter's '=' can only
        # occur *inside* the already-open param list). A field with a
        # call-style default initializer (`Type foo = SomeFunc();`) has no
        # '(' before its own first '=', so checking only the pre-'=' segment
        # tells the two apart. Residual gap: a field using call-style
        # brace-init with no '=' at all (`Type foo{SomeFunc()};`) is still
        # misclassified as a method; no such field exists in GuiState today
        # (verified: 62/62 fields extracted, zero false positives/negatives).
        declarator = line.split("=", 1)[0]
        if "(" in declarator:
            continue
        fm = GUI_FIELD_LINE_RE.match(line)
        if fm:
            fields.add(fm.group("name"))
    return fields, open_line


TIERS_TABLE_ENTRY_RE = re.compile(
    r'\{\s*"(?P<name>[A-Za-z_]\w*)"\s*,\s*FieldTier::'
)
TIERS_EXCLUDE_ENTRY_RE = re.compile(r'"(?P<name>[A-Za-z_]\w*)"')


def _extract_tier_table_names(path: Path) -> tuple[set[str], set[str], int]:
    text = path.read_text(encoding="utf-8")
    stripped = strip_comments(text)

    def _extract_body(marker: str) -> tuple[str, int]:
        m = re.search(re.escape(marker) + r"\s*\[\s*\]\s*=\s*\{", stripped)
        if not m:
            return "", -1
        open_line = stripped.count("\n", 0, m.start()) + 1
        depth = 1
        i = m.end()
        while i < len(stripped) and depth > 0:
            c = stripped[i]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    break
            i += 1
        return stripped[m.end():i], open_line

    tier_body, tier_line = _extract_body("kFieldTierTable")
    exclude_body, _ = _extract_body("kDerivedFieldsExcludeList")

    tier_names = {m.group("name") for m in TIERS_TABLE_ENTRY_RE.finditer(tier_body)}
    exclude_names = {m.group("name") for m in TIERS_EXCLUDE_ENTRY_RE.finditer(exclude_body)}
    return tier_names, exclude_names, tier_line


def check_gui_state_field_tier_registration() -> list[Violation]:
    out: list[Violation] = []
    if not GUI_STATE_HPP.exists() or not GUI_STATE_TIERS_HPP.exists():
        return out
    struct_fields, struct_open_line = _extract_gui_state_top_level_fields(GUI_STATE_HPP)
    if not struct_fields:
        out.append(
            Violation(
                GUI_STATE_HPP,
                struct_open_line if struct_open_line > 0 else 1,
                "gui-state-field-tier-registration",
                "could not extract any top-level fields from `struct GuiState` — "
                "the parser likely regressed; investigate before trusting the gate.",
            )
        )
        return out
    tier_names, exclude_names, tiers_open_line = _extract_tier_table_names(GUI_STATE_TIERS_HPP)
    if not tier_names and not exclude_names:
        out.append(
            Violation(
                GUI_STATE_TIERS_HPP,
                tiers_open_line if tiers_open_line > 0 else 1,
                "gui-state-field-tier-registration",
                "kFieldTierTable / kDerivedFieldsExcludeList not found or empty in "
                "gui_state_tiers.hpp; the field-tier registry is missing.",
            )
        )
        return out
    registered = tier_names | exclude_names
    missing = sorted(struct_fields - registered)
    for name in missing:
        out.append(
            Violation(
                GUI_STATE_TIERS_HPP,
                tiers_open_line,
                "gui-state-field-tier-registration",
                f"GuiState field `{name}` is not registered in kFieldTierTable "
                "or kDerivedFieldsExcludeList (src/gui/gui_state_tiers.hpp); "
                "pick a FieldTier or add it to kDerivedFieldsExcludeList "
                "(see doc/gui-state-governance.md).",
            )
        )
    dup = sorted(tier_names & exclude_names)
    for name in dup:
        out.append(
            Violation(
                GUI_STATE_TIERS_HPP,
                tiers_open_line,
                "gui-state-field-tier-registration",
                f"field `{name}` appears in BOTH kFieldTierTable and "
                "kDerivedFieldsExcludeList; a field must be in exactly one.",
            )
        )
    stale = sorted(registered - struct_fields)
    for name in stale:
        out.append(
            Violation(
                GUI_STATE_TIERS_HPP,
                tiers_open_line,
                "gui-state-field-tier-registration",
                f"registered name `{name}` is not a top-level field of "
                "`struct GuiState`; the entry is stale or misspelled.",
            )
        )
    return out


def check_no_msvc_unsafe_builtin() -> list[Violation]:
    out: list[Violation] = []
    for path in cxx_sources(SRC):
        for lineno, _orig, code in code_lines(path):
            if MSVC_UNSAFE_BUILTIN.search(code):
                out.append(
                    Violation(
                        path,
                        lineno,
                        "no-msvc-unsafe-builtin",
                        "`__builtin_popcount`/`ctz`/`clz` are GCC/Clang-only "
                        "(MSVC reports C3861); use `lumice::PopCount` "
                        "(src/util/bit_utils.hpp) or an equivalent portable helper.",
                    )
                )
    test_root = REPO_ROOT / "test"
    if test_root.exists():
        for path in cxx_sources(test_root):
            for lineno, _orig, code in code_lines(path):
                if MSVC_UNSAFE_BUILTIN.search(code):
                    out.append(
                        Violation(
                            path,
                            lineno,
                            "no-msvc-unsafe-builtin",
                            "`__builtin_popcount`/`ctz`/`clz` are GCC/Clang-only "
                            "(MSVC reports C3861); use `lumice::PopCount` "
                            "(src/util/bit_utils.hpp) or an equivalent portable helper.",
                        )
                    )
    return out


# `std::vector<Crystal> name;` / `std::vector<Crystal> name(...)` — the leading
# boundary keeps it from matching `std::vector<CrystalConfig>` and friends.
CRYSTAL_VECTOR_DECL = re.compile(r"std::vector\s*<\s*Crystal\s*>\s*&?\s*([A-Za-z_]\w*)")
# Operations that materialise default-constructed elements. `reserve` is
# deliberately absent: it allocates without constructing, which is exactly the
# pattern this check wants people to use.
CRYSTAL_VECTOR_DEFAULT_FILL = ("resize", "assign")


def check_no_default_constructed_crystal_slots() -> list[Violation]:
    """Flag `std::vector<Crystal>` sites that materialise default-constructed
    elements (single-argument `resize`/`assign`, count-ctor, no-arg
    `emplace_back()`).

    Scope: text-matches direct `std::vector<Crystal> name` declarations only —
    type aliases (`using CrystalVec = std::vector<Crystal>;`) and indirect
    holders (a `std::vector<Crystal>` wrapped in a struct member and exposed
    elsewhere) are not recognised and can bypass this check undetected.

    Any `resize()`/`push_back`-based reshape of a `std::vector<Crystal>` is
    rejected wholesale, even a hypothetical safe `resize()` immediately
    followed by a loop that fills every new index — the win of one uniform,
    mechanically-verifiable rule (no case-by-case safety proof required) is
    judged worth that false-positive rate; push_back/emplace_back(args...) is
    the one sanctioned shape.
    """
    out: list[Violation] = []
    for path in cxx_sources(SRC):
        lines = list(code_lines(path))
        names: set[str] = set()
        for _lineno, _orig, code in lines:
            for m in CRYSTAL_VECTOR_DECL.finditer(code):
                names.add(m.group(1))
        if not names:
            continue
        # A count-ctor on the declaration line itself: `std::vector<Crystal> v(n);`
        # is the same hazard spelled differently. `v(a, b)` (iterator range) and
        # `v(other)` (copy) are fine, so only a single non-empty argument counts.
        for lineno, _orig, code in lines:
            for m in CRYSTAL_VECTOR_DECL.finditer(code):
                tail = code[m.end():].lstrip()
                if tail.startswith("(") and not tail.startswith("()"):
                    inner = tail[1: tail.find(")")] if ")" in tail else ""
                    if inner.strip() and "," not in inner:
                        out.append(
                            Violation(
                                path,
                                lineno,
                                "no-default-constructed-crystal-slots",
                                f"`{m.group(1)}` is sized by a count-ctor, which fills it with "
                                "default-constructed Crystals (face_cnt == 0, no polygon faces). "
                                "Reserve and push_back real crystals instead, so size == populated.",
                            )
                        )
        # Same "no comma == hazard" rule as the count-ctor check above:
        # `resize(n, value)` / `assign(n, value)` / `assign(first, last)` fill
        # with real values and are safe; only the single-argument forms
        # (`resize(n)`; `assign(n)` isn't even valid C++, but a stray
        # single-argument call is still worth flagging) materialise defaults.
        for lineno, _orig, code in lines:
            for name in names:
                for op in CRYSTAL_VECTOR_DEFAULT_FILL:
                    m = re.search(rf"\b{re.escape(name)}\s*\.\s*{op}\s*\(", code)
                    if not m:
                        continue
                    tail = code[m.end():]
                    inner = tail[: tail.find(")")] if ")" in tail else tail
                    if inner.strip() and "," not in inner:
                        out.append(
                            Violation(
                                path,
                                lineno,
                                "no-default-constructed-crystal-slots",
                                f"`{name}.{op}(...)` fills the vector with default-constructed "
                                "Crystals (face_cnt == 0, no polygon faces, fn_period_ == -1). "
                                "A consumer cannot distinguish those slots from a legitimately "
                                "empty crystal. Reserve and push_back real crystals instead.",
                            )
                        )
                if re.search(rf"\b{re.escape(name)}\s*\.\s*emplace_back\s*\(\s*\)", code):
                    out.append(
                        Violation(
                            path,
                            lineno,
                            "no-default-constructed-crystal-slots",
                            f"`{name}.emplace_back()` appends a default-constructed Crystal. "
                            "Pass the crystal to append, or push_back one built by a factory.",
                        )
                    )
    return out


CHECKS = [
    check_getenv_centralization,
    check_env_knob_registration,
    check_gui_api_boundary,
    check_reconciler_widget_include,
    check_no_using_namespace,
    check_struct_layout_parity,
    check_no_config_by_value_copy,
    check_gui_state_field_tier_registration,
    check_no_msvc_unsafe_builtin,
    check_no_default_constructed_crystal_slots,
]


def main() -> int:
    if "--help" in sys.argv or "-h" in sys.argv:
        print(__doc__)
        return 0
    violations: list[Violation] = []
    for check in CHECKS:
        violations.extend(check())
    if violations:
        print("Policy check FAILED:\n", file=sys.stderr)
        for v in sorted(violations, key=lambda x: (str(x.path), x.line)):
            print(v.render(), file=sys.stderr)
        print(
            f"\n{len(violations)} violation(s). See doc/env-var-policy.md and AGENTS.md.",
            file=sys.stderr,
        )
        return 1
    print(
        "Policy check passed (env centralization, knob registration, GUI API boundary, "
        "reconciler-widget-include, using-namespace, struct-layout parity, no-config-by-value-copy, "
        "gui-state-field-tier-registration, no-msvc-unsafe-builtin, "
        "no-default-constructed-crystal-slots)."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
