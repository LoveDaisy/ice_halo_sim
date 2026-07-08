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
  6. no-config-by-value-copy — LUMICE_Config owns a heap allocation via its
     raypath_color pointer (task-344, v4.8). Copying the struct by value
     (copy-init, direct-init, list-init, by-value parameter, or later-line
     assignment `y = x;`) aliases the same heap block and double-frees on
     double Release. Route through pointer or `const LUMICE_Config&`, and
     manage lifetime via LUMICE_ConfigCreateColorClasses / Release (or
     lumice::ConfigColorGuard).

Add a new check as a function returning a list of Violation and append it to
CHECKS. Keep each check deterministic and artifact-inspecting.
"""
from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path

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
CONFIG_TYPE = "LUMICE_Config"

# Pattern 1a: copy-init  `LUMICE_Config a = b;`
CONFIG_COPY_INIT_RE = re.compile(
    r"\b" + CONFIG_TYPE + r"\s+(\w+)\s*=\s*(\w+)\s*;"
)
# Pattern 1b: direct-init  `LUMICE_Config a(b);`
CONFIG_DIRECT_INIT_RE = re.compile(
    r"\b" + CONFIG_TYPE + r"\s+(\w+)\s*\(\s*(\w+)\s*\)\s*;"
)
# Pattern 1c: list-init  `LUMICE_Config a{b};`  (excludes `LUMICE_Config a{};`
# and multi-field aggregate init `LUMICE_Config a{...,...}` — the initializer
# must be a single bare identifier).
CONFIG_LIST_INIT_RE = re.compile(
    r"\b" + CONFIG_TYPE + r"\s+(\w+)\s*\{\s*(\w+)\s*\}\s*;"
)
# Pattern 2: by-value parameter  in a signature `... LUMICE_Config x, ...`
# Excludes pointer/reference (`LUMICE_Config*` / `LUMICE_Config&`) and `const`
# qualifiers via the "no `*`/`&` immediately before the name" clause. Anchors on
# `,` or `)` following the identifier so we only match parameter positions.
CONFIG_BY_VALUE_PARAM_RE = re.compile(
    r"\b" + CONFIG_TYPE + r"\s+(\w+)\s*[,)]"
)
# Pattern 3: declaration collector  `LUMICE_Config name` (any suffix); the
# alias-guard drops pointers/references separately.
CONFIG_DECL_RE = re.compile(r"\b" + CONFIG_TYPE + r"(\s*[*&]?)\s*(\w+)")


def _is_pointer_or_ref_decl(match: "re.Match[str]") -> bool:
    return "*" in match.group(1) or "&" in match.group(1)


def check_no_config_by_value_copy() -> list[Violation]:
    out: list[Violation] = []
    for path in cxx_sources(SRC):
        out.extend(_scan_config_copies(path))
    test_root = REPO_ROOT / "test"
    if test_root.exists():
        for path in cxx_sources(test_root):
            out.extend(_scan_config_copies(path))
    return out


def _scan_config_copies(path: Path) -> list[Violation]:
    out: list[Violation] = []
    # First pass: collect all `LUMICE_Config <name>` value declarations by name.
    # Excludes pointer/reference declarations. Used only for Pattern 3
    # (later-line bare assignment) to keep the check anchored on same-type
    # locals rather than any arbitrary `a = b`.
    value_decls: set[str] = set()
    for _lineno, _orig, code in code_lines(path):
        for m in CONFIG_DECL_RE.finditer(code):
            if _is_pointer_or_ref_decl(m):
                continue
            value_decls.add(m.group(2))

    for lineno, _orig, code in code_lines(path):
        # Patterns 1a/1b/1c: copy/direct/list initialization from a single
        # existing identifier. All three share the "RHS is one bare
        # identifier" judgement — aggregate init `{}`, `= {}`, `{a, b}`,
        # function call `foo()` are not matched.
        for m in CONFIG_COPY_INIT_RE.finditer(code):
            lhs, rhs = m.group(1), m.group(2)
            if lhs == rhs:
                continue
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"copy-init `{CONFIG_TYPE} {lhs} = {rhs};` aliases the "
                    "raypath_color heap allocation; use a pointer or reference.",
                )
            )
        for m in CONFIG_DIRECT_INIT_RE.finditer(code):
            lhs, rhs = m.group(1), m.group(2)
            if lhs == rhs:
                continue
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"direct-init `{CONFIG_TYPE} {lhs}({rhs});` aliases the "
                    "raypath_color heap allocation; use a pointer or reference.",
                )
            )
        for m in CONFIG_LIST_INIT_RE.finditer(code):
            lhs, rhs = m.group(1), m.group(2)
            if lhs == rhs:
                continue
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"list-init `{CONFIG_TYPE} {lhs}{{{rhs}}};` aliases the "
                    "raypath_color heap allocation; use a pointer or reference.",
                )
            )
        # Pattern 2: by-value function parameter. The `\s+` after LUMICE_Config
        # already excludes `LUMICE_Config*name` / `LUMICE_Config&name`
        # (adjacent `*`/`&` binds to the type token with no space). Explicit
        # spaced variants like `LUMICE_Config * name` or `... & name` still
        # match Pattern 2 as a false positive — the plan accepts this because
        # the codebase uses adjacent `*`/`&` universally and the spaced form
        # would violate the local style anyway.
        for m in CONFIG_BY_VALUE_PARAM_RE.finditer(code):
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
            # Skip trivial `LUMICE_Config x` where x is followed by `[` in
            # e.g. `LUMICE_Config x[]` — that's an array parameter, not a
            # by-value scalar. The regex ends on `[,)]` so an array param
            # `LUMICE_Config x[]` would already not match (the `]` is not `,`
            # or `)`), no extra guard needed.
            name = m.group(1)
            out.append(
                Violation(
                    path,
                    lineno,
                    "no-config-by-value-copy",
                    f"by-value parameter `{CONFIG_TYPE} {name}` aliases the "
                    "raypath_color heap allocation; use "
                    f"`const {CONFIG_TYPE}&` or `{CONFIG_TYPE}*`.",
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
                        # `LUMICE_Config` does not immediately precede name_lhs
                        # on this line.
                        decl_here = re.compile(
                            r"\b" + CONFIG_TYPE + r"\s+" + re.escape(name_lhs) + r"\b"
                        )
                        if decl_here.search(code):
                            continue
                        out.append(
                            Violation(
                                path,
                                lineno,
                                "no-config-by-value-copy",
                                f"assignment `{name_lhs} = {name_rhs};` between two "
                                f"{CONFIG_TYPE} locals aliases the raypath_color "
                                "heap allocation; use a pointer/reference.",
                            )
                        )
    return out


CHECKS = [
    check_getenv_centralization,
    check_env_knob_registration,
    check_gui_api_boundary,
    check_no_using_namespace,
    check_struct_layout_parity,
    check_no_config_by_value_copy,
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
        "using-namespace, struct-layout parity, no-config-by-value-copy)."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
