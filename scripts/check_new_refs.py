#!/usr/bin/env python3
"""Diff-scoped policy check: new prose must not cite per-task working notes.

Companion to check_policies.py, deliberately kept as a SECOND entry point rather
than a tenth check inside it: the two ask different questions. check_policies.py
asks "what state is the repo in?" (whole-tree, deterministic). This one asks
"what did you just write?" (added lines of one diff). Mixing the two shapes
behind one command would make `python3 scripts/check_policies.py` mean different
things locally and in CI.

The rule
--------
The repo keeps its in-flight reasoning in a git-ignored working-notes tree, one
directory per task, archived once the task closes. Prose in a *tracked* file is
read long after that — by someone who never had those notes on disk. So a
tracked comment that points at them ("see the plan doc", "answers review round
1", "measured in <task-name> E6") resolves to nothing: the reader cannot check
what the comment claims, and the comment ends up costing more credibility than
it adds. Write the mechanism itself instead, self-contained.

Permanent anchors are exempt and should stay: PR numbers, commit hashes,
file:line, code symbol names — those survive in git forever.

Why diff-scoped
---------------
The rule has never held historically: an inventory found ~2400 pre-existing
citations across 41% of tracked files. A whole-tree gate could not start green,
and a frozen baseline would be a long-lived asset to maintain. Scanning only
added lines starts green for free, needs no baseline, and matches how the rule
actually gets broken — every recorded breach was newly written prose.

Scope
-----
Only natural-language regions are scanned: comments in every language, Python
docstrings, and the whole of Markdown. String literals are program data, not
prose, and are skipped — test-case names and identifiers legitimately carry
task-shaped tokens.

Excluded paths (EXCLUDED_PATHS) hold to one criterion: a file is exempt only if
its *subject* is the working-notes system itself, i.e. it must name the system
to do its job at all. That is the agent-instruction entry points plus the ignore
rule that makes the directory git-ignored. Citing the system is not a reason to
be exempt — that is the very thing being checked. Note this is deliberately
narrower than a per-file allowlist: nested .gitignore files, for instance, stay
in scope, because they cite artifacts rather than declare the ignore rule.

Usage
-----
  python3 scripts/check_new_refs.py --staged          # pre-commit: the index
  python3 scripts/check_new_refs.py --range A..B      # CI: PR vs merge-base

Exit code 0 when clean, 1 when any added prose line matches.
"""
from __future__ import annotations

import argparse
import io
import re
import subprocess
import sys
import tokenize
from pathlib import Path, PurePosixPath

sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_policies import REPO_ROOT, Violation, strip_comments  # noqa: E402

RULE = "no-working-note-refs"

# Files whose subject IS the working-notes system: they must name it to work.
# Root-relative and exact — a nested .gitignore is not the ignore rule for the
# notes tree, it is a normal tracked file that can only be citing artifacts.
EXCLUDED_PATHS = frozenset({"AGENTS.md", "CLAUDE.md", ".gitignore"})

# Each entry: (regex, what the reader should be told it matched).
#
# The task-id regex requires either a number (`<kind>-367`, `<kind>-358.1`) or a
# multi-word slug (`<kind>-some-long-name`). A bare single word after the prefix
# is NOT matched: across all 364 distinct such tokens in the tree, the six
# single-word ones are English compounds ("<kind>-specific") and workflow-command
# names ("<kind>-drive") — i.e. false positives — while all 184 multi-word slugs
# are real task names. Recall loss is ~1 real citation; the trade is deliberate,
# because this gate blocks commits and has no inline exemption, which makes a
# false positive far more expensive than a miss.
#
# A `.` is only consumed when digits follow, so a sentence-final period is not
# swallowed into the token.
PATTERNS: list[tuple[re.Pattern[str], str]] = [
    (
        # Word-boundary lookbehind, not a path one: a relative-path prefix in
        # front of the directory name must still match, while a longer word that
        # merely ends in the directory name must not.
        re.compile(r"(?<!\w)scratchpad/"),
        "a path into the git-ignored working-notes tree",
    ),
    (
        # The number branch deliberately has no trailing \b, unlike the two
        # below. Letter-suffixed phase labels (`<kind>-3b`, `<kind>-3c`) are a
        # real citation form here: a \b would drop 10 of them across the tree,
        # all in the shape this gate targets ("host-only until <kind>-3c"). It
        # reports the numeric stem as the matched token, which is enough — the
        # file:line points at the line. Nothing is gained in exchange: the audit
        # of 364 real tokens turned up no `<kind>-<digits><letters>` false
        # positive, so the guard would cost recall and buy no precision.
        re.compile(
            r"\b(?:task|explore|scrum|chore)-"
            r"(?:\d+(?:\.\d+)*|[A-Za-z0-9]+(?:-[A-Za-z0-9]+)+(?:\.\d+)*)"
        ),
        "a task identifier",
    ),
    (
        # Trailing \b, and here it is load-bearing in the other direction: the
        # rule's own prose names the filename pattern with an `N` placeholder
        # (`code-review-0N`), which an unguarded `\d+` matches via the `0`.
        # That wording is how anyone documents this rule, so without the guard
        # the gate fires on any file explaining itself. Unlike the branch above
        # there is no letter-suffixed round to lose: rounds are numbered.
        re.compile(r"\bcode-review-\d+\b"),
        "a review-round note filename",
    ),
    (
        re.compile(
            r"\b(?:progress|plan|issue|hypothesis|experiments|insights"
            r"|SUMMARY|backlog|tasks)\.md\b"
        ),
        "a per-task note filename",
    ),
    (
        re.compile(r"\bround-\d+\b"),
        "a review-round pointer",
    ),
]

FIX_HINT = (
    "Rewrite it as the mechanism itself, self-contained "
    "(what is true and why, not where it was decided). "
    "Permanent anchors may stay: PR number, commit hash, file:line, symbol name."
)

C_FAMILY_SUFFIXES = frozenset(
    {".c", ".cc", ".cpp", ".cxx", ".h", ".hpp", ".hxx", ".inl", ".m", ".mm", ".cu", ".cuh", ".metal"}
)
HASH_COMMENT_SUFFIXES = frozenset({".sh", ".bash", ".ps1", ".yml", ".yaml", ".toml", ".cfg", ".cmake"})
HASH_COMMENT_NAMES = frozenset({"CMakeLists.txt", ".clang-tidy", ".gitignore", "pre-commit"})
MARKDOWN_SUFFIXES = frozenset({".md"})

HUNK_RE = re.compile(r"^@@ -\d+(?:,\d+)? \+(\d+)(?:,\d+)? @@")


def _git(args: list[str]) -> str:
    proc = subprocess.run(
        ["git", "-c", "core.quotePath=false", *args],
        cwd=REPO_ROOT,
        capture_output=True,
        check=True,
    )
    return proc.stdout.decode("utf-8", errors="replace")


def _blob(ref: str, rel: str) -> str:
    """Content of `rel` at `ref`; empty ref means the index."""
    spec = f"{ref}:{rel}" if ref else f":{rel}"
    proc = subprocess.run(["git", "show", spec], cwd=REPO_ROOT, capture_output=True)
    if proc.returncode != 0:
        return ""
    return proc.stdout.decode("utf-8", errors="replace")


def added_lines(diff_args: list[str]) -> dict[str, set[int]]:
    """Map path -> line numbers (in the NEW file) added by the diff.

    -U0 drops context, so every non-header line inside a hunk is a `+` or a `-`;
    only `+` lines advance the new-file counter.

    Rename detection is forced on (-M, not left to diff.renames) and R kept in
    the filter. Without it, `git mv` of an untouched file is reported as a whole
    delete plus a whole add, so every historical line of the moved file arrives
    here as "added" — moving a file would demand its pre-existing prose be
    cleaned up first, which is the one thing this gate exists not to do. With
    rename detection a pure move emits no hunks at all, and a move-plus-edit
    emits only the lines actually rewritten.
    """
    out = _git(["diff", "-U0", "--no-color", "-M", "--diff-filter=ACMR", *diff_args])
    result: dict[str, set[int]] = {}
    path: str | None = None
    lineno = 0
    after_old_header = False
    for line in out.splitlines():
        # A `+++ ` header only counts directly after its `--- ` partner. An added
        # line of a Markdown file that quotes a patch verbatim starts with `++ `
        # and would otherwise be read as a header, rerouting the rest of the hunk
        # to whatever path it names.
        if after_old_header and line.startswith("+++ "):
            target = line[4:]
            path = None if target == "/dev/null" else target[2:] if target.startswith("b/") else target
        elif line.startswith("@@"):
            m = HUNK_RE.match(line)
            if m:
                lineno = int(m.group(1))
        elif path and line.startswith("+"):
            result.setdefault(path, set()).add(lineno)
            lineno += 1
        after_old_header = line.startswith("--- ")
    return result


def _c_prose(text: str) -> dict[int, str]:
    """Comment text per line, derived from strip_comments' offset-preserving output.

    strip_comments blanks comment characters to spaces and copies everything else
    verbatim, keeping offsets 1:1. So a position that is a space in the stripped
    text but not in the original is exactly a comment character — no second
    comment parser needed.
    """
    stripped = strip_comments(text)
    out: dict[int, str] = {}
    for idx, (orig, code) in enumerate(zip(text.splitlines(), stripped.splitlines()), 1):
        prose = "".join(
            oc if (sc == " " and oc != " ") else " " for oc, sc in zip(orig, code)
        ).strip()
        if prose:
            out[idx] = prose
    return out


def _find_hash_comment(line: str) -> str | None:
    quote: str | None = None
    for i, ch in enumerate(line):
        if quote is not None:
            if ch == quote:
                quote = None
        elif ch in "'\"":
            quote = ch
        elif ch == "#":
            return line[i:]
    return None


def _hash_prose(text: str) -> dict[int, str]:
    out: dict[int, str] = {}
    for idx, line in enumerate(text.splitlines(), 1):
        comment = _find_hash_comment(line)
        if comment:
            out[idx] = comment
    return out


_DOCSTRING_PREDECESSORS = frozenset(
    {tokenize.NEWLINE, tokenize.INDENT, tokenize.DEDENT, tokenize.ENCODING}
)


def _python_prose(text: str) -> dict[int, str]:
    """Comments plus docstrings. A STRING that opens a logical line is a docstring
    (module/class/function) or a bare string expression — either way it is prose.
    Every other STRING is program data and stays out of scope."""
    try:
        toks = list(tokenize.generate_tokens(io.StringIO(text).readline))
    except (tokenize.TokenError, IndentationError, SyntaxError):
        return _hash_prose(text)  # unparsable (e.g. a partial edit): comments only
    src = text.splitlines()
    out: dict[int, str] = {}
    prev_type: int | None = None
    for tok in toks:
        if tok.type == tokenize.COMMENT:
            out[tok.start[0]] = out.get(tok.start[0], "") + " " + tok.string
        elif tok.type == tokenize.STRING and prev_type in _DOCSTRING_PREDECESSORS.union({None}):
            for ln in range(tok.start[0], tok.end[0] + 1):
                if 0 < ln <= len(src):
                    out[ln] = out.get(ln, "") + " " + src[ln - 1]
        if tok.type not in (tokenize.NL, tokenize.COMMENT):
            prev_type = tok.type
    return out


def prose_by_line(rel: str, text: str) -> dict[int, str]:
    """Natural-language regions of a file, keyed by line number.

    Unknown file types yield nothing rather than everything: the whole-file
    reading would fire on identifiers and JSON payloads.
    """
    p = PurePosixPath(rel)
    if p.suffix in MARKDOWN_SUFFIXES:
        return {i: line for i, line in enumerate(text.splitlines(), 1) if line.strip()}
    if p.suffix == ".py":
        return _python_prose(text)
    if p.suffix in C_FAMILY_SUFFIXES:
        return _c_prose(text)
    if p.suffix in HASH_COMMENT_SUFFIXES or p.name in HASH_COMMENT_NAMES:
        return _hash_prose(text)
    return {}


def check(diff_args: list[str], content_ref: str) -> tuple[list[Violation], int]:
    violations: list[Violation] = []
    scanned = 0
    for rel, linenos in sorted(added_lines(diff_args).items()):
        if rel in EXCLUDED_PATHS:
            continue
        text = _blob(content_ref, rel)
        if not text:
            continue
        prose = prose_by_line(rel, text)
        for lineno in sorted(linenos):
            line = prose.get(lineno)
            if not line:
                continue
            scanned += 1
            for pattern, what in PATTERNS:
                m = pattern.search(line)
                if m:
                    violations.append(
                        Violation(
                            REPO_ROOT / rel,
                            lineno,
                            RULE,
                            f"new prose cites {what} (`{m.group(0)}`), which is "
                            f"git-ignored and archived per task — the reference "
                            f"dangles for every later reader. {FIX_HINT}",
                        )
                    )
                    break
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

    if args.rng:
        if ".." not in args.rng:
            parser.error("--range expects BASE..HEAD")
        base, head = args.rng.split("..", 1)
        if not base or not head:
            parser.error("--range expects BASE..HEAD")
        diff_args, content_ref = [base, head], head
    else:
        # --staged, passed or defaulted to. Empty content_ref means the index.
        diff_args, content_ref = ["--cached"], ""

    violations, scanned = check(diff_args, content_ref)
    if violations:
        print("New-reference check FAILED:\n", file=sys.stderr)
        for v in sorted(violations, key=lambda x: (str(x.path), x.line)):
            print(v.render(), file=sys.stderr)
        print(
            f"\n{len(violations)} violation(s) on newly added lines. "
            "See the rule in AGENTS.md; pre-existing lines are out of scope.",
            file=sys.stderr,
        )
        return 1
    print(f"New-reference check passed ({scanned} added prose line(s) scanned).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
