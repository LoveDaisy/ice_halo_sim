#!/usr/bin/env python3
r"""Main-worktree guard for src/ and test/: one predicate, two entry points.

    python3 scripts/hooks/worktree-guard.py pre-commit          # from the git pre-commit hook
    python3 scripts/hooks/worktree-guard.py claude-pretooluse   # from a Claude Code PreToolUse hook
    python3 scripts/hooks/worktree-guard.py print-settings      # the canonical settings.local.json
    python3 scripts/hooks/worktree-guard.py check-settings      # is this checkout's settings file current?

The rule it enforces (stated in full in AGENTS.md, "Collaboration Constraints")
-----------------------------------------------------------------------------
Production code is changed on a task branch in a linked git worktree, never in
the main worktree. The main worktree is shared by every task running on the
machine, so an edit made there is mixed into whichever diff happens to be
staged next. The rule applies at N=1: a single task still gets its own
worktree, because "nothing else is running right now" is a judgement the
editor makes, and the incidents this gate exists for were exactly that
judgement being wrong.

Why the commit is the gate, and the tool hook only a second line
----------------------------------------------------------------
The first version of this guard was a PreToolUse hook alone. It was correct
and it would not have caught the incident that motivated it: that edit was
written through Bash (`python3 - <<'PY'`), and a tool-call hook sees only the
structured Edit/Write calls — a file written by shell redirection, `sed -i`,
`tee`, a heredoc or an inline script never passes through it. Widening the
hook to Bash would mean parsing command lines for write operations, which is a
heuristic with both misses and false alarms. Every write, whatever produced
it, converges on `git commit` instead — and the failure shape being closed
("edit a lot, then squeeze it into one big commit") lands there by
construction. So the authoritative gate is the `pre-commit` entry point,
wired into `scripts/hooks/pre-commit` ahead of the four policy checkers; the
`claude-pretooluse` entry point is kept as defence in depth because it fires
at the first keystroke rather than at the end, and costs nothing once the
predicate is shared. Both use the ONE predicate below; do not re-implement it
in shell in the hook file.

Honest boundary of the commit gate: it cannot see a change that is never
committed. Such a change also never reaches the repository, so its blast
radius is a different one — a dirty main worktree, not a polluted history —
and that is the residue this design accepts. The escape is git's own explicit
`git commit --no-verify`, not a new switch: if the owner wants a one-off
commit from the main worktree they say so, and the flag is visible in the
shell history.

Main-worktree test
------------------
git's own structural definition rather than a directory name:
`git rev-parse --git-common-dir`, resolved, equals `<toplevel>/.git` only for
the main worktree. A linked worktree's common dir points at the main
repository's `.git`, so the equality fails there. (A `--separate-git-dir`
layout breaks this equality in the main worktree too and would read as
linked; that layout is not used here.)

`pre-commit` mode
-----------------
Run from inside the repository (the hook has already `cd`'d to the toplevel).
Staged paths come from `git diff --cached --name-only`; if any of them has
`src` or `test` as its first component AND the toplevel is the main worktree,
print the refusal on stderr and exit 1 — the hook maps that to "commit
blocked". Everything else exits 0.

`claude-pretooluse` mode
------------------------
Read the tool call from stdin (JSON: `tool_name`, `tool_input.file_path`,
`cwd`). Only Edit and Write carry `file_path`; anything else passes. Resolve
the target, find the worktree it belongs to (not in a repository → pass),
apply the same main-worktree test and the same first-component test. A deny
is a JSON object on stdout with `hookSpecificOutput.permissionDecision =
"deny"` and exit 0; a pass is exit 0 with no output. Malformed input or an
unrunnable git is reported on stderr with exit 1, which Claude Code treats as
a non-blocking error: the edit proceeds, but the user sees that the guard did
not run. Exit 2 is deliberately NOT used for the deny, because it is also what
python3 itself returns when the script path does not exist — keeping the two
distinct means "the guard said no" and "the guard is not installed" can be
told apart from the exit code alone.

Enabling the PreToolUse entry point (per machine, git-ignored)
--------------------------------------------------------------
`.claude/settings.local.json` in the MAIN checkout — the one worktree this
guard is about; a linked worktree needs no settings because everything is
allowed there. The canonical content is `SETTINGS_SNIPPET` below, printed by
`print-settings`; it is a Python constant rather than prose so that a change
to it is a diff a reviewer sees. Its command tests for the script before
running it. That is the lesson of the first enablement: settings were
switched on while this file still lived on an unmerged branch, and a command
that simply ran `python3 <missing path>` returned exit 2 — which Claude Code
reads as a BLOCKING error — so every Edit/Write in the main worktree was
refused, not just src/ and test/. A missing script now degrades *loudly* (one
stderr line, exit 1, edit proceeds) rather than either silently (exit 0, the
gate is invisibly off) or catastrophically (exit 2). Loud degradation is the
permitted kind: the user can read on screen that the guard is not installed
in this checkout.

The settings file is git-ignored, so no diff can show whether a machine's
copy still matches this script — and a stale copy is exactly how the first
enablement failed (it named a path that did not exist). `check-settings`
closes that gap mechanically: it reads the checkout's settings file, finds
the PreToolUse command, and compares it byte-for-byte with the snippet.
`scripts/install-hooks.sh` runs it last, so the one-time install step after
a clone or a merge also says whether the Claude side is current, absent, or
stale; the exit code is what the caller reads (0 current or absent-in-a-
linked-worktree, 1 stale or absent-in-the-main-worktree).

This script must stay runnable on its own: standard library only, no repo
imports, no private tooling.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys

GUARDED_TOP_DIRS = frozenset({"src", "test"})
GUARDED_TOOLS = frozenset({"Edit", "Write"})
# Section-level pointer on purpose: a top-level heading is a stabler anchor than
# the bullet's own title, and the bullet is the only one in that section about
# worktrees, so the reader lands on it either way.
RULE_LOCATION = 'AGENTS.md, "Collaboration Constraints"'

SETTINGS_RELPATH = os.path.join(".claude", "settings.local.json")
# The command is a plain string, not the JSON file's decoded form, so the
# comparison in `check-settings` is byte-for-byte and needs no normalisation.
SETTINGS_COMMAND = (
    'f="$CLAUDE_PROJECT_DIR/scripts/hooks/worktree-guard.py"; '
    'if [ -f "$f" ]; then python3 "$f" claude-pretooluse; '
    'else echo "worktree-guard: $f not in this checkout (guard did not run; edit not blocked)" >&2; exit 1; fi'
)
SETTINGS_SNIPPET = {
    "hooks": {
        "PreToolUse": [
            {
                "matcher": "Edit|Write",
                "hooks": [
                    {
                        "type": "command",
                        "command": SETTINGS_COMMAND,
                        "timeout": 10,
                        "statusMessage": "worktree guard: checking edit target",
                    }
                ],
            }
        ]
    }
}



class GitUnavailable(Exception):
    """git itself could not be run (not on PATH, not executable).

    Kept distinct from "git ran and said no": the latter means the target is
    not in a repository and there is nothing to guard, the former means the
    guard cannot do its job and must say so instead of silently passing.
    """


def _git(cwd: str, *args: str) -> str | None:
    try:
        out = subprocess.run(
            ["git", "-C", cwd, *args],
            capture_output=True,
            text=True,
            check=False,
        )
    except OSError as exc:
        raise GitUnavailable(str(exc)) from exc
    if out.returncode != 0:
        return None
    return out.stdout.strip()


def is_main_worktree(anchor: str, toplevel: str) -> bool | None:
    """The one predicate. None means git could not answer (caller reports it)."""
    common_dir = _git(anchor, "rev-parse", "--git-common-dir")
    if common_dir is None:
        return None
    toplevel_real = os.path.realpath(toplevel)
    common_real = os.path.realpath(os.path.join(anchor, common_dir))
    return common_real == os.path.join(toplevel_real, ".git")


def is_guarded_path(rel: str) -> bool:
    # Segment-exact on the first component, both for "outside the toplevel"
    # and for the guarded set: a prefix test would let a directory literally
    # named `..something` under the toplevel read as outside it and slip past,
    # and `srcfoo/` must not read as `src/`.
    first = rel.split(os.sep, 1)[0]
    return first != os.pardir and first in GUARDED_TOP_DIRS


# --------------------------------------------------------------------------
# pre-commit entry point
# --------------------------------------------------------------------------


def commit_refusal(paths: list[str], toplevel: str) -> str:
    listed = "\n".join(f"    {p}" for p in paths)
    return (
        f"pre-commit: refusing to commit src/ or test/ changes from the MAIN worktree `{toplevel}`:\n"
        f"{listed}\n"
        "\n"
        "Production code is committed on a task branch in a linked git worktree, never in the\n"
        "main worktree — it is shared by every task on this machine, and N=1 is not an exception.\n"
        "  1. Make sure this change has a task directory (the task ledger is the authority).\n"
        "  2. Create a linked worktree for the task and move the change there, e.g.\n"
        "       git diff --cached | git -C ../<worktree> apply --index\n"
        f"     The worktree recipe is written once, in {RULE_LOCATION}.\n"
        "  3. Commit from inside that worktree.\n"
        "\n"
        "This gate has no switch of its own. For an owner-authorized one-off commit from the main\n"
        "worktree use git's explicit escape, `git commit --no-verify`, so the exception is visible."
    )


def run_pre_commit() -> int:
    cwd = os.getcwd()
    try:
        toplevel = _git(cwd, "rev-parse", "--show-toplevel")
        if toplevel is None:
            print("worktree-guard: not inside a git repository; nothing to check", file=sys.stderr)
            return 0
        in_main = is_main_worktree(cwd, toplevel)
        # -z: NUL-separated and unquoted, so a path that core.quotePath would
        # otherwise wrap in quotes and escape cannot slip past the first-
        # component test by arriving as `"src/..."`.
        staged = _git(cwd, "diff", "--cached", "--name-only", "-z", "--diff-filter=ACMRDT")
    except GitUnavailable as exc:
        print(f"worktree-guard: cannot run git ({exc}); refusing to guess", file=sys.stderr)
        return 1
    if in_main is None or staged is None:
        print("worktree-guard: git rev-parse/diff failed; refusing to guess", file=sys.stderr)
        return 1
    if not in_main:
        return 0
    # `git diff --name-only` prints paths relative to the toplevel with `/`
    # separators regardless of platform; normalise so the first-component
    # test sees the native separator.
    hits = [p for p in staged.split("\0") if p and is_guarded_path(os.path.normpath(p))]
    if not hits:
        return 0
    print(commit_refusal(hits, os.path.realpath(toplevel)), file=sys.stderr)
    return 1


# --------------------------------------------------------------------------
# claude-pretooluse entry point
# --------------------------------------------------------------------------


def _warn(msg: str) -> int:
    print(f"worktree-guard: {msg} (guard did not run; edit not blocked)", file=sys.stderr)
    return 1


def _nearest_existing_dir(path: str) -> str:
    # Write may target a file (or directory chain) that does not exist yet;
    # git needs a real directory to answer from.
    d = path if os.path.isdir(path) else os.path.dirname(path)
    while d and not os.path.isdir(d):
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return d or os.sep


def edit_refusal(target_rel: str, toplevel: str) -> str:
    return (
        f"Refusing to edit `{target_rel}` in the MAIN worktree `{toplevel}`.\n"
        "\n"
        "Production code (src/, test/) is changed on a task branch in a linked git worktree, "
        "never in the main worktree — it is shared by every task on this machine, and N=1 is "
        "not an exception. Before editing:\n"
        "  1. Make sure this change has a task directory (the task ledger is the authority). "
        "No directory → create it first, do not write code first.\n"
        "  2. Create a linked worktree for the task and work there. The recipe (naming, the "
        "shared pre-commit hook's behaviour inside a worktree) is written once, in "
        f"{RULE_LOCATION}; read it there rather than from memory.\n"
        "  3. Re-issue this edit against the file inside that worktree.\n"
        "\n"
        "This guard has no override switch by design (scripts/hooks/worktree-guard.py); the same "
        "rule is enforced again at `git commit`, where a Bash-written file is caught too. If the "
        "owner genuinely wants a one-off edit in the main worktree, they say so explicitly and "
        "you proceed with a different path — the friction is meant to be visible."
    )


def run_claude_pretooluse() -> int:
    raw = sys.stdin.read()
    try:
        payload = json.loads(raw) if raw.strip() else {}
    except json.JSONDecodeError as exc:
        return _warn(f"stdin is not JSON ({exc})")

    tool_name = payload.get("tool_name")
    if tool_name not in GUARDED_TOOLS:
        return 0

    tool_input = payload.get("tool_input")
    if not isinstance(tool_input, dict):
        return _warn(f"{tool_name} call carries no tool_input object")
    file_path = tool_input.get("file_path")
    if not isinstance(file_path, str) or not file_path:
        return _warn(f"{tool_name} call carries no tool_input.file_path")

    base = payload.get("cwd") or os.getcwd()
    target = file_path if os.path.isabs(file_path) else os.path.join(base, file_path)
    target = os.path.normpath(target)

    anchor = _nearest_existing_dir(target)
    try:
        toplevel = _git(anchor, "rev-parse", "--show-toplevel")
        if toplevel is None:
            # git ran and found no repository above the target: nothing to guard.
            return 0
        in_main = is_main_worktree(anchor, toplevel)
    except GitUnavailable as exc:
        return _warn(f"cannot run git ({exc})")
    if in_main is None:
        return _warn("git rev-parse --git-common-dir failed")
    if not in_main:
        return 0

    # realpath both sides so a symlinked path to the file compares against the
    # same canonical toplevel; the anchor is the deepest existing directory, so
    # re-attach the not-yet-existing tail after resolving it.
    toplevel_real = os.path.realpath(toplevel)
    tail = os.path.relpath(target, anchor)
    target_real = os.path.normpath(os.path.join(os.path.realpath(anchor), tail))
    rel = os.path.relpath(target_real, toplevel_real)
    if not is_guarded_path(rel):
        return 0

    print(
        json.dumps(
            {
                "hookSpecificOutput": {
                    "hookEventName": "PreToolUse",
                    "permissionDecision": "deny",
                    "permissionDecisionReason": edit_refusal(rel, toplevel_real),
                }
            }
        )
    )
    return 0


# --------------------------------------------------------------------------
# settings entry points
# --------------------------------------------------------------------------


def run_print_settings() -> int:
    print(json.dumps(SETTINGS_SNIPPET, indent=2))
    return 0


def _installed_commands(settings_path: str) -> list[str] | None:
    """Every PreToolUse command in the file that names this script; None if unreadable."""
    try:
        with open(settings_path, encoding="utf-8") as fh:
            data = json.load(fh)
    except (OSError, json.JSONDecodeError):
        return None
    found: list[str] = []
    for entry in (data.get("hooks") or {}).get("PreToolUse") or []:
        for hook in (entry or {}).get("hooks") or []:
            cmd = (hook or {}).get("command")
            if isinstance(cmd, str) and "worktree-guard" in cmd:
                found.append(cmd)
    return found


def run_check_settings() -> int:
    cwd = os.getcwd()
    try:
        toplevel = _git(cwd, "rev-parse", "--show-toplevel")
        if toplevel is None:
            print("worktree-guard: not inside a git repository", file=sys.stderr)
            return 1
        in_main = is_main_worktree(cwd, toplevel)
    except GitUnavailable as exc:
        print(f"worktree-guard: cannot run git ({exc})", file=sys.stderr)
        return 1
    if in_main is None:
        print("worktree-guard: git rev-parse --git-common-dir failed", file=sys.stderr)
        return 1
    path = os.path.join(toplevel, SETTINGS_RELPATH)
    where = "main worktree" if in_main else "linked worktree"
    if not os.path.exists(path):
        if in_main:
            print(
                f"worktree-guard: {SETTINGS_RELPATH} absent in the {where} — the PreToolUse line is "
                f"not enabled here. Write it from `{os.path.basename(sys.argv[0])} print-settings`.",
                file=sys.stderr,
            )
            return 1
        print(f"worktree-guard: {SETTINGS_RELPATH} absent in the {where}; none needed there.")
        return 0
    cmds = _installed_commands(path)
    if cmds is None:
        print(f"worktree-guard: {path} is not readable JSON", file=sys.stderr)
        return 1
    if cmds == [SETTINGS_COMMAND]:
        print(f"worktree-guard: {SETTINGS_RELPATH} is current ({where}).")
        return 0
    if not cmds:
        print(
            f"worktree-guard: {path} exists but carries no PreToolUse command naming this script; "
            f"merge in the output of `print-settings`.",
            file=sys.stderr,
        )
        return 1
    print(
        f"worktree-guard: {path} carries a STALE PreToolUse command for this script — this is the "
        f"shape that once blocked every edit in the main worktree. Replace it with the output of "
        f"`print-settings`.\n  installed: {cmds}\n  expected:  [{SETTINGS_COMMAND!r}]",
        file=sys.stderr,
    )
    return 1


MODES = {
    "pre-commit": run_pre_commit,
    "claude-pretooluse": run_claude_pretooluse,
    "print-settings": run_print_settings,
    "check-settings": run_check_settings,
}


def main(argv: list[str]) -> int:
    if len(argv) != 2 or argv[1] not in MODES:
        print(
            f"usage: {os.path.basename(argv[0])} {{{'|'.join(MODES)}}}",
            file=sys.stderr,
        )
        return 2
    return MODES[argv[1]]()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
