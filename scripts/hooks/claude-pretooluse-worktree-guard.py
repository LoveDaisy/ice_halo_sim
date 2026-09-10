#!/usr/bin/env python3
"""Claude Code PreToolUse hook: refuse Edit/Write under src/ or test/ in the MAIN worktree.

This is a Claude Code hook, not a git hook. It lives next to `pre-commit` so
that both gates on "what may be changed, and from where" sit in one place; the
`claude-` filename prefix is what tells them apart. The logic is tracked here
on purpose: `.claude/settings.local.json`, which enables the hook per machine,
is git-ignored and therefore invisible to code review. Settings only say "run
this file"; everything reviewable is in this file.

The rule it enforces (stated in full in AGENTS.md, "Collaboration Constraints")
-----------------------------------------------------------------------------
Production code is changed on a task branch in a linked git worktree, never in
the main worktree. The main worktree is shared by every task running on the
machine, so an edit made there is mixed into whichever diff happens to be
staged next. The rule applies at N=1: a single task still gets its own
worktree, because "nothing else is running right now" is a judgement the
editor makes, and the incidents this gate exists for were exactly that
judgement being wrong.

Why a hook and not a sentence in AGENTS.md
------------------------------------------
Both incidents that motivated this were committed by an agent that had read
the rule. A rule whose trigger is "when you judge that you are working on a
task" is bypassed by never making that judgement; this hook attaches the check
to the *action* (the first Edit/Write of a production file) instead, where no
judgement is involved. That is also why there is no escape hatch here — no
environment variable, no sentinel file. If the rule must be broken, the owner
says so out loud and the friction stays visible.

Decision procedure
------------------
1. Read the tool call from stdin (JSON: `tool_name`, `tool_input.file_path`,
   `cwd`). Only Edit and Write carry `file_path`; anything else passes.
2. Resolve the target path and find the git worktree it belongs to.
   Not inside a git repository → pass.
3. Main-worktree test, using git's own structural definition rather than a
   directory name: `git rev-parse --git-common-dir`, resolved, equals
   `<toplevel>/.git` only for the main worktree. A linked worktree's common
   dir points at the main repository's `.git`, so the equality fails there.
4. Path test: the first path component below the toplevel is `src` or `test`.
5. Both true → deny, with a message that says what to do instead. Otherwise
   → pass. Every other outcome (malformed input, git missing) is reported on
   stderr with exit 1, which Claude Code treats as a non-blocking error: the
   edit proceeds, but the user sees that the guard did not run.

Output protocol: a deny is a JSON object on stdout with
`hookSpecificOutput.permissionDecision = "deny"` and exit 0. A pass is exit 0
with no output. Exit 2 is deliberately NOT used for the deny, because it is
also what python3 itself returns when the script path does not exist — keeping
the two distinct means "the guard said no" and "the guard is not installed" can
be told apart from the exit code alone.

Coverage boundary, stated so nobody reads this as a complete fence: the hook
sees the structured Edit and Write tool calls and nothing else. A file written
from a Bash call (shell redirection, `sed -i`, a script) never passes through
it. That is the shape of every guard built on tool-call hooks, and it is
accepted here because the incidents being closed were all Edit/Write.

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


def _warn(msg: str) -> int:
    print(f"worktree-guard: {msg} (guard did not run; edit not blocked)", file=sys.stderr)
    return 1


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


def deny_message(target_rel: str, toplevel: str) -> str:
    return (
        f"Refusing to edit `{target_rel}` in the MAIN worktree `{toplevel}`.\n"
        "\n"
        "Production code (src/, test/) is changed on a task branch in a linked git worktree, "
        "never in the main worktree — it is shared by every task on this machine, and N=1 is "
        "not an exception. Before editing:\n"
        "  1. Make sure this change has a task directory under `scratchpad/`. "
        "No directory → create it first, do not write code first.\n"
        "  2. Create a linked worktree for the task and work there. The recipe (naming, the "
        "`scratchpad` symlink, what the shared pre-commit hook does inside a worktree) is written "
        "once, in AGENTS.md → \"Collaboration Constraints\" → \"Where a change lives, and from where "
        "it is made\"; read it there rather than from memory.\n"
        "  3. Re-issue this edit against the file inside that worktree.\n"
        "\n"
        "This guard has no override switch by design (scripts/hooks/claude-pretooluse-worktree-guard.py). "
        "If the owner genuinely wants a one-off edit in the main worktree, they say so explicitly and "
        "you proceed with a different path — the friction is meant to be visible."
    )


def main() -> int:
    raw = sys.stdin.read()
    try:
        payload = json.loads(raw) if raw.strip() else {}
    except json.JSONDecodeError as exc:
        return _warn(f"stdin is not JSON ({exc})")

    tool_name = payload.get("tool_name")
    if tool_name not in GUARDED_TOOLS:
        return 0

    tool_input = payload.get("tool_input") or {}
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
        common_dir = _git(anchor, "rev-parse", "--git-common-dir")
    except GitUnavailable as exc:
        return _warn(f"cannot run git ({exc})")
    if common_dir is None:
        return _warn("git rev-parse --git-common-dir failed")

    toplevel_real = os.path.realpath(toplevel)
    common_real = os.path.realpath(os.path.join(anchor, common_dir))
    is_main_worktree = common_real == os.path.join(toplevel_real, ".git")
    if not is_main_worktree:
        return 0

    # realpath both sides so a symlinked path to the file compares against the
    # same canonical toplevel; the anchor is the deepest existing directory, so
    # re-attach the not-yet-existing tail after resolving it.
    tail = os.path.relpath(target, anchor)
    target_real = os.path.normpath(os.path.join(os.path.realpath(anchor), tail))
    rel = os.path.relpath(target_real, toplevel_real)
    if rel.startswith(os.pardir):
        return 0
    first = rel.split(os.sep, 1)[0]
    if first not in GUARDED_TOP_DIRS:
        return 0

    print(
        json.dumps(
            {
                "hookSpecificOutput": {
                    "hookEventName": "PreToolUse",
                    "permissionDecision": "deny",
                    "permissionDecisionReason": deny_message(rel, toplevel_real),
                }
            }
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
