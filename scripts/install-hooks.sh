#!/bin/sh
#
# Install Lumice's local git hooks into THIS REPOSITORY's hooks directory:
# `<git-common-dir>/hooks` (not assumed to be .git/hooks — in a git worktree,
# .git is a file, not a directory, and hooks live in the main checkout's
# .git/hooks, shared across all worktrees). Deliberately NOT
# `git rev-parse --git-path hooks`: that honours `core.hooksPath`, and with a
# *global* hooksPath (a per-user hook chain) it names a directory outside the
# repository — the previous version of this script overwrote the user's global
# `pre-commit` there with a symlink that dangled. A repo hook belongs in the
# repo; if a global hooksPath is set, git only runs the repo hook when the
# global one chains back to `<git-common-dir>/hooks`, or when the repo sets
# `core.hooksPath` locally to its own `.git/hooks` — this script says so and
# leaves that choice to the user rather than editing config.
# Currently installs the pre-commit hook (main-worktree guard + policy checks +
# clang-format on staged files). The existing git-lfs hooks (post-checkout /
# post-commit / post-merge / pre-push) are left untouched — we only add
# pre-commit, which git-lfs does not use.
#
# Run once after cloning:  ./scripts/install-hooks.sh
# Bypass the hook for one commit:  git commit --no-verify

set -e
REPO_ROOT=$(git rev-parse --show-toplevel)
HOOK_SRC="$REPO_ROOT/scripts/hooks/pre-commit"
HOOK_DIR="$(git -C "$REPO_ROOT" rev-parse --path-format=absolute --git-common-dir)/hooks"
HOOK_DST="$HOOK_DIR/pre-commit"
mkdir -p "$HOOK_DIR"

GLOBAL_HOOKS_PATH=$(git config --global --get core.hooksPath || true)
LOCAL_HOOKS_PATH=$(git -C "$REPO_ROOT" config --local --get core.hooksPath || true)
if [ -n "$GLOBAL_HOOKS_PATH" ] && [ -z "$LOCAL_HOOKS_PATH" ]; then
  echo "note: a global core.hooksPath is set ($GLOBAL_HOOKS_PATH); git will run the hook installed" >&2
  echo "      below only if that directory chains back to $HOOK_DIR, or if you point this" >&2
  echo "      repo at its own hooks:  git config core.hooksPath \"$HOOK_DIR\"" >&2
fi

chmod +x "$HOOK_SRC"

# Prefer a symlink so the hook tracks updates to scripts/hooks/pre-commit; fall
# back to a copy where symlinks are unavailable (e.g. some Windows setups).
if ln -sf "../../scripts/hooks/pre-commit" "$HOOK_DST" 2>/dev/null; then
  echo "Linked pre-commit hook -> scripts/hooks/pre-commit"
else
  cp "$HOOK_SRC" "$HOOK_DST"
  chmod +x "$HOOK_DST"
  echo "Copied pre-commit hook (symlink unavailable on this platform)."
fi

echo "Done. The pre-commit hook runs the main-worktree guard (scripts/hooks/worktree-guard.py),"
echo "scripts/check_policies.py + the diff-scoped checkers, and clang-format on staged sources."
