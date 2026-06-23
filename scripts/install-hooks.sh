#!/bin/sh
#
# Install Lumice's local git hooks into .git/hooks. Currently installs the
# pre-commit hook (policy checks + clang-format on staged files). The existing
# git-lfs hooks (post-checkout / post-commit / post-merge / pre-push) are left
# untouched — we only add pre-commit, which git-lfs does not use.
#
# Run once after cloning:  ./scripts/install-hooks.sh
# Bypass the hook for one commit:  git commit --no-verify

set -e
REPO_ROOT=$(git rev-parse --show-toplevel)
HOOK_SRC="$REPO_ROOT/scripts/hooks/pre-commit"
HOOK_DST="$REPO_ROOT/.git/hooks/pre-commit"

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

echo "Done. The pre-commit hook runs scripts/check_policies.py + clang-format on staged sources."
