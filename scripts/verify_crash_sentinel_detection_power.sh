#!/usr/bin/env bash
# verify_crash_sentinel_detection_power.sh — two-arm A/B verification of the
# random-face_distance SIGSEGV regression sentinel
# (test/regression-sentinel/test_face_distance_crash.py).
#
# Purpose:
#   Confirm the sentinel actually detects the crash it is written to catch —
#   i.e. that reverting the fix commits + rebuilding clean + rerunning the
#   sentinel reliably turns it red. This guards against the "build reported
#   exit 0 but was up-to-date and never rebuilt" trap that has silently voided
#   past two-arm verifications.
#
# Discipline:
#   - Runs both arms in independent `git worktree`s (never touches the caller's
#     working tree). No `git revert` on the checked-out branch.
#   - Forces a clean rebuild (`rm -rf build/`) on each arm — never trust an
#     incremental "up-to-date" report as evidence a rebuild happened.
#   - Records md5 of both arm binaries and refuses to proceed if they match.
#   - Reports the sentinel-red count from the reverted arm (N=15 → expected
#     ≥1 signal death at 20% baseline crash rate; probability of 0/15 under
#     that baseline is ~3.5%).
#
# NOT a CI gate. Manual invocation only, when a suspicion arises that the
# sentinel may have lost detection power (e.g. after a refactor touching the
# fix commits, or after a session-level environment change).
#
# Usage: from the repo root
#   ./scripts/verify_crash_sentinel_detection_power.sh
#
# Options via env:
#   SENTINEL_REVERT_BASE  — commit to check out for the reverted arm (default
#                          ab7aa0b5, parent of the first fix commit 23a63386
#                          on branch fix/degenerate-geometry). Set to override
#                          if the fix has been rebased onto a different base.
#   SENTINEL_N            — sentinel run count per arm (default 15).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

REVERT_BASE="${SENTINEL_REVERT_BASE:-ab7aa0b5}"
N_RUNS="${SENTINEL_N:-15}"

FIXED_HEAD="$(git rev-parse HEAD)"

# Refuse to run if the caller has uncommitted tracked-file changes; the
# last-line "worktree clean" invariant only holds if the tree started clean.
# Untracked/ignored files are fine — check only tracked-file diffs.
if ! git diff --quiet HEAD --; then
  echo "ERROR: working tree has uncommitted tracked-file changes. Commit or stash first." >&2
  exit 1
fi

# Independent worktree paths — outside REPO_ROOT so cmake's `build/` under each
# worktree does not collide with the caller's build dir.
WORK_PARENT="$(mktemp -d "${TMPDIR:-/tmp}/lumice_sentinel_verify_XXXXXX")"
FIXED_WORKTREE="$WORK_PARENT/fixed"
REVERTED_WORKTREE="$WORK_PARENT/reverted"

cleanup() {
  echo "--- cleanup: removing worktrees ---"
  git worktree remove --force "$FIXED_WORKTREE" 2>/dev/null || true
  git worktree remove --force "$REVERTED_WORKTREE" 2>/dev/null || true
  rm -rf "$WORK_PARENT"
  # Prune stale worktree registrations.
  git worktree prune
  echo "cleanup done. Main worktree at $REPO_ROOT was never modified."
}
trap cleanup EXIT

echo "=== fixed arm (HEAD=$FIXED_HEAD) ==="
git worktree add --detach "$FIXED_WORKTREE" "$FIXED_HEAD"
(
  cd "$FIXED_WORKTREE"
  rm -rf build
  ./scripts/build.sh -sj release >/dev/null
)
FIXED_BIN="$FIXED_WORKTREE/build/cmake_install/Lumice"
FIXED_LIB="$(find "$FIXED_WORKTREE/build/cmake_install" -name 'liblumice*' -print -quit)"
FIXED_MD5="$(md5 -q "$FIXED_BIN" 2>/dev/null || md5sum "$FIXED_BIN" | awk '{print $1}')"
echo "fixed arm binary: $FIXED_BIN"
echo "fixed arm md5:    $FIXED_MD5"

echo "=== reverted arm (base=$REVERT_BASE) ==="
git worktree add --detach "$REVERTED_WORKTREE" "$REVERT_BASE"
(
  cd "$REVERTED_WORKTREE"
  rm -rf build
  ./scripts/build.sh -sj release >/dev/null
)
REVERTED_BIN="$REVERTED_WORKTREE/build/cmake_install/Lumice"
REVERTED_LIB="$(find "$REVERTED_WORKTREE/build/cmake_install" -name 'liblumice*' -print -quit)"
REVERTED_MD5="$(md5 -q "$REVERTED_BIN" 2>/dev/null || md5sum "$REVERTED_BIN" | awk '{print $1}')"
echo "reverted arm binary: $REVERTED_BIN"
echo "reverted arm md5:    $REVERTED_MD5"

if [ "$FIXED_MD5" = "$REVERTED_MD5" ]; then
  echo "ERROR: fixed and reverted arm binaries have IDENTICAL md5 — one of the arms" >&2
  echo "was not actually rebuilt. This is the 'up-to-date' trap the two-arm" >&2
  echo "protocol exists to catch. Refusing to draw any conclusion about detection" >&2
  echo "power." >&2
  exit 1
fi

echo "=== reverted arm: sentinel run (N=$N_RUNS) ==="
# Run the *current-HEAD* sentinel test (which has the signal-vs-clean-exit
# discipline improvements) against the reverted-arm binary. This is the point
# of two-arm testing: the sentinel must fire on the reverted arm.
set +e
LUMICE_BIN="$REVERTED_BIN" LUMICE_LIB="$FIXED_LIB" \
  pytest -v "test/regression-sentinel/test_face_distance_crash.py" \
  --tb=short 2>&1 | tee "$WORK_PARENT/reverted_arm.log"
SENTINEL_EXIT=${PIPESTATUS[0]}
set -e

# Count failures classified as "signal-death" (SIGSEGV-class) vs any other exit.
SIGNAL_DEATH_COUNT=$(grep -c "signal-death" "$WORK_PARENT/reverted_arm.log" || true)
CLEAN_NONZERO_COUNT=$(grep -c "clean non-zero exit" "$WORK_PARENT/reverted_arm.log" || true)

echo ""
echo "=== summary ==="
echo "fixed arm md5:      $FIXED_MD5"
echo "reverted arm md5:   $REVERTED_MD5"
echo "reverted arm N:     $N_RUNS"
echo "signal deaths:      $SIGNAL_DEATH_COUNT"
echo "clean non-zero:     $CLEAN_NONZERO_COUNT"
echo "pytest exit:        $SENTINEL_EXIT"

if [ "$SIGNAL_DEATH_COUNT" -ge 1 ]; then
  echo "PASS: reverted arm exhibits signal death — sentinel detection power confirmed."
  exit 0
fi

if [ "$CLEAN_NONZERO_COUNT" -ge 1 ] && [ "$SIGNAL_DEATH_COUNT" -eq 0 ]; then
  echo "INCONCLUSIVE: reverted arm exited non-zero but not from a signal —" >&2
  echo "this looks like an infrastructure failure (binary won't run), not the" >&2
  echo "SIGSEGV the sentinel is written to catch. Investigate the reverted-arm" >&2
  echo "build output at $REVERTED_WORKTREE/build/ before drawing conclusions" >&2
  echo "about sentinel detection power." >&2
  exit 2
fi

# 0 signal deaths in N=15 at a 20% base rate ≈ P=0.035 — small but not
# negligible. Do not soften-message this: report the probability and stop
# short of concluding "sentinel is broken" without more data.
echo "AMBIGUOUS: 0/$N_RUNS signal deaths on the reverted arm." >&2
echo "At the 20% pre-fix baseline crash rate (377.1 measurement, 4/20)," >&2
echo "P(0 crashes in $N_RUNS runs) = 0.8^$N_RUNS ≈ 3.5% — small but not" >&2
echo "impossible. Rerun with SENTINEL_N=$((N_RUNS * 2)) before concluding" >&2
echo "the sentinel has lost detection power." >&2
exit 3
