#!/usr/bin/env bash
# verify_pyramid_crash_sentinel_detection_power.sh — two-arm A/B verification of
# the pyramid+random-face_distance SIGSEGV regression sentinel
# (test/regression-sentinel/test_pyramid_geometry_crash.py).
#
# Purpose:
#   Confirm the sentinel detects the crash it is written to catch — i.e. that
#   reverting the fix commit + rebuilding clean + rerunning the sentinel
#   reliably turns it red on hardware that reproduces the SIGSEGV. Guards
#   against the "build reported exit 0 but was up-to-date and never rebuilt"
#   trap that has silently voided past two-arm verifications.
#
# Discipline:
#   - Runs both arms in independent `git worktree`s (never touches the caller's
#     working tree). No `git revert` on the checked-out branch.
#   - Forces a clean rebuild (`rm -rf build/`) on each arm — never trust an
#     incremental "up-to-date" report as evidence a rebuild happened.
#   - Records md5 of both arm binaries and refuses to proceed if they match.
#   - Reports the reverted arm's signal-death count.
#
# NOT a CI gate. Manual invocation only, when a suspicion arises that the
# sentinel may have lost detection power (e.g. after a refactor touching the
# fix commit, or after a session-level environment change).
#
# Usage: from the repo root
#   ./scripts/verify_pyramid_crash_sentinel_detection_power.sh
#
# Options via env:
#   SENTINEL_REVERT_BASE  — commit to check out for the reverted arm (default
#                          =  the parent of HEAD, i.e. the state before the fix
#                          commit landed). Set to override if the fix has been
#                          rebased onto a different base.
#   SENTINEL_N            — sentinel run count per arm (default 15). At the
#                          calibrated fixture the shrink branch fires ~6 times
#                          per run, so N=15 samples ~90 shrink events — enough
#                          for a reasonable Bernoulli signal at the observed
#                          Mac-native crash rate (which may be zero: the
#                          historical 4/4 signal deaths were reported on
#                          different hardware; the sentinel is still valuable
#                          on Mac because the anti-vacuous WARN assertion
#                          keeps it honest).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# Default REVERT_BASE = the pre-fix tree state = this branch's merge-base with
# origin/main. Do NOT use HEAD~1: the crystal.cpp fix is the FIRST commit on
# this branch while the test/config/this-script land in LATER commits, so HEAD~1
# still contains the fix and the "reverted" arm would silently compile fixed
# code (false detection power — the exact HEAD~N off-by-one this task's plan
# forbade). Override with SENTINEL_REVERT_BASE for other tree layouts (e.g. a
# rebased/merged fix).
if [ -n "${SENTINEL_REVERT_BASE:-}" ]; then
  REVERT_BASE="$SENTINEL_REVERT_BASE"
else
  REVERT_BASE="$(git merge-base HEAD origin/main 2>/dev/null || true)"
  if [ -z "$REVERT_BASE" ]; then
    echo "ERROR: could not derive REVERT_BASE (git merge-base HEAD origin/main failed)." >&2
    echo "Pass SENTINEL_REVERT_BASE=<pre-fix commit> explicitly." >&2
    exit 1
  fi
fi
N_RUNS="${SENTINEL_N:-15}"

FIXED_HEAD="$(git rev-parse HEAD)"

# Refuse to run if the caller has uncommitted tracked-file changes.
if ! git diff --quiet HEAD --; then
  echo "ERROR: working tree has uncommitted tracked-file changes. Commit or stash first." >&2
  exit 1
fi

WORK_PARENT="$(mktemp -d "${TMPDIR:-/tmp}/lumice_pyramid_sentinel_verify_XXXXXX")"
FIXED_WORKTREE="$WORK_PARENT/fixed"
REVERTED_WORKTREE="$WORK_PARENT/reverted"

cleanup() {
  local exit_code="$1"
  echo "--- cleanup: removing worktrees ---"
  git worktree remove --force "$FIXED_WORKTREE" 2>/dev/null || true
  if [ "$exit_code" -eq 2 ] || [ "$exit_code" -eq 4 ]; then
    git worktree prune
    echo "cleanup: exit $exit_code — retaining reverted arm worktree for inspection:" >&2
    echo "  $REVERTED_WORKTREE" >&2
    echo "  (remove manually with: git worktree remove --force '$REVERTED_WORKTREE' && rm -rf '$WORK_PARENT')" >&2
    return
  fi
  git worktree remove --force "$REVERTED_WORKTREE" 2>/dev/null || true
  rm -rf "$WORK_PARENT"
  git worktree prune
  echo "cleanup done. Main worktree at $REPO_ROOT was never modified."
}
trap 'cleanup $?' EXIT

echo "=== fixed arm (HEAD=$FIXED_HEAD) ==="
git worktree add --detach "$FIXED_WORKTREE" "$FIXED_HEAD"
(
  cd "$FIXED_WORKTREE"
  rm -rf build
  ./scripts/build.sh -sj release >/dev/null
)
FIXED_BIN="$FIXED_WORKTREE/build/cmake_install/Lumice"
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
# Run the current-HEAD sentinel against the reverted-arm binary — the sentinel
# code has the signal-vs-clean-exit classification the script's grep relies on,
# and reverting the fixture along with the fix would defeat the point of the
# A/B test. Pytest picks up the current-HEAD sentinel from the caller's repo.
set +e
SENTINEL_N="$N_RUNS" LUMICE_BIN="$REVERTED_BIN" \
  pytest -v "test/regression-sentinel/test_pyramid_geometry_crash.py" \
  --tb=short 2>&1 | tee "$WORK_PARENT/reverted_arm.log"
SENTINEL_EXIT=${PIPESTATUS[0]}
set -e

if grep -q "Lumice binary infrastructure check failed" "$WORK_PARENT/reverted_arm.log"; then
  echo "" >&2
  echo "INFRASTRUCTURE ERROR: reverted arm's module-scope smoke check failed —" >&2
  echo "the reverted-arm binary could not even run a known-good config. The" >&2
  echo "parametrized sentinel loop never executed. Inspect" >&2
  echo "  $REVERTED_WORKTREE/build/" >&2
  echo "before drawing any conclusion." >&2
  exit 4
fi

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

# The pre-fix crash rate is hardware-dependent — the historical 4/4 SIGSEGV was
# observed at a different site and Metal driver combination; on some hardware
# the wild read stays within the mapped allocation and the process completes
# cleanly (still delivering silent-wrong output on CPU, and possibly wild
# centroid data on Metal that the shader tolerates). 0/N signal deaths on such
# hardware is expected — but that does NOT mean the sentinel is worthless: it
# still catches (a) any hardware/driver combo that DOES reproduce the crash,
# and (b) any regression that removes the fixture's shrink-branch coverage
# (via the anti-vacuous WARN assertion in the sentinel itself).
echo "AMBIGUOUS: 0/$N_RUNS signal deaths on the reverted arm." >&2
echo "This does NOT necessarily mean the sentinel is broken — pre-fix crashes" >&2
echo "were hardware-dependent. Rerun with SENTINEL_N=$((N_RUNS * 4)) on hardware" >&2
echo "known to reproduce the SIGSEGV before concluding the sentinel has lost" >&2
echo "detection power. On hardware that never reproduces, the sentinel's" >&2
echo "anti-vacuous WARN assertion (in the sentinel itself, not this script)" >&2
echo "still provides meaningful coverage of the fixed code path." >&2
exit 3
