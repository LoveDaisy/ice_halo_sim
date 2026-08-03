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
  local exit_code="$1"
  echo "--- cleanup: removing worktrees ---"
  # exit 2 (INCONCLUSIVE) and exit 4 (INFRASTRUCTURE ERROR) both point the user
  # at an arm's build tree or build log to explain the verdict — deleting them
  # here would make that instruction unfollowable. Keep both worktrees and
  # $WORK_PARENT (which holds {fixed,reverted}_build.log and the pytest log) on
  # those two paths; every other path cleans up as before. Both arms are kept,
  # not just the reverted one: a build failure can land on either arm, and the
  # failing arm is exactly the one worth inspecting.
  if [ "$exit_code" -eq 2 ] || [ "$exit_code" -eq 4 ]; then
    git worktree prune
    echo "cleanup: exit $exit_code — retaining worktrees and logs for inspection:" >&2
    echo "  $WORK_PARENT" >&2
    echo "  (remove manually with:" >&2
    echo "     git worktree remove --force '$FIXED_WORKTREE' 2>/dev/null" >&2
    echo "     git worktree remove --force '$REVERTED_WORKTREE' 2>/dev/null" >&2
    echo "     rm -rf '$WORK_PARENT')" >&2
    return
  fi
  git worktree remove --force "$FIXED_WORKTREE" 2>/dev/null || true
  git worktree remove --force "$REVERTED_WORKTREE" 2>/dev/null || true
  rm -rf "$WORK_PARENT"
  # Prune stale worktree registrations.
  git worktree prune
  echo "cleanup done. Main worktree at $REPO_ROOT was never modified."
}
trap 'cleanup $?' EXIT

# Build one arm, keeping the full build log on disk.
#
# stdout goes to $WORK_PARENT/<arm>_build.log (a clean-rebuild log is thousands
# of compiler lines and would bury the verdict); stderr stays on the terminal so
# a failure is visible while it happens. The two halves are deliberately NOT
# merged: the terminal keeps the short version, the file keeps the long one.
#
# On failure this exits 4 rather than letting `set -e` propagate build.sh's own
# exit code. That is the whole point of the log: any code other than 2 or 4
# sends cleanup() down the delete-everything path, which would remove
# $WORK_PARENT — taking the log that was just written down with it, and the
# worktree with it, leaving nothing to diagnose but terminal scrollback.
build_arm() {
  local arm="$1" worktree="$2"
  local log="$WORK_PARENT/${arm}_build.log"
  echo "building $arm arm — stdout → $log (stderr stays on this terminal)"
  if ! (
    cd "$worktree"
    rm -rf build
    ./scripts/build.sh -sj release >"$log"
  ); then
    echo "" >&2
    echo "INFRASTRUCTURE ERROR: $arm arm failed to build." >&2
    echo "No conclusion about sentinel detection power can be drawn from this run" >&2
    echo "— the sentinel never ran. Retained for inspection:" >&2
    echo "  build log: $log" >&2
    echo "  worktree:  $worktree" >&2
    exit 4
  fi
}

echo "=== fixed arm (HEAD=$FIXED_HEAD) ==="
git worktree add --detach "$FIXED_WORKTREE" "$FIXED_HEAD"
build_arm fixed "$FIXED_WORKTREE"
FIXED_BIN="$FIXED_WORKTREE/build/cmake_install/shared/Lumice"
FIXED_MD5="$(md5 -q "$FIXED_BIN" 2>/dev/null || md5sum "$FIXED_BIN" | awk '{print $1}')"
echo "fixed arm binary: $FIXED_BIN"
echo "fixed arm md5:    $FIXED_MD5"

echo "=== reverted arm (base=$REVERT_BASE) ==="
git worktree add --detach "$REVERTED_WORKTREE" "$REVERT_BASE"
build_arm reverted "$REVERTED_WORKTREE"
REVERTED_BIN="$REVERTED_WORKTREE/build/cmake_install/shared/Lumice"
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
#
# LUMICE_BIN is the only override this test path consumes (see
# test/e2e/runner.py::find_lumice_binary) — the CLI binary is statically
# linked against liblumice in this build (confirmed via `otool -L`/`ldd`: no
# liblumice entry in the shared-library list), so there is no companion
# shared-library path to pin here. Do NOT introduce a LUMICE_LIB override on
# this invocation: capi_runner.py-style tests read it, but this sentinel goes
# through the CLI subprocess path, which does not — setting it here would
# silently do nothing (or, if runner.py ever grows LUMICE_LIB support later,
# risk re-introducing a cross-arm-library pairing bug where the reverted
# binary loads the fixed arm's library).
set +e
SENTINEL_N="$N_RUNS" LUMICE_BIN="$REVERTED_BIN" \
  pytest -v "test/regression-sentinel/test_face_distance_crash.py" \
  --tb=short 2>&1 | tee "$WORK_PARENT/reverted_arm.log"
SENTINEL_EXIT=${PIPESTATUS[0]}
set -e

# Infrastructure failures (the module-scope smoke-check fixture erroring out)
# report as pytest ERROR, not FAILED, and never reach _classify_exit — so they
# contain neither "signal-death" nor "clean non-zero exit". Detect this first
# so it is never miscounted as "0 signal deaths observed" (AMBIGUOUS below);
# it means the reverted arm never got to run the parametrized loop at all.
# Covers both shapes of "the binary cannot run": a non-zero exit and a hang.
#
# Anchor: this literal is the single source `_INFRA_ANCHOR` in
# test_face_distance_crash.py, which carries a matching comment pointing back
# here. Change one side without the other and this grep silently stops firing.
if grep -q "Lumice binary infrastructure check failed" "$WORK_PARENT/reverted_arm.log"; then
  echo "" >&2
  echo "INFRASTRUCTURE ERROR: reverted arm's module-scope smoke check failed —" >&2
  echo "the reverted-arm binary could not even run a known-good config. The" >&2
  echo "parametrized sentinel loop never executed, so 0 signal-deaths here does" >&2
  echo "NOT mean 'no detection power' — it means the reverted-arm build itself" >&2
  echo "is broken. Inspect $REVERTED_WORKTREE/build/ before drawing any" >&2
  echo "conclusion." >&2
  exit 4
fi

# Count failures classified as "signal-death" (SIGSEGV-class) vs any other exit.
#
# Anchor: these literals ("signal-death", "clean non-zero exit") are only
# produced by test_face_distance_crash.py::_classify_exit(). That function
# carries a matching anchor comment pointing back here. If either literal
# changes on one side without the other, this script silently falls through
# to AMBIGUOUS/INCONCLUSIVE regardless of what the reverted arm actually did.
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

# Second net, for infrastructure failures that never reach the anchored
# assertion above: a collection/import error, `find_lumice_binary()` raising,
# an unhandled exception in the fixture, or a hang in the *parametrized* loop
# (whose `subprocess.TimeoutExpired` surfaces as pytest FAILED, not ERROR).
# None of those produce "signal-death" or "clean non-zero exit" either, so
# without this they land in the AMBIGUOUS branch below and get reported as
# "could be luck — rerun with a bigger N" when nothing ran at all.
#
# Deliberately placed AFTER the PASS branch: an error in one test must not
# retract detection power that another test actually demonstrated.
#
# The patterns match pytest's own report structure, not free text — the
# short-test-summary line (`ERROR <path>.py::<nodeid> - ...`), the final
# counts line (`=== N errors in 1.23s ===`), and the stdlib exception name a
# hang prints verbatim. Lumice's own log lines cannot collide: spdlog emits
# `[timestamp] [logger] [level] msg`, so they never start with `ERROR `.
if grep -qE '^ERROR .*\.py|[0-9]+ errors? in |subprocess\.TimeoutExpired' \
     "$WORK_PARENT/reverted_arm.log"; then
  echo "" >&2
  echo "INFRASTRUCTURE ERROR: the reverted-arm pytest run reported errors (or a" >&2
  echo "timeout), not test failures — so the parametrized sentinel loop did not" >&2
  echo "complete. 0 signal deaths here does NOT mean 'no detection power'; it" >&2
  echo "means this run produced no evidence either way. Offending lines:" >&2
  grep -nE '^ERROR .*\.py|[0-9]+ errors? in |subprocess\.TimeoutExpired' \
    "$WORK_PARENT/reverted_arm.log" | head -20 >&2
  echo "Full log: $WORK_PARENT/reverted_arm.log" >&2
  echo "Inspect $REVERTED_WORKTREE/build/ before drawing any conclusion." >&2
  exit 4
fi

# 0/N signal deaths does not by itself mean "sentinel has lost detection
# power" — at any nonzero crash rate it can happen by chance for small N.
# Do not soften-message this: stop short of concluding "sentinel is broken"
# without more data.
echo "AMBIGUOUS: 0/$N_RUNS signal deaths on the reverted arm." >&2
echo "This could be luck rather than a broken sentinel. Rerun with" >&2
echo "SENTINEL_N=$((N_RUNS * 2)) before concluding the sentinel has lost" >&2
echo "detection power." >&2
exit 3
