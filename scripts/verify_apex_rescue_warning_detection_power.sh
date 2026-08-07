#!/usr/bin/env bash
# verify_apex_rescue_warning_detection_power.sh — two-arm A/B verification that
# the near-apex guards in test/golden-analytic/core/test_closed_form_pyramid.cpp
# detect the defect they were written for, and that the degradation warning in
# src/core/geo3d_closedform.cpp actually reaches the log.
#
# Why this exists:
#   ComputeClosedFormPyramid decides "this cone has reached its apex" and "this
#   cone still has a cross section to intersect" from ONE tolerance, which is
#   what makes the band where neither applies structurally empty. Two of the
#   things that guards that binding cannot be observed firing on any legal input
#   any more, by construction:
#     - ClosedFormPyramid.ApexRescueDegradationNeverFiresOnLegalShapes, whose
#       whole assertion is that a counter stays at zero;
#     - the LOG_WARNING inside the degradation branch itself.
#   A guard that can never be seen going red is indistinguishable from one with
#   no teeth. This script makes it observable on demand: it unbinds the gate in
#   a throwaway worktree — restoring the absolute epsilon the binding replaced —
#   rebuilds clean, and confirms both arms behave as claimed.
#
# Discipline (mirrors verify_pyramid_crash_sentinel_detection_power.sh):
#   - Runs the reverted arm in an independent `git worktree`; never touches the
#     caller's working tree, never rewrites the checked-out branch.
#   - Builds that worktree in its OWN build directory, so an "up to date"
#     incremental report can never stand in for a rebuild that did not happen.
#   - Fails loudly if the patch target string is not found, rather than
#     silently testing an unpatched tree.
#
# NOT a CI gate. Manual invocation, from the repo root:
#   ./scripts/verify_apex_rescue_warning_detection_power.sh
#
# Arm A is built from a worktree of HEAD, so it verifies the COMMITTED tree.
# Uncommitted work is not in it — commit first, or arm A silently measures the
# detection power of code that is not the code under test.
#
# Expected outcome:
#   ARM A (unbound gate): NearApexHeightWindowStructuralValidity FAILS,
#     ApexRescueDegradationNeverFiresOnLegalShapes FAILS with a non-zero
#     degraded count, and the captured log it prints contains the warning text
#     naming the offending face_distance.
#   ARM B (as committed): both tests pass.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

WORKTREE="${TMPDIR:-/tmp}/lumice-apex-rescue-detection-$$"
BUILD_DIR="$WORKTREE/build-arm-a"
GEO_SRC="src/core/geo3d_closedform.cpp"
FILTER='ClosedFormPyramid.NearApexHeightWindowStructuralValidity:ClosedFormPyramid.ApexRescueDegradationNeverFiresOnLegalShapes'

cleanup() {
  cd "$REPO_ROOT"
  git worktree remove --force "$WORKTREE" >/dev/null 2>&1 || true
  rm -rf "$WORKTREE"
}
trap cleanup EXIT

echo "=== ARM A: unbinding the apex-collapse gate in a throwaway worktree ==="
git worktree add --detach "$WORKTREE" HEAD >/dev/null
python3 - "$WORKTREE/$GEO_SRC" <<'PY'
import sys

path = sys.argv[1]
src = open(path).read()
# The one line that ties the collapse gate to the cross-section tolerance.
old = "  return (apex_m - m) <= gap_tol;"
new = ("  (void)gap_tol;  // detection-power arm: pre-binding absolute epsilon.\n"
       "  return (apex_m - m) <= 1e-9;")
if old not in src:
    sys.exit("patch target not found in %s — the gate was refactored; update this script" % path)
open(path, "w").write(src.replace(old, new, 1))
PY
echo "  patched $GEO_SRC (gate -> absolute 1e-9)"

cmake -S "$WORKTREE" -B "$BUILD_DIR" -G Ninja -DCMAKE_BUILD_TYPE=Release -DBUILD_TEST=ON >/dev/null
cmake --build "$BUILD_DIR" --target golden_analytic_test >/dev/null
ARM_A_BIN="$(find "$WORKTREE/build" -name golden_analytic_test -type f -perm -u+x | head -1)"
if [ -z "$ARM_A_BIN" ]; then
  echo "FAIL: arm A test binary not found" >&2
  exit 1
fi

set +e
"$ARM_A_BIN" --gtest_filter="$FILTER" 2>&1 | tee "$WORKTREE/arm-a.log"
ARM_A_STATUS=${PIPESTATUS[0]}
set -e

echo
echo "=== ARM B: the tree as committed ==="
cmake --build build/cmake_build/static --target golden_analytic_test >/dev/null
set +e
./build/Release/static/bin/golden_analytic_test --gtest_filter="$FILTER" 2>&1 | tee "$WORKTREE/arm-b.log"
ARM_B_STATUS=${PIPESTATUS[0]}
set -e

echo
echo "=== verdict ==="
FAILED=0
if [ "$ARM_A_STATUS" -eq 0 ]; then
  echo "FAIL: arm A (unbound gate) PASSED — the guards have no detection power"
  FAILED=1
else
  echo "ok: arm A (unbound gate) failed as expected"
fi
if ! grep -q "no cross-section" "$WORKTREE/arm-a.log"; then
  echo "FAIL: arm A never logged the degradation warning"
  FAILED=1
else
  echo "ok: arm A logged the degradation warning:"
  grep -m1 "no cross-section" "$WORKTREE/arm-a.log" | sed 's/^/    /'
fi
if [ "$ARM_B_STATUS" -ne 0 ]; then
  echo "FAIL: arm B (as committed) failed — the fix is not in this tree"
  FAILED=1
else
  echo "ok: arm B (as committed) passed"
fi
grep -h "^\[apex-rescue\] swept=" "$WORKTREE/arm-a.log" "$WORKTREE/arm-b.log" | sed 's/^/    /'

exit "$FAILED"
