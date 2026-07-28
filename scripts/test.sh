#!/usr/bin/env bash
#
# test.sh — one entry point for the three local test scopes.
#
#   quick : ctest + fast e2e + gui_test correctness pool
#   full  : quick + gui_test real-timing pool
#   pr    : full + shared-lib freshness + slow e2e (the CI e2e-slow shape)
#
# Two contracts this script owes its caller, both learned from how the scopes
# used to be assembled by hand:
#
#  1. A layer's command is run in the FOREGROUND, never through a pipe. `cmd |
#     tee log` and `cmd > log 2>&1` hand back the pipeline's last exit status (or
#     hide it in a file), which is how a finished run still needs a second call
#     just to learn pass/fail. Every layer's status is read straight from `$?`
#     and reprinted in one summary table with a single RESULT line, so one
#     invocation is enough to judge the outcome.
#  2. Layers do not fail fast: a failing layer does not stop the ones after it.
#     Fail-fast would recreate the same problem from the other side ("run it
#     again to find out whether the rest is broken too"). The single exception is
#     the slow e2e layer, which is skipped — with the cause spelled out in the
#     summary — when the shared library it loads could not be built.
#
# Style note: build.sh takes combinable flags (`-tgj release`) because its
# options compose. test.sh takes one subcommand because the three scopes are
# mutually exclusive nested supersets, not switches you mix.
#
# This script runs tests; it does not build the static flavor. `quick`/`full`
# expect `./scripts/build.sh -tgj <type>` to have been run already, and say so
# when it has not. `pr` is the one scope that may build, and only the shared
# flavor, because the slow e2e layer loads liblumice through ctypes.

set -u
set -o pipefail

ROOT_DIR=$(cd "$(dirname "$0")/.."; pwd)
cd "${ROOT_DIR}" || exit 2

# Exit codes: 0 = every layer PASS/SKIP, 1 = at least one layer FAIL,
# 2 = the run never started (bad argument or an unmet precondition).
EXIT_USAGE=2

usage() {
  echo "Usage:"
  echo "  ./scripts/test.sh <quick|full|pr>"
  echo ""
  echo "SCOPES (each one is a superset of the previous):"
  echo "  quick:  ctest (unit-correctness|parity|golden-analytic)"
  echo "          + fast e2e (pytest -n auto)"
  echo "          + gui_test correctness pool (--fixed-dt)"
  echo "  full:   quick + gui_test real-timing pool"
  echo "  pr:     full + shared-lib freshness check + slow e2e"
  echo ""
  echo "ENVIRONMENT:"
  echo "  LUMICE_SKIP_GUI_TESTS / CI:  skip the gui_test layers (as build.sh does)."
  echo "  LUMICE_LIB:                  absolute path to liblumice; when set, the"
  echo "                               pr scope trusts it and never rebuilds."
  echo ""
  echo "Builds the static flavor? No — run ./scripts/build.sh -tgj <type> first."
}

die() {
  printf 'test.sh: %s\n' "$1" >&2
  exit "${EXIT_USAGE}"
}

# ---------------------------------------------------------------------------
# Layer bookkeeping
# ---------------------------------------------------------------------------

LAYER_NAMES=()
LAYER_STATUS=()
LAYER_SECS=()
LAYER_NOTES=()

record_layer() { # name status seconds note
  LAYER_NAMES+=("$1")
  LAYER_STATUS+=("$2")
  LAYER_SECS+=("$3")
  LAYER_NOTES+=("$4")
}

# Appends to the note of the most recently recorded layer. Used to attach a
# readable hint to a raw tool failure without swallowing the tool's own output.
annotate_last_layer() {
  local last=$(( ${#LAYER_NOTES[@]} - 1 ))
  if [[ ${last} -ge 0 ]]; then
    if [[ -n ${LAYER_NOTES[${last}]} ]]; then
      LAYER_NOTES[${last}]="${LAYER_NOTES[${last}]}; $1"
    else
      LAYER_NOTES[${last}]="$1"
    fi
  fi
}

run_layer() { # name command...
  local name=$1
  shift
  local start=${SECONDS}
  local rc=0
  printf '\n========== [RUN] %s ==========\n' "${name}"
  "$@" || rc=$?
  local elapsed=$(( SECONDS - start ))
  if [[ ${rc} -eq 0 ]]; then
    record_layer "${name}" PASS "${elapsed}" ""
  else
    record_layer "${name}" FAIL "${elapsed}" "exit=${rc}"
  fi
  return ${rc}
}

skip_layer() { # name note
  printf '\n========== [SKIP] %s (%s) ==========\n' "$1" "$2"
  record_layer "$1" SKIP 0 "$2"
}

print_summary() { # scope
  local total=${#LAYER_NAMES[@]}
  local width=0 i
  for (( i = 0; i < total; i++ )); do
    if [[ ${#LAYER_NAMES[${i}]} -gt ${width} ]]; then width=${#LAYER_NAMES[${i}]}; fi
  done

  local failed=0
  local failed_names=""
  printf '\n========== SUMMARY (scope: %s) ==========\n' "$1"
  for (( i = 0; i < total; i++ )); do
    printf '[%s] %-*s %5ss' "${LAYER_STATUS[${i}]}" "${width}" "${LAYER_NAMES[${i}]}" "${LAYER_SECS[${i}]}"
    if [[ -n ${LAYER_NOTES[${i}]} ]]; then printf '  (%s)' "${LAYER_NOTES[${i}]}"; fi
    printf '\n'
    if [[ ${LAYER_STATUS[${i}]} == FAIL ]]; then
      failed=$(( failed + 1 ))
      if [[ -n ${failed_names} ]]; then failed_names="${failed_names}, "; fi
      failed_names="${failed_names}${LAYER_NAMES[${i}]}"
    fi
  done

  if [[ ${failed} -eq 0 ]]; then
    printf 'RESULT: PASS\n'
    return 0
  fi
  printf 'RESULT: FAIL (%s/%s layers failed: %s)\n' "${failed}" "${total}" "${failed_names}"
  return 1
}

# ---------------------------------------------------------------------------
# Preconditions
# ---------------------------------------------------------------------------

STATIC_CACHE="build/cmake_build/static/CMakeCache.txt"

detect_build_type() {
  [[ -f ${STATIC_CACHE} ]] || die "${STATIC_CACHE} not found — build the static flavor first: ./scripts/build.sh -tgj release"
  # Original case is what the artifact paths use (build/<BUILD_TYPE>/<flavor>/...,
  # matching build.sh's GUI_TEST_BIN); the lowercase form exists only to be
  # passed back to build.sh as its positional argument. Kept as two variables on
  # purpose so neither use silently borrows the other's casing.
  BUILD_TYPE=$(sed -n 's/^CMAKE_BUILD_TYPE:[^=]*=//p' "${STATIC_CACHE}" | head -1)
  [[ -n ${BUILD_TYPE} ]] || die "CMAKE_BUILD_TYPE missing from ${STATIC_CACHE} — reconfigure with ./scripts/build.sh -tgj release"
  BUILD_TYPE_LOWER=$(printf '%s' "${BUILD_TYPE}" | tr '[:upper:]' '[:lower:]')
}

# `quick`/`full` lean on bare `pytest` already meaning "fast subset". That is a
# property of pyproject.toml's addopts, so this reads that setting directly
# rather than inferring it from whether some particular slow test shows up in a
# collect — a sentinel test can be renamed or deleted for unrelated reasons, and
# the check would then pass while verifying nothing.
check_pytest_addopts() {
  local block
  block=$(sed -n '/^[[:space:]]*addopts[[:space:]]*=/,/\]/p' pyproject.toml)
  [[ -n ${block} ]] || die "pyproject.toml has no addopts setting — bare pytest would run the FULL suite, not the fast subset"
  printf '%s\n' "${block}" | grep -q -- '-m' \
    || die "pyproject.toml addopts no longer passes -m — bare pytest would run the FULL suite: ${block}"
  printf '%s\n' "${block}" | grep -q 'not slow' \
    || die "pyproject.toml addopts no longer pins 'not slow' — bare pytest would run the FULL suite: ${block}"
}

# ---------------------------------------------------------------------------
# gui_test pools
# ---------------------------------------------------------------------------

# The two filter expressions are read out of build.sh rather than copied here: a
# copy is a second source of truth that goes stale silently, and a stale filter
# turns into either a skipped test or a real-timing test starved by --fixed-dt.
# Anchored on `GUI_TEST_BIN"` — the same anchor check_policies.py's
# gui-test-suite-args-sync rule uses, and one that cannot match the prose in
# build.sh's comments. The two invocations are structurally exclusive (only the
# correctness one carries --fixed-dt), so neither pattern depends on line order.
extract_gui_filter() { # fixed|real
  local pattern lines value count
  case "$1" in
    fixed) pattern='GUI_TEST_BIN"[[:space:]]+--fixed-dt[[:space:]]+--filter[[:space:]]+"' ;;
    real)  pattern='GUI_TEST_BIN"[[:space:]]+--filter[[:space:]]+"' ;;
    *)     die "extract_gui_filter: unknown pool '$1'" ;;
  esac
  lines=$(grep -E "${pattern}" scripts/build.sh)
  count=$(printf '%s\n' "${lines}" | grep -c . )
  [[ ${count} -eq 1 ]] || die "gui filter extraction ($1): expected exactly 1 match in scripts/build.sh, got ${count}:
${lines}"
  value=$(printf '%s\n' "${lines}" | sed -n 's/.*--filter[[:space:]]*"\([^"]*\)".*/\1/p')
  [[ -n ${value} ]] || die "gui filter extraction ($1): matched a line but read an empty filter: ${lines}"
  printf '%s' "${value}"
}

GUI_TEST_BIN=""
GUI_FILTER_FIXED=""
GUI_FILTER_REAL=""

# Both filters are extracted before any layer runs, not lazily inside the gui
# layer: extraction failure is fatal, and a fatal error raised in the third layer
# throws away the ctest and e2e minutes already spent and prints no summary —
# which is the "run it again to find out what happened" behaviour this script
# exists to remove. Cheap enough to do unconditionally-when-needed: two greps.
preflight_gui_filters() {
  GUI_FILTER_FIXED=$(extract_gui_filter fixed) || exit $?
  GUI_FILTER_REAL=$(extract_gui_filter real) || exit $?
  # The two pools partition the suite; identical expressions would mean one pool
  # is running under the other's timing mode, which no amount of green output
  # would reveal.
  [[ ${GUI_FILTER_FIXED} != "${GUI_FILTER_REAL}" ]] \
    || die "gui filter extraction: both pools in scripts/build.sh read the same filter (${GUI_FILTER_FIXED})"
}

gui_skip_reason() {
  # build.sh documents both knobs ("Set CI=1 or LUMICE_SKIP_GUI_TESTS=1 to skip
  # gui_test"); honouring only one of them would make the same environment mean
  # two different things depending on which script you ran.
  if [[ -n ${LUMICE_SKIP_GUI_TESTS:-} ]]; then
    echo "LUMICE_SKIP_GUI_TESTS set"
  elif [[ -n ${CI:-} ]]; then
    echo "CI environment detected"
  fi
}

run_gui_layer() { # name fixed|real
  local name=$1 pool=$2 reason
  reason=$(gui_skip_reason)
  if [[ -n ${reason} ]]; then
    skip_layer "${name}" "${reason}"
    return 0
  fi
  GUI_TEST_BIN="build/${BUILD_TYPE}/static/bin/gui_test"
  if [[ ! -x ${GUI_TEST_BIN} ]]; then
    record_layer "${name}" FAIL 0 "${GUI_TEST_BIN} not found — build it with ./scripts/build.sh -tgj ${BUILD_TYPE_LOWER}"
    printf '\n========== [FAIL] %s: %s not found ==========\n' "${name}" "${GUI_TEST_BIN}"
    return 1
  fi
  if [[ ${pool} == fixed ]]; then
    run_layer "${name}" "${GUI_TEST_BIN}" --fixed-dt --filter "${GUI_FILTER_FIXED}" --no-user-config
  else
    run_layer "${name}" "${GUI_TEST_BIN}" --filter "${GUI_FILTER_REAL}" --no-user-config
  fi
}

# ---------------------------------------------------------------------------
# Layers
# ---------------------------------------------------------------------------

layer_ctest() {
  # --test-dir instead of cd/pushd: the layer must not leave test.sh in a
  # different working directory than the one the next layer's paths assume.
  run_layer ctest ctest --test-dir build/cmake_build/static \
    -L "unit-correctness|parity|golden-analytic" --output-on-failure
  local rc=$?
  if [[ ${rc} -ne 0 ]] && ! grep -q '^BUILD_TEST:[^=]*=ON' "${STATIC_CACHE}"; then
    annotate_last_layer "static tree has BUILD_TEST=OFF — rebuild with ./scripts/build.sh -tgj ${BUILD_TYPE_LOWER}"
  fi
  return ${rc}
}

layer_fast_e2e() {
  # Bare pytest is already the fast subset (pyproject addopts, verified above).
  # -n auto rather than a fixed worker count: the measured xdist ceiling for this
  # suite is ~1.75x and is reached well below full core count, so pinning a
  # number would only hard-code one machine's core count for no gain.
  run_layer fast-e2e pytest -n auto
}

# Two phases, mirroring CI's e2e-slow job: the throughput gates must not measure
# under the concurrent load of the parallel phase, so they run alone afterwards.
# Phase 2 runs even if phase 1 failed (same no-fail-fast reasoning as the layers).
layer_slow_e2e_cmd() {
  local rc1=0 rc2=0
  pytest --ignore=test/performance -n 3 -m "slow and not heavy" || rc1=$?
  pytest test/performance -m "slow and not heavy" || rc2=$?
  [[ ${rc1} -eq 0 ]] || return ${rc1}
  return ${rc2}
}

# ---------------------------------------------------------------------------
# Shared library (pr scope only)
# ---------------------------------------------------------------------------

# Mirrors test/e2e/capi_runner.py::_find_lib()'s candidate list and its order,
# because that is the code which actually loads the library. The build/<type>
# pair is parameterised on the detected BUILD_TYPE while _find_lib() hard-codes
# Release there; see the non-Release warning in ensure_shared_lib.
shared_lib_candidates() {
  printf '%s\n' \
    "build/${BUILD_TYPE}/shared/lib/liblumice.dylib" \
    "build/${BUILD_TYPE}/shared/lib/liblumice.so" \
    "build/cmake_install/shared/liblumice.dylib" \
    "build/cmake_install/shared/liblumice.so" \
    "build/cmake_install/shared/lib/liblumice.dylib" \
    "build/cmake_install/shared/lib/liblumice.so" \
    "build/cmake_build/shared/liblumice.dylib" \
    "build/cmake_build/shared/liblumice.so"
}

find_shared_lib() {
  local c
  while IFS= read -r c; do
    if [[ -f ${c} ]]; then
      printf '%s' "${c}"
      return 0
    fi
  done <<EOF
$(shared_lib_candidates)
EOF
  return 1
}

# Written after a successful shared build. The library's own mtime is not a
# sufficient reference point: a source change that the shared flavor does not
# compile (src/gui/, which this flavor does not build) leaves ninja with nothing
# to relink, so the .dylib stays older than that source forever and every
# subsequent run would re-report "stale" and rebuild again. The question the
# check actually wants to ask is "have sources changed since a build last
# succeeded", which is what the stamp records. Living under the shared build
# tree, it is removed by `build.sh -ks` along with everything else it vouches for.
SHARED_STAMP="build/cmake_build/shared/.lumice_test_sh_build_stamp"

# mtime, not git state: a locally modified but uncommitted source file is exactly
# the case where a stale library produces failures that look like real
# regressions. Consequence worth knowing: right after a fresh clone or a branch
# switch, every source file is newer than any pre-existing library, so this
# reports stale and rebuilds. That is the safe direction, not a defect.
shared_lib_stale_reason() { # lib
  local ref=$1 newer
  # The stamp only ever moves the reference point forward, never back.
  if [[ -f ${SHARED_STAMP} && ${SHARED_STAMP} -nt ${ref} ]]; then ref=${SHARED_STAMP}; fi
  newer=$(find src CMakeLists.txt cmake/CPM.cmake -newer "${ref}" -print 2>/dev/null | head -1)
  [[ -n ${newer} ]] && printf 'stale (%s is newer)' "${newer}"
}

ensure_shared_lib() {
  local lib reason
  if [[ -n ${LUMICE_LIB:-} ]]; then
    if [[ -f ${LUMICE_LIB} ]]; then
      skip_layer shared-lib "LUMICE_LIB=${LUMICE_LIB} — caller-supplied, not rebuilt"
      return 0
    fi
    record_layer shared-lib FAIL 0 "LUMICE_LIB=${LUMICE_LIB} does not exist"
    return 1
  fi

  if [[ ${BUILD_TYPE} != Release ]]; then
    printf 'test.sh: warning: BUILD_TYPE is %s, but capi_runner.py::_find_lib() anchors its\n' "${BUILD_TYPE}" >&2
    printf '         first candidate on build/Release/shared/lib — a non-Release shared build\n' >&2
    printf '         may not be the library the slow e2e layer actually loads.\n' >&2
  fi

  lib=$(find_shared_lib) || lib=""
  if [[ -z ${lib} ]]; then
    reason="missing"
  else
    reason=$(shared_lib_stale_reason "${lib}")
    if [[ -z ${reason} ]]; then
      skip_layer shared-lib "${lib} is present and fresh"
      return 0
    fi
  fi

  printf '\ntest.sh: shared library %s — rebuilding with ./scripts/build.sh -sj %s\n' \
    "${reason}" "${BUILD_TYPE_LOWER}"
  run_layer shared-lib ./scripts/build.sh -sj "${BUILD_TYPE_LOWER}"
  local rc=$?
  annotate_last_layer "${reason}"
  if [[ ${rc} -eq 0 ]]; then
    # Only after the build succeeded, and only if it produced something to vouch
    # for: a build that leaves no library behind must not be recorded as fresh.
    if find_shared_lib > /dev/null; then
      : > "${SHARED_STAMP}" || printf 'test.sh: warning: could not write %s\n' "${SHARED_STAMP}" >&2
    else
      annotate_last_layer "build succeeded but no liblumice found in any candidate path"
      rc=1
      LAYER_STATUS[$(( ${#LAYER_STATUS[@]} - 1 ))]=FAIL
    fi
  fi
  return ${rc}
}

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if [[ $# -eq 0 ]]; then
  usage >&2
  exit "${EXIT_USAGE}"
fi

SCOPE=$1
case "${SCOPE}" in
  quick|full|pr) ;;
  -h|--help|help)
    usage
    exit 0
    ;;
  *)
    printf 'test.sh: unknown scope %s\n\n' "${SCOPE}" >&2
    usage >&2
    exit "${EXIT_USAGE}"
    ;;
esac

if [[ $# -gt 1 ]]; then
  shift
  printf 'test.sh: unexpected extra arguments: %s\n\n' "$*" >&2
  usage >&2
  exit "${EXIT_USAGE}"
fi

detect_build_type
check_pytest_addopts
if [[ -z $(gui_skip_reason) ]]; then
  preflight_gui_filters
fi

printf 'test.sh: scope=%s  build_type=%s  flavor=static\n' "${SCOPE}" "${BUILD_TYPE}"

layer_ctest
layer_fast_e2e
run_gui_layer gui-correctness fixed

if [[ ${SCOPE} == full || ${SCOPE} == pr ]]; then
  run_gui_layer gui-real-timing real
fi

if [[ ${SCOPE} == pr ]]; then
  if ensure_shared_lib; then
    run_layer slow-e2e layer_slow_e2e_cmd
  else
    skip_layer slow-e2e "skipped because the shared library could not be provided"
  fi
fi

print_summary "${SCOPE}"
exit $?
