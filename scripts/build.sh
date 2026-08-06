#!/usr/bin/env bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.."; pwd)
PROJ_DIR=${ROOT_DIR}

# BUILD_DIR / INSTALL_DIR / GUI_TEST_BIN / FLAVOR are all derived from -s and are
# therefore assigned *after* getopts (see the resolve_flavor_paths call below),
# not here — assigning them at the top would freeze them at the static values
# before -s has been seen.

build() {
  mkdir -p "${BUILD_DIR}"
  pushd "${BUILD_DIR}" > /dev/null
  cmake -S "${PROJ_DIR}" -B "${BUILD_DIR}" -G Ninja \
        -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_TEST=$BUILD_TEST \
        -DBUILD_BENCH=$BUILD_BENCH -DBUILD_GUI=$BUILD_GUI \
        -DBUILD_SHARED_LIBS=$BUILD_SHARED \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}"
  cmake --build "${BUILD_DIR}" -j $MAKE_J_N
  ret=$?
  if [[ $ret == 0 && $BUILD_TEST == ON ]]; then
    echo "Testing..."
    ctest -L "unit-correctness|parity|golden-analytic" --output-on-failure
    ret=$?
  fi
  if [[ $ret == 0 && $BUILD_TEST == ON && $BUILD_GUI == ON ]]; then
    if [[ -n "${LUMICE_SKIP_GUI_TESTS:-}" || -n "${CI:-}" ]]; then
      skip_reason="LUMICE_SKIP_GUI_TESTS set"
      if [[ -n "${CI:-}" && -z "${LUMICE_SKIP_GUI_TESTS:-}" ]]; then
        skip_reason="CI environment detected"
      fi
      echo "Skipping GUI tests ($skip_reason)"
    else
      GUI_TEST_BIN="${ROOT_DIR}/build/${BUILD_TYPE}/${FLAVOR}/bin/gui_test"
      if [[ -x "$GUI_TEST_BIN" ]]; then
        # Two pools (see scratchpad/task-gui-test-fixed-dt):
        #  1. Correctness pool: --fixed-dt injects a deterministic 1/60s frame dt
        #     and skips the frame-limit sleep, so functional/visual tests run at
        #     full wall-clock speed.
        #  2. Real-timing pool: real frame timing + real dt, run in isolation.
        #     Holds tests whose meaning depends on real wall-clock:
        #       - perf_test: measures main-loop FPS / rays-per-sec.
        #       - save_open_visual_consistency: compares the live poller preview
        #         against the saved snapshot. It now waits on accumulated rays
        #         rather than on a frame count, so fixed-dt no longer starves it
        #         outright; it stays here because that wait is bounded by a real
        #         wall-clock deadline, and fixed-dt decouples the test engine's
        #         watchdog (which counts simulated frame time) from that deadline.
        #       - revert_repushes_server_display_state, zorder_priority_persists_across_rerun
        #         (task-color-migration code-review round-1 revision): both assert on
        #         LUMICE_FrameGetComposite() right after a display-time PushDisplayState()
        #         edit (color edit / z_order swap); the edit only materializes in the
        #         composite once the background ServerPoller's WakeForRefresh-triggered
        #         PollOnce() actually runs, which needs real wall-clock time between
        #         ctx->Yield() calls — fixed-dt (and --no-frame-limit) starve that thread
        #         the same way they starve save_open_visual_consistency's accumulation.
        echo "Running GUI correctness tests (fixed-dt, fast)..."
        # --no-user-config is redundant with gui_test's own default and stated anyway: it puts the
        # contract in the diff where a reviewer sees it, and it survives someone changing that
        # default. Keep it AFTER --filter — check_policies.py's gui-test-suite-args-sync reads the
        # filter value with a regex anchored on `--fixed-dt --filter "..."`.
        "$GUI_TEST_BIN" --fixed-dt --filter "-perf_test,-save_open_visual_consistency,-revert_repushes_server_display_state,-zorder_priority_persists_across_rerun,-p2_gpu_color_degrade" --no-user-config
        ret=$?
        if [[ $ret == 0 ]]; then
          echo "Running GUI real-timing tests (perf + wall-clock-dependent, isolated)..."
          "$GUI_TEST_BIN" --filter "perf_test,save_open_visual_consistency,revert_repushes_server_display_state,zorder_priority_persists_across_rerun,p2_gpu_color_degrade" --no-user-config
          ret=$?
        fi
      else
        echo "Warning: $GUI_TEST_BIN not found, skipping GUI tests"
      fi
    fi
  fi
  if [[ $ret == 0 && $INSTALL_FLAG == ON ]]; then
    echo "Installing..."
    cmake --build "${BUILD_DIR}" --target install
    ret=$?
  fi
  popd > /dev/null
}


help() {
  echo "Usage:"
  echo "  ./build.sh [-tgbjksxh] <debug|release|minsizerel>"
  echo "    Executables will be installed at build/cmake_install/<flavor>,"
  echo "    where <flavor> is shared with -s and static without it. The two"
  echo "    flavors have separate build and install trees, so they coexist and"
  echo "    switching between them does not force a rebuild."
  echo "OPTIONS:"
  echo "  -t:          Build test cases and run unit/parity/golden tests"
  echo "               (CTest -L \"unit-correctness|parity|golden-analytic\")."
  echo "               Combined with -g, also runs gui_test (requires a display)."
  echo "               Set CI=1 or LUMICE_SKIP_GUI_TESTS=1 to skip gui_test."
  echo "  -g:          Build GUI application (Dear ImGui + GLFW + OpenGL)."
  echo "  -b:          Build benchmarks (Google Benchmark)."
  echo "  -j:          Build in parallel, i.e. use make -j"
  echo "  -k:          Clean build artifacts of the selected flavor only"
  echo "               (keep dependency cache). -k cleans static, -ks cleans shared."
  echo "               NOTE: this removes build/cmake_build/<flavor> and"
  echo "               build/cmake_install/<flavor>, but NOT the compiler output tree"
  echo "               build/<BUILD_TYPE>/<flavor>/. A liblumice there survives -k, and"
  echo "               it is the first thing the e2e C API loader looks for — so after"
  echo "               -k it can still load a pre-clean library. Use -x to wipe build/."
  echo "  -x:          Clean everything including dependency cache (both flavors)."
  echo "  -s:          Build shared library (default: static)."
  echo "  -h:          Show this message."
}


# Derive every flavor-dependent path from BUILD_SHARED. Called once, after
# getopts, so -s has already been seen no matter where it appeared in the flag
# string (the reason clean_all is deferred too — see DO_CLEAN_ALL below).
resolve_flavor_paths() {
  if [[ $BUILD_SHARED == ON ]]; then
    FLAVOR=shared
  else
    FLAVOR=static
  fi
  BUILD_DIR="${ROOT_DIR}/build/cmake_build/${FLAVOR}"
  INSTALL_DIR="${ROOT_DIR}/build/cmake_install/${FLAVOR}"
}

# Cleans the selected flavor only. That narrowing is not a new feature: BUILD_DIR
# and INSTALL_DIR simply are per-flavor paths now, and -k cleans what they name.
# The other flavor's tree is left warm on purpose.
clean_all() {
  echo "Cleaning build artifacts (${FLAVOR})..."
  rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
}

clean_everything() {
  echo "Cleaning all build files and dependency cache..."
  rm -rf "${ROOT_DIR}/build"
}


BUILD_TYPE=Debug
BUILD_TEST=OFF
BUILD_BENCH=OFF
BUILD_GUI=OFF
BUILD_SHARED=OFF
INSTALL_FLAG=OFF
MAKE_J_N=1
DO_CLEAN_ALL=OFF
DO_CLEAN_EVERYTHING=OFF

if [ $# -eq 0 ]; then
  help
  exit 0
fi

# Use getopts to parse arguments
# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

while getopts "htgbjksx" opt; do
  case "$opt" in
  h)
    help
    exit 0
    ;;
  t)
    BUILD_TEST=ON
    ;;
  g)
    BUILD_GUI=ON
    ;;
  b)
    BUILD_BENCH=ON
    ;;
  j)
    MAKE_J_N=$(nproc 2>/dev/null) || MAKE_J_N=$(sysctl -n hw.ncpu 2>/dev/null) || MAKE_J_N=8
    ;;
  k)
    DO_CLEAN_ALL=ON
    ;;
  x)
    DO_CLEAN_EVERYTHING=ON
    ;;
  s)
    BUILD_SHARED=ON
    ;;
  *)
    help
    exit 0
    ;;
  esac
done

shift $((OPTIND-1))

# Flavor is known only now: -s may appear anywhere in the flag string (-ks vs
# -sk). Resolving inside the getopts loop would make "which flavor does -k
# clean?" depend on flag order.
resolve_flavor_paths

if [[ $DO_CLEAN_EVERYTHING == ON ]]; then
  clean_everything
fi
if [[ $DO_CLEAN_ALL == ON ]]; then
  clean_all
fi

[ "${1:-}" = "--" ] && shift

while [ ! $# -eq 0 ]; do
  case $1 in
    debug)
      BUILD_TYPE=Debug
      INSTALL_FLAG=OFF
      build
      exit 0
    ;;
    release)
      BUILD_TYPE=Release
      INSTALL_FLAG=ON
      build
      exit 0
    ;;
    minsizerel)
      BUILD_TYPE=MinSizeRel
      INSTALL_FLAG=ON
      build
      exit 0
    ;;
    *)
      help
      exit 0
    ;;
  esac
done
