#!/usr/bin/env bash
set -e

ROOT_DIR=$(cd "$(dirname "$0")/.."; pwd)
BUILD_DIR="${ROOT_DIR}/build/cmake_build"
INSTALL_DIR="${ROOT_DIR}/build/cmake_install"
PROJ_DIR=${ROOT_DIR}

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
      GUI_TEST_BIN="${ROOT_DIR}/build/${BUILD_TYPE}/bin/gui_test"
      if [[ -x "$GUI_TEST_BIN" ]]; then
        # Two pools (see scratchpad/task-gui-test-fixed-dt):
        #  1. Correctness pool: --fixed-dt injects a deterministic 1/60s frame dt
        #     and skips the frame-limit sleep, so functional/visual tests run at
        #     full wall-clock speed.
        #  2. Real-timing pool: real frame timing + real dt, run in isolation.
        #     Holds tests whose meaning depends on real wall-clock:
        #       - perf_test: measures main-loop FPS / rays-per-sec.
        #       - save_open_visual_consistency: compares the live poller preview
        #         (which converges over ~30 frames of real wall-clock simulation)
        #         against the saved snapshot; fixed-dt starves that accumulation
        #         and drops its PSNR ~7 dB below threshold.
        #       - revert_repushes_server_display_state, zorder_priority_persists_across_rerun
        #         (task-color-migration code-review round-1 revision): both assert on
        #         LUMICE_GetCompositeResults() right after a display-time PushDisplayState()
        #         edit (color edit / z_order swap); the edit only materializes in the
        #         composite once the background ServerPoller's WakeForRefresh-triggered
        #         PollOnce() actually runs, which needs real wall-clock time between
        #         ctx->Yield() calls — fixed-dt (and --no-frame-limit) starve that thread
        #         the same way they starve save_open_visual_consistency's accumulation.
        echo "Running GUI correctness tests (fixed-dt, fast)..."
        "$GUI_TEST_BIN" --fixed-dt --filter "-perf_test,-save_open_visual_consistency,-revert_repushes_server_display_state,-zorder_priority_persists_across_rerun,-p2_gpu_color_degrade"
        ret=$?
        if [[ $ret == 0 ]]; then
          echo "Running GUI real-timing tests (perf + wall-clock-dependent, isolated)..."
          "$GUI_TEST_BIN" --filter "perf_test,save_open_visual_consistency,revert_repushes_server_display_state,zorder_priority_persists_across_rerun,p2_gpu_color_degrade"
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
  echo "    Executables will be installed at build/cmake_install"
  echo "OPTIONS:"
  echo "  -t:          Build test cases and run unit/parity/golden tests"
  echo "               (CTest -L \"unit-correctness|parity|golden-analytic\")."
  echo "               Combined with -g, also runs gui_test (requires a display)."
  echo "               Set CI=1 or LUMICE_SKIP_GUI_TESTS=1 to skip gui_test."
  echo "  -g:          Build GUI application (Dear ImGui + GLFW + OpenGL)."
  echo "  -b:          Build benchmarks (Google Benchmark)."
  echo "  -j:          Build in parallel, i.e. use make -j"
  echo "  -k:          Clean build artifacts (keep dependency cache)."
  echo "  -x:          Clean everything including dependency cache."
  echo "  -s:          Build shared library (default: static)."
  echo "  -h:          Show this message."
}


clean_all() {
  echo "Cleaning build artifacts..."
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
    clean_all
    ;;
  x)
    clean_everything
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
