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
    ctest -L unit --output-on-failure
    ret=$?
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
  echo "  -t:          Build test cases and run test on them."
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
