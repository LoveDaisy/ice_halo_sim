ROOT_DIR=$(cd "$(dirname $0)"; pwd)
BUILD_DIR="${ROOT_DIR}/build/cmake_build"
INSTALL_DIR="${ROOT_DIR}/build/cmake_install"
PROJ_DIR=${ROOT_DIR}

build() {
  mkdir -p "${BUILD_DIR}"
  pushd "${BUILD_DIR}"
  cmake -S "${PROJ_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=$BUILD_TYPE -DBUILD_TEST=$BUILD_TEST \
        -DMULTI_THREAD=$MULTI_THREAD -DRANDOM_SEED=$RANDOM_SEED -DVERBOSE_LOG=$VERBOSE_LOG \
        -DCMAKE_INSTALL_PREFIX="${INSTALL_DIR}" -DGTEST_DIR="${GTEST_DIR}"
  cmake --build "${BUILD_DIR}" -j $MAKE_J_N
  ret=$?
  if [[ $ret == 0 && $BUILD_TEST == ON ]]; then
    echo "Testing..."
    ctest
    ret=$?
  fi
  if [[ $ret == 0 && $INSTALL_FLAG == ON ]]; then
    echo "Installing..."
    cmake --build "${BUILD_DIR}" --target install
    ret=$?
  fi
}


benchmarking() {
  python3 "${PROJ_DIR}/test/run_bench.py" --exe "${INSTALL_DIR}/IceHaloEndless" \
          --config "${PROJ_DIR}/test/benchmark_config_template.json" \
          --tmp_dir "${BUILD_DIR}"
}


help() {
  echo "Usage:"
  echo "  ./build.sh [-tjkrh1] <debug|release|minsizerel>"
  echo "    Executables will be installed at build/cmake_install"
  echo "OPTIONS:"
  echo "  -t:          Build test cases and run test on them."
  echo "  -j:          Build in parallel, i.e. use make -j"
  echo "  -k:          Clean temporary building files."
  echo "  -b:          Run a benchmarking. It tells how fast the program runs on your computer."
  echo "  -v:          Enable verbose log."
  echo "  -r:          Use random seed for random number generator. Without this option,"
  echo "               the program will use a constant value. Thus generate a repeatable result"
  echo "               (usually together with -1)."
  echo "  -1:          Use single thread."
  echo "  -h:          Show this message."
}


clean_all() {
  echo "Cleaning temporary building files..."
  rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
}


BUILD_TYPE=Debug
BUILD_TEST=OFF
INSTALL_FLAG=OFF
MAKE_J_N=1
MULTI_THREAD=ON
RANDOM_SEED=OFF
VERBOSE_LOG=OFF

if [ $# -eq 0 ]; then
  help
  exit 0
fi

# Use getopts to parse arguments
# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

while getopts "htrjkb1" opt; do
  case "$opt" in
  h)
    help
    exit 0
    ;;
  t)
    BUILD_TEST=ON
    ;;
  j)
    MAKE_J_N=$(sysctl -n hw.ncpu) || MAKE_J_N=$(cat /proc/cpuinfo | grep processor | wc -l) || MAKE_J_N=8
    ;;
  r)
    RANDOM_SEED=ON
    ;;
  v)
    VERBOSE_LOG=ON
    ;;
  k)
    clean_all
    ;;
  b)
    benchmarking
    ;;
  1)
    MULTI_THREAD=OFF
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
