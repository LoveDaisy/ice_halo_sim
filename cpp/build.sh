ROOT_DIR=$(cd "$(dirname $0)"; pwd)
BUILD_DIR="${ROOT_DIR}/build/cmake_build"
INSTALL_DIR="${ROOT_DIR}/build/cmake_install"
PROJ_DIR=${ROOT_DIR}

build() {
  mkdir -p "${BUILD_DIR}"
  cd "${BUILD_DIR}"
  rm -rf CMakeFiles
  rm -rf CMakeCache.txt
  rm -rf Makefile cmake_install.cmake
  cmake "${PROJ_DIR}" \
        -DDEBUG=$DEBUG_FLAG -DBUILD_TEST=$BUILD_TEST \
        -DMULTI_THREAD=$MULTI_THREAD \
        -DRANDOM_SEED=$RANDOM_SEED \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR"
  make -j$MAKE_J_N
  ret=$?
  if [[ $ret == 0 && $BUILD_TEST == ON ]]; then
    echo "Testing..."
    make test
    ret=$?
  fi
  if [[ $ret == 0 && $INSTALL_FLAG == ON ]]; then
    echo "Installing..."
    make install
  fi
}


help() {
  echo "Usage:"
  echo "  ./build.sh [-tjkrh1] <debug|release>"
  echo "    Build executables for debug | release"
  echo "    Executables will be installed at build/cmake_install"
  echo "OPTIONS:"
  echo "  -t:          Build test cases and run test on them."
  echo "  -j:          Make in parallel, i.e. use make -j"
  echo "  -k:          Clean temporary building files."
  echo "  -r:          Use system time as seed for random number generator. Without this option,"
  echo "               the program will use default value. Thus generate a repeatable result (together with -1)."
  echo "  -1:          Use single thread."
  echo "  -h:          Show this message."
}


clean_all() {
  echo "Cleaning temporary building files..."
  rm -rf "${BUILD_DIR}" "${INSTALL_DIR}"
}


DEBUG_FLAG=OFF
BUILD_TEST=OFF
INSTALL_FLAG=OFF
MAKE_J_N=1
MULTI_THREAD=ON
RANDOM_SEED=OFF

if [ $# -eq 0 ]; then
  help
  exit 0
fi

# Use getopts to parse arguments
# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

while getopts "htrjk1" opt; do
  case "$opt" in
  h)
    help
    exit 0
    ;;
  t)
    BUILD_TEST=ON
    ;;
  j)
    MAKE_J_N=""
    ;;
  r)
    RANDOM_SEED=ON
    ;;
  k)
    clean_all
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
      DEBUG_FLAG=ON
      INSTALL_FLAG=OFF
      build
      exit 0
    ;;
    release)
      DEBUG_FLAG=OFF
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
