#!/bin/bash

function format_dir() {
  local working_dir="$1"
  echo "Directory ${working_dir} ..."
  pushd "`pwd`"
  cd "${working_dir}"
  for f in $(find . | grep -E "[^/]*\.(cpp|cc|h|hpp|inl|mm|m|java)\$"); do
    echo "Formatting $f ..."
    clang-format -i "$f"
  done
  popd
}

function format_dir_recursive() {
  local working_dir=$1
  format_dir "${working_dir}"
  for f in `ls "${working_dir}"`; do
    if [[ -d "${working_dir}/$f" ]]; then
      format_dir_recursive "${working_dir}/$f"
    fi
  done
}


base_dir=$(cd `dirname $0`; pwd)

if [ -z "$(which clang-format)" ]; then
  echo "No clang-format found! Do nothing."
  exit 0
fi

format_dir "${base_dir}/src"
format_dir "${base_dir}/test"

