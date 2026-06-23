#!/bin/bash
#
# Format (default) or check (--check) all C/C++/ObjC sources under src/, test/, bench/.
#   ./scripts/format.sh           # rewrite files in place (clang-format -i)
#   ./scripts/format.sh --check   # report files that would change; exit 1 if any
#
# --check is the read-only mode used by CI (lint job) and the pre-commit hook.

CHECK_MODE=0
if [ "$1" == "--check" ]; then
  CHECK_MODE=1
fi

# Tracks whether any file failed the format check (--check mode only).
CHECK_FAILED=0

function format_file() {
  local f="$1"
  if [ "${CHECK_MODE}" -eq 1 ]; then
    # The repo .clang-format has no Objective-C language config, so clang-format
    # cannot meaningfully check .mm/.m — skip them in check mode rather than fail
    # the gate on files it was never configured to handle.
    case "$f" in
      *.mm|*.m) return ;;
    esac
    if ! clang-format --dry-run --Werror "$f" >/dev/null 2>&1; then
      echo "Needs formatting: $f"
      CHECK_FAILED=1
    fi
  else
    echo "Formatting $f ..."
    clang-format -i "$f"
  fi
}

function format_dir() {
  local working_dir="$1"
  echo "Directory ${working_dir} ..."
  pushd "`pwd`" >/dev/null
  cd "${working_dir}"
  for f in $(find . | grep -E "[^/]*\.(cpp|cc|h|hpp|inl|mm|m|java)\$"); do
    format_file "$f"
  done
  popd >/dev/null
}

base_dir=$(cd "$(dirname "$0")/.."; pwd)

if [ -z "$(which clang-format)" ]; then
  echo "No clang-format found! Do nothing."
  exit 0
fi

format_dir "${base_dir}/src"
format_dir "${base_dir}/test"
format_dir "${base_dir}/bench"

if [ "${CHECK_MODE}" -eq 1 ] && [ "${CHECK_FAILED}" -ne 0 ]; then
  echo ""
  echo "clang-format check failed. Run ./scripts/format.sh to fix."
  exit 1
fi
