#!/usr/bin/env bash
#
# Verify that the CUDA-only test translation units are actually compiled INTO the
# unit-correctness and parity binaries, and that their cases self-skip rather than fail on a
# runner with no CUDA device.
#
# Why this is a script and not two inline `run:` blocks: both `cuda-compile` (Linux) and
# `windows-cuda-compile` (Windows) call it, and the two must not drift apart. Both invoke it
# through `shell: bash`, which windows-2022 provides.
#
# Why it asserts a case count instead of just running ctest: the CUDA test TUs are wrapped in
# `#if defined(LUMICE_CUDA_ENABLED)`. Without that macro they compile to nothing, GTEST_FILTER
# then selects zero cases, and gtest exits 0 — so a bare ctest run is GREEN in exactly the state
# this gate exists to detect (that state was the status quo on every platform until these two
# jobs started configuring with BUILD_TEST=ON). Requiring at least one selected case per binary
# is what turns "still an empty TU" into a red.
#
# Scope: this is COMPILE coverage. Every case here is expected to report SKIPPED, because no GPU
# is attached to these runners. Runtime and parity validation still run on the CUDA reference
# machines under the manual protocol in doc/gpu-remote-cuda-build-testing.md; a green run here
# must not be read as "CUDA all green".

set -euo pipefail

build_dir="${1:?usage: $0 <build-dir> <gtest-filter>}"
gtest_filter="${2:?usage: $0 <build-dir> <gtest-filter>}"

status=0

for test_name in LumiceUnitCorrectnessTest LumiceParityTest; do
  log="ctest-${test_name}.log"
  echo "=== ${test_name} (GTEST_FILTER=${gtest_filter}) ==="

  # --no-tests=error: an empty ctest selection is a broken gate, not a pass. ctest's default
  # action for "no tests matched" is not something to leave implicit here.
  if ! GTEST_FILTER="${gtest_filter}" ctest --test-dir "${build_dir}" \
      -R "^${test_name}\$" --no-tests=error --verbose >"${log}" 2>&1; then
    cat "${log}"
    echo "ERROR: ${test_name} did not pass."
    echo "       On a GPU-less runner every selected CUDA case must SKIP. A FAILED case means the"
    echo "       ShouldSkipCudaTests()/CudaDeviceAvailable() guard itself is broken — report that"
    echo "       rather than relaxing this gate."
    status=1
    continue
  fi

  # gtest prints "[==========] Running N tests from M test suites." once per run; ctest --verbose
  # prefixes it with the test index, hence the leading `.*`.
  selected="$(sed -n 's/.*Running \([0-9][0-9]*\) test.*/\1/p' "${log}" | head -n 1)"
  if [ -z "${selected}" ] || [ "${selected}" -eq 0 ]; then
    cat "${log}"
    echo "ERROR: ${test_name} selected 0 cases under GTEST_FILTER='${gtest_filter}'."
    echo "       The CUDA test TUs compiled to nothing, which is the exact gap this job closes."
    echo "       Check that the configure step still passes -DBUILD_TEST=ON -DLUMICE_CUDA_ENABLED=ON."
    status=1
    continue
  fi

  echo "${test_name}: ${selected} CUDA case(s) selected (SKIPPED expected on a GPU-less runner)."
  grep -E '\[  SKIPPED \]|\[       OK \]|\[  FAILED  \]' "${log}" || true
done

exit "${status}"
