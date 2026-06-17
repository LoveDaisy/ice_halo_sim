// Parity-only Metal kernel source string (test scaffold).
//
// Hosts the standalone `filter_match_test_kernel` MSL entry point used by the
// parity harness in `test_metal_filter_match_parity.mm`. This symbol used to
// live in `src/core/metal_filter_match_src.{hpp,mm}` and was therefore
// compiled into liblumice; task-270.8 boundary-hardening moved it into the
// parity layer so production binaries no longer carry test scaffolding.

#ifndef TEST_PARITY_METAL_FILTER_MATCH_TEST_SRC_H_
#define TEST_PARITY_METAL_FILTER_MATCH_TEST_SRC_H_

#if defined(__APPLE__)

namespace lumice {

// Self-contained kernel entry that exercises `DeviceFilterMatch` once per
// thread. Concatenated after `kFilterMatchHelperSrc` (see
// `core/metal_filter_match_src.hpp`) before MSL compilation.
extern const char* const kFilterMatchTestKernelSrc;

}  // namespace lumice

#endif  // defined(__APPLE__)

#endif  // TEST_PARITY_METAL_FILTER_MATCH_TEST_SRC_H_
