// Reference oracle vertex counts, generated once from the __int128 predecessor
// of test/support/exact_pyramid_oracle.hpp.
//
// Contract: index-for-index parallel to the sample-pool arrays in
// closed_form_samples_generated.hpp (kPyramidWellConditionedSamples,
// kPyramidMillerSamples, six kPyramidFlatTailAlpha* pools, two
// kPyramidDegenerateSigma* pools). Each entry is either the __int128 oracle's
// vertex_count for that sample, or -1 when the __int128 oracle refused
// (arithmetic budget exhausted). refused=1 samples in the reference file are
// encoded as -1 here so a downstream test can distinguish "known good count"
// from "predecessor could not answer".
//
// Generation mechanism: swap test/support/exact_pyramid_oracle.hpp to the
// __int128 revision (parent of commit 40e3f2ae — the last revision using
// __int128 numeric arithmetic instead of the current symbolic-alpha
// PolyQS3 engine), build the golden-analytic test with the same fixed
// sample pools, dump `oracle.vertex_count` and `oracle.refused` per
// sample, and feed the dump into a one-shot script that emits this file.
// The generator is kept out of the tree — same convention as
// closed_form_samples_generated.hpp ("throwaway generator, replace the
// arrays verbatim on regeneration, never edit in place").
//
// Regeneration checklist (trigger: any sample-pool array in
// closed_form_samples_generated.hpp is added/removed/reordered):
//   1. Check out the __int128 revision of exact_pyramid_oracle.hpp.
//   2. Rebuild the golden-analytic test with the CURRENT sample pools.
//   3. Re-dump `oracle.vertex_count`/`oracle.refused` per (pool, idx).
//   4. Regenerate this file wholesale from the new dump. Do NOT patch
//      individual entries — the whole file is replaced or nothing.

#ifndef LUMICE_TEST_GOLDEN_ANALYTIC_PYRAMID_ORACLE_INT128_REFERENCE_GENERATED_HPP_
#define LUMICE_TEST_GOLDEN_ANALYTIC_PYRAMID_ORACLE_INT128_REFERENCE_GENERATED_HPP_

namespace lumice {
namespace test_support {

inline constexpr int kPyramidWellConditionedOracleRefVtx[200] = {
  20, 20, 20, 20, 22, 20, 22, 22, 20, 20, 22, 20, 20, 20, 20, 20, 22, 24, 20, 20, 20, 20, 20, 20, 22, 20, 20, 20, 20,
  20, 20, 20, 20, 24, 20, 20, 20, 20, 24, 20, 20, 20, 20, 20, 20, 20, 22, 20, 20, 20, 22, 20, 20, 20, 20, 22, 22, 20,
  20, 24, 20, 20, 22, 22, 20, 20, 20, 20, 20, 22, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 22,
  20, 20, 22, 20, 22, 20, 20, 22, 20, 22, 20, 20, 20, 22, 22, 20, 20, 20, 20, 20, 20, 22, 20, 20, 20, 20, 20, 22, 20,
  20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 22, 20, 20, 20, 20, 20, 20, 22, 22, 22, 20, 20, 20, 20, 24,
  20, 20, 22, 20, 20, 20, 20, 20, 20, 20, 20, 20, 22, 20, 20, 20, 20, 24, 22, 20, 20, 20, 20, 20, 22, 22, 22, 22, 22,
  20, 20, 20, 20, 20, 20, 22, 24, 22, 20, 24, 20, 20, 20, 22, 20, 20, 22, 22, 22, 22, 20, 22, 22, 22, 22,
};

inline constexpr int kPyramidMillerOracleRefVtx[288] = {
  20, 22, 20, 20, 20, 20, 22, 22, 20, 22, 20, 20, 20, 22, 20, 20, 22, 20, 20, 22, 20, 22, 20, 22, 20, 20, 20, 20, 20,
  20, 22, 20, 20, 20, 20, 22, 22, 22, 22, 20, 20, 20, 20, 20, 20, 20, 20, 22, 20, 20, 24, 22, 24, 20, 22, 20, 22, 22,
  20, 22, 20, 20, 20, 24, 24, 22, 20, 22, 22, 20, 22, 22, 20, 20, 20, 20, 20, 20, 24, 22, 20, 20, 22, 24, 24, 24, 20,
  20, 22, 22, 22, 22, 24, 22, 20, 20, 22, 22, 24, 20, 20, 22, 20, 20, 22, 20, 22, 20, 22, 22, 20, 24, 22, 20, 22, 24,
  24, 24, 22, 22, 24, 20, 22, 22, 20, 20, 20, 20, 20, 22, 22, 20, 20, 20, 24, 22, 20, 20, 22, 20, 24, 22, 20, 20, 20,
  20, 22, 20, 24, 20, 20, 22, 20, 20, 20, 20, 20, 20, 20, 22, 20, 22, 20, 20, 22, 22, 22, 20, 20, 22, 20, 22, 20, 20,
  20, 20, 22, 20, 20, 20, 20, 20, 20, 20, 20, 20, 24, 20, 20, 22, 20, 20, 20, 20, 24, 22, 24, 20, 22, 20, 20, 22, 20,
  22, 22, 20, 22, 22, 22, 20, 20, 20, 20, 20, 22, 24, 20, 22, 22, 22, 20, 20, 20, 22, 24, 22, 20, 24, 24, 22, 22, 20,
  22, 22, 24, 22, 20, 22, 22, 22, 24, 20, 22, 22, 20, 20, 20, 22, 24, 24, 22, 22, 22, 22, 24, 22, 20, 24, 20, 22, 22,
  22, 22, 22, 20, 20, 22, 22, 22, 20, 22, 20, 22, 24, 22, 24, 22, 22, 22, 22, 20, 20, 24, 24, 24, 22, 20, 22,
};

inline constexpr int kPyramidFlatTailAlpha85OracleRefVtx[40] = {
  -1, -1, -1, -1, 20, -1, -1, -1, -1, -1, -1, -1, -1, 20, -1, -1, -1, -1, -1, 20,
  -1, -1, 20, -1, -1, 20, -1, -1, -1, -1, -1, 20, -1, -1, -1, -1, -1, -1, -1, -1,
};

inline constexpr int kPyramidFlatTailAlpha87OracleRefVtx[40] = {
  20, -1, 20, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 20, -1, -1, -1, -1,
  -1, -1, -1, -1, 20, 20, -1, -1, -1, -1, -1, 20, -1, -1, -1, 20, -1, -1, 20, -1,
};

inline constexpr int kPyramidFlatTailAlpha875OracleRefVtx[40] = {
  20, -1, -1, 20, -1, -1, -1, -1, -1, 20, -1, -1, 20, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, 20, -1, 20, 20, -1, -1, -1, -1, 20, -1, -1, -1, -1,
};

inline constexpr int kPyramidFlatTailAlpha88OracleRefVtx[40] = {
  -1, -1, 20, 20, -1, -1, -1, -1, -1, 20, 20, -1, 20, -1, -1, -1, 20, -1, 20, -1,
  -1, -1, -1, -1, 20, 20, 20, -1, -1, -1, -1, 20, -1, -1, 20, 20, -1, -1, 20, -1,
};

inline constexpr int kPyramidFlatTailAlpha89OracleRefVtx[40] = {
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

inline constexpr int kPyramidFlatTailAlpha895OracleRefVtx[40] = {
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};

inline constexpr int kPyramidDegenerateSigma030OracleRefVtx[39] = {
  -1, 20, -1, -1, 16, -1, -1, -1, 20, 20, -1, 16, 20, -1, -1, 20, -1, 20, -1, -1,
  20, -1, -1, -1, -1, -1, -1, -1, -1, -1, 20, 20, -1, -1, -1, -1, 20, -1, 20,
};

inline constexpr int kPyramidDegenerateSigma050OracleRefVtx[40] = {
  -1, 20, 12, 20, -1, -1, -1, 20, -1, -1, -1, -1, 12, 16, -1, 20, -1, -1, 16, -1,
  -1, -1, 16, -1, -1, -1, 16, -1, 18, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 16,
};

}  // namespace test_support
}  // namespace lumice

#endif  // LUMICE_TEST_GOLDEN_ANALYTIC_PYRAMID_ORACLE_INT128_REFERENCE_GENERATED_HPP_
