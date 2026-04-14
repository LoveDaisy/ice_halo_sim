#ifndef SRC_CORE_CRYSTAL_KIND_HPP_
#define SRC_CORE_CRYSTAL_KIND_HPP_

// Minimal zero-dependency header exposing a GUI-facing crystal-kind enum.
//
// This header exists to let the config layer (src/config/raypath_validation.hpp)
// and the core layer (src/core/crystal.hpp) share a tiny, dependency-free enum
// without establishing a reverse dependency (core -> config) or pulling the
// full crystal.hpp surface into the config layer.
//
// Rationale (see scratchpad/scrum-gui-refactor-debt/task-filter-face-validation/plan.md):
//   - `core/crystal.hpp` defines the richer `CrystalType` enum that includes
//     every variant the core simulator recognises (~10 values).
//   - The GUI / filter-validation path only needs to distinguish two high-level
//     kinds (prism-family vs. pyramid-family) to gate legal face numbers.
//   - Keeping this enum in its own header means both the core and config
//     translation units can share it without extra coupling.
namespace lumice {

// Coarse GUI-facing crystal classification used for raypath face-number validation.
// If additional kinds are introduced later, every switch over `CrystalKind` must
// be extended; there is deliberately no default branch in `IsLegalFace` to force
// compile-time / runtime exposure of unhandled values.
enum class CrystalKind {
  kPrism,
  kPyramid,
};

}  // namespace lumice

#endif  // SRC_CORE_CRYSTAL_KIND_HPP_
