#ifndef LUMICE_GUI_CRYSTAL_PREVIEW_HPP
#define LUMICE_GUI_CRYSTAL_PREVIEW_HPP

#include "gui/axis_presets.hpp"
#include "include/lumice.h"

namespace lumice::gui {

struct CrystalConfig;

// Crystal preview trackball state
extern float g_crystal_rotation[16];
extern float g_crystal_zoom;
extern int g_crystal_style;
extern int g_crystal_mesh_hash;

// Crystal preview helpers
int CrystalParamHash(const CrystalConfig& c);
// Legacy GUI initialization path only; always returns Rx(20°). Any new
// reset-view logic should use the two-argument overload below with preset +
// params. Used by main GUI startup and the test harness where no axis preset /
// distribution params are in scope.
void ResetCrystalView();
// Reset to the default view derived from the given axis preset and current
// distribution params (zenith / azimuth / roll layout matches g_axis_buf in
// edit_modals.cpp: params[0]=zenith, [1]=azimuth, [2]=roll). The matrix is
// produced by DefaultPreviewRotation (single source of truth shared with the
// entry-card thumbnail). Pass nullptr for params to fall back to the
// preset-typical or isometric sentinel default (only meaningful for kCustom).
void ResetCrystalView(AxisPreset preset, const AxisDist params[3]);
// Reset to the default view derived from a CrystalConfig (classifies axis
// preset internally + builds params). Used by main.cpp startup, DoNew, and
// DoOpen so the modal preview's initial view always matches what the outer
// entry-card thumbnail shows for the current entry's axis distribution.
void ResetCrystalViewToCrystal(const CrystalConfig& cr);
void ApplyTrackballRotation(float dx, float dy);

// Fixed preview sample seed. The GUI CrystalConfig shape fields are now first-class distributions,
// so a randomized shape DOES vary with the seed; this constant deliberately pins the preview to a
// single deterministic draw (seed 0) so the modal preview and thumbnails stay stable frame-to-frame
// and match the four tracked crystal reference images. It keeps the call sites (edit_modals.cpp,
// thumbnail_cache.cpp) on the SAME value rather than each inlining a literal. `inline constexpr`
// gives one definition across TUs.
// TODO(gui-preview-animation, gui-shape-randomization): replace with a real, per-frame-varying seed
// source so the preview animates while a shape distribution is active.
inline constexpr unsigned long long kPreviewFixedSampleSeed = 0;

// Build crystal mesh data from config: construct a LUMICE_CrystalParam, sample via
// LUMICE_GetCrystalMesh, then apply Y-Z swap + AABB normalization. `sample_seed` selects the
// shape draw (fixed to kPreviewFixedSampleSeed for a stable preview; a randomized shape yields a
// different but deterministic mesh per seed).
// Caller allocates LUMICE_CrystalMesh on stack (fixed-size arrays, no free needed). Returns true on success.
bool BuildCrystalMeshData(const CrystalConfig& cr, unsigned long long sample_seed, LUMICE_CrystalMesh* out);

// Build crystal mesh from config, apply Y-Z swap and AABB normalization,
// then upload to g_crystal_renderer. Returns the computed param hash, or 0 on failure.
int BuildAndUploadCrystalMesh(const CrystalConfig& cr, unsigned long long sample_seed);

// Access the most-recently uploaded crystal mesh (Y-Z swapped + AABB normalized).
// Returns nullptr if no mesh has been built yet or after ResetLastCrystalMesh().
// The returned pointer refers to file-scope singleton state in crystal_preview.cpp;
// the address remains stable across calls — only the pointed-to contents are
// overwritten by each BuildAndUploadCrystalMesh() call.
const LUMICE_CrystalMesh* GetLastCrystalMesh();

// Invalidate the cached mesh. Used in GUI tests to ensure per-case state isolation.
void ResetLastCrystalMesh();

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CRYSTAL_PREVIEW_HPP
