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
void ApplyTrackballRotation(float dx, float dy);

// Build crystal mesh data from config: JSON construction, LUMICE_GetCrystalMesh, Y-Z swap, AABB normalization.
// Caller allocates LUMICE_CrystalMesh on stack (fixed-size arrays, no free needed). Returns true on success.
bool BuildCrystalMeshData(const CrystalConfig& cr, LUMICE_CrystalMesh* out);

// Build crystal mesh from config JSON, apply Y-Z swap and AABB normalization,
// then upload to g_crystal_renderer. Returns the computed param hash, or 0 on failure.
int BuildAndUploadCrystalMesh(const CrystalConfig& cr);

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
