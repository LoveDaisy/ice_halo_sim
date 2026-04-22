#ifndef LUMICE_GUI_CRYSTAL_PREVIEW_HPP
#define LUMICE_GUI_CRYSTAL_PREVIEW_HPP

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
void ResetCrystalView();
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
