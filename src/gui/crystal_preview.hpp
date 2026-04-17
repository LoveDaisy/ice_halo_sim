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

// Rebuild the crystal mesh (if params changed since last upload) and render it into
// g_crystal_renderer's FBO using the shared rotation/zoom/style state. Intended for
// callers that need to drive the FBO explicitly (e.g. GUI visual tests that capture
// from g_crystal_renderer.GetTextureId()).
void UpdateCrystalPreviewRenderer(const CrystalConfig& cr);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CRYSTAL_PREVIEW_HPP
