#ifndef LUMICE_GUI_CRYSTAL_PREVIEW_HPP
#define LUMICE_GUI_CRYSTAL_PREVIEW_HPP

namespace lumice::gui {

struct CrystalConfig;

// Crystal preview trackball state
extern float g_crystal_rotation[16];
extern float g_crystal_zoom;
extern int g_crystal_style;
extern int g_crystal_mesh_id;
extern int g_crystal_mesh_hash;

// Crystal preview helpers
int CrystalParamHash(const CrystalConfig& c);
void ResetCrystalView();
void ApplyTrackballRotation(float dx, float dy);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CRYSTAL_PREVIEW_HPP
