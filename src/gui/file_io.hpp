#ifndef LUMICE_GUI_FILE_IO_HPP
#define LUMICE_GUI_FILE_IO_HPP

#include <filesystem>
#include <string>
#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

struct GuiState;
struct PreviewViewport;
class PreviewRenderer;

// Identifies which filter reference triggered an ABI-bounds overflow inside FillLumiceConfig.
// Populated only when FillLumiceConfig returns false AND the caller passed a non-null pointer.
// Captures the FIRST reference walked in scattering-layer order — same filter pool id reused
// across multiple (layer, entry) sites only reports the first, which is sufficient to locate
// the offending filter (all sites share identical content).
struct FilterOverflowInfo {
  int layer_index = -1;     // 0-based scattering layer index (present to caller as +1)
  int entry_index = -1;     // 0-based entry index within the layer (present to caller as +1)
  std::string filter_name;  // FilterConfig::name (may be empty if user did not name the filter)
};

// task-342.3 (Step 3): identifies which raypath color-class ref triggered an ABI-bounds
// overflow inside FillLumiceConfig (LUMICE_MAX_CONFIG_COLOR_CLASSES / _COLOR_REFS caps).
// Populated only when FillLumiceConfig returns false due to color-class overflow AND the
// caller passed a non-null pointer. class_index is 0-based (+1 for user-facing display);
// ref_index is -1 for a class-cap overflow (too many classes) or 0-based when the class
// itself has too many refs. class_over_cap distinguishes the class-cap case from a ref-cap
// case (ref_index == -1 with class_over_cap == false is unused).
struct ColorClassOverflowInfo {
  int class_index = -1;         // 0-based color-class index (present to caller as +1); -1 = no overflow
  int ref_index = -1;           // 0-based ref index within the class; -1 for class-cap overflow
  bool class_over_cap = false;  // true = state.raypath_color.size() exceeds _COLOR_CLASSES
};

// Fill LUMICE_Config C struct from GuiState (for LUMICE_CommitConfigStruct, bypasses JSON serialization)
// Fill a LUMICE_Config from GuiState for the typed-struct commit path. Returns false if a
// filter expansion exceeded the ABI bounds (composition pool / clause / filter capacity)
// OR a raypath color class/ref exceeded LUMICE_MAX_CONFIG_COLOR_{CLASSES,REFS}. In either
// case the caller must NOT commit `out` and should keep the prior committed state.
// When `filter_overflow` is non-null and the return is false due to a filter overflow, it
// receives the identity of the first offending (layer, entry, filter name). Symmetric
// contract for `color_overflow` on color-class overflow. Only one overflow is reported per
// call (whichever is hit first, in file order: filter walk before color walk).
bool FillLumiceConfig(const GuiState& state, LUMICE_Config* out, FilterOverflowInfo* filter_overflow = nullptr,
                      ColorClassOverflowInfo* color_overflow = nullptr);

// Format a human-readable locator ("filter \"NAME\", Layer L / Entry E", or just
// "Layer L / Entry E" when the filter is unnamed) from a FilterOverflowInfo, using 1-based
// Layer/Entry to match the panel header convention (panels.cpp "Layer %d"). Extracted as a pure
// function so the message format is unit-testable (test_gui_import_export.cpp) instead of only
// exercised through on-screen GUI. Returns the inner text WITHOUT surrounding parentheses so the
// caller can embed it inside its own "(limit N; ...)" grouping.
std::string FormatOverflowLocator(const FilterOverflowInfo& overflow);

// Serialize GuiState to Core JSON string (for .lmc save and CLI compatibility)
std::string SerializeCoreConfig(const GuiState& state);

// Build the export Core-JSON for `state`, OR reject with a user-facing warning when a
// filter's expansion exceeds the ABI clause/term bounds (which SerializeFilterForCore
// would otherwise degrade to a semantically-opposite match-all stand-in). Pure — no file
// dialog / filesystem — so the reject path is directly unit-testable (the file-dialog wrapper
// DoExportConfigJson only supplies the path). Returns true and fills *out_json on success;
// returns false and fills *out_warning (with a FormatOverflowLocator-bearing message) on
// overflow, leaving *out_json untouched. Either out-param may be null.
bool BuildExportJsonOrWarn(const GuiState& state, std::string* out_json, std::string* out_warning);

// Deserialize Core JSON string to GuiState, returns true on success
bool DeserializeFromJson(const std::string& json_str, GuiState& state);

// Serialize GuiState to full JSON (for .lmc file, preserves all frontend params)
std::string SerializeGuiStateJson(const GuiState& state);

// Deserialize full GuiState JSON, returns true on success
bool DeserializeGuiStateJson(const std::string& json_str, GuiState& state);

// Save .lmc binary file
bool SaveLmcFile(const std::filesystem::path& path, const GuiState& state, const PreviewRenderer& preview,
                 bool save_texture);

// Load .lmc binary file. If texture data is present, returns it via tex_data/tex_w/tex_h.
bool LoadLmcFile(const std::filesystem::path& path, GuiState& state, std::vector<unsigned char>& tex_data, int& tex_w,
                 int& tex_h);

// Export preview as PNG (renders via FBO, must be called on GL thread).
// Thin wrapper over RenderExportToRgba + WriteRgbaBufferToPng.
bool ExportPreviewPng(const std::filesystem::path& path, PreviewRenderer& renderer, const PreviewViewport& vp);

// Write an already-rendered RGBA8 top-down buffer to disk as a PNG.
// Shared by ExportPreviewPng and DoExportPreviewPng so PNG I/O lives in one place.
[[nodiscard]] bool WriteRgbaBufferToPng(const std::filesystem::path& path, int w, int h,
                                        const std::vector<unsigned char>& rgba);

// Export configuration as JSON (CLI-compatible format)
bool ExportConfigJson(const std::filesystem::path& path, const std::string& json_str);

// File dialog wrappers (return empty path on cancel)
std::filesystem::path ShowOpenDialog();
std::filesystem::path ShowSaveDialog();
std::filesystem::path ShowExportPngDialog();
std::filesystem::path ShowExportDualFisheyeEqualAreaDialog();
std::filesystem::path ShowExportEquirectangularDialog();
std::filesystem::path ShowExportJsonDialog();
std::filesystem::path ShowOpenImageDialog();


// Parse dash/comma-separated raypath text into face indices (tolerant: skips invalid tokens).
std::vector<int> ParseRaypathText(const std::string& text);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_FILE_IO_HPP
