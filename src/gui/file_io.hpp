#ifndef LUMICE_GUI_FILE_IO_HPP
#define LUMICE_GUI_FILE_IO_HPP

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "include/lumice.h"

namespace lumice::gui {

// RAII owner for a LUMICE_Scene handle. BuildScene (below) hands ownership to the caller as a
// ScenePtr rather than a raw handle so no GUI code path — however many early returns it grows —
// can leak one: there is deliberately not a single bare LUMICE_SceneDestroy call in src/gui/.
struct SceneDeleter {
  void operator()(LUMICE_Scene* scene) const { LUMICE_SceneDestroy(scene); }
};
using ScenePtr = std::unique_ptr<LUMICE_Scene, SceneDeleter>;

struct GuiState;
struct PreviewViewport;
struct FilterConfig;
class PreviewRenderer;

// Identifies which filter reference triggered an ABI-bounds overflow inside BuildScene.
// Populated only when BuildScene returns nullptr AND the caller passed a non-null pointer.
// Captures the FIRST reference walked in scattering-layer order — same filter pool id reused
// across multiple (layer, entry) sites only reports the first, which is sufficient to locate
// the offending filter (all sites share identical content).
struct FilterOverflowInfo {
  int layer_index = -1;     // 0-based scattering layer index (present to caller as +1)
  int entry_index = -1;     // 0-based entry index within the layer (present to caller as +1)
  std::string filter_name;  // FilterConfig::name (may be empty if user did not name the filter)
};

// task-342.3 (Step 3): identifies which raypath color-class ref triggered an ABI-bounds
// overflow inside BuildScene (LUMICE_MAX_CONFIG_COLOR_CLASSES / _COLOR_REFS caps).
// Populated only when BuildScene returns nullptr due to color-class overflow AND the
// caller passed a non-null pointer. class_index is 0-based (+1 for user-facing display);
// ref_index is -1 for a class-cap overflow (too many classes) or 0-based when the class
// itself has too many refs. class_over_cap distinguishes the class-cap case from a ref-cap
// case (ref_index == -1 with class_over_cap == false is unused).
struct ColorClassOverflowInfo {
  int class_index = -1;         // 0-based color-class index (present to caller as +1); -1 = no overflow
  int ref_index = -1;           // 0-based ref index within the class; -1 for class-cap overflow
  bool class_over_cap = false;  // true = state.raypath_color.size() exceeds _COLOR_CLASSES
};

// Which consumer a BuildScene result is destined for. The two differ in exactly ONE field —
// renderer intensity_factor — and that divergence is deliberate (doc/ev-pipeline-architecture.md
// §2.4): the GUI carries manual + auto EV at display time, the CLI has no display-time EV concept
// at all, so a config exported for the CLI must bake the current exposure to render at the same
// brightness the user is looking at. Everything else about the two scenes is identical.
enum class SceneIntent {
  kSimCommit,   // GUI Run / startup calibration → LUMICE_CommitScene. intensity_factor stays at
                // the core default (1.0); baking EV here would double-apply it against the
                // display-time push (app.cpp RefreshPreviewParams / LUMICE_SetCompositeExposure).
  kJsonExport,  // "Export Config JSON" for the CLI → LUMICE_SceneToJson. intensity_factor bakes
                // 2^exposure_offset so `Lumice -f exported.json` matches the on-screen brightness.
};

// Build a LUMICE_Scene from GuiState: the single GUI→core assembly path, feeding both the
// simulation commit (LUMICE_CommitScene) and the JSON export (LUMICE_SceneToJson) via `intent`.
//
// Returns a ScenePtr owning a fresh handle on success, nullptr if a filter expansion exceeded
// the ABI bounds (clause / term / filter capacity) OR a raypath color class/ref exceeded
// LUMICE_MAX_CONFIG_COLOR_{CLASSES,REFS}. On failure NO handle is produced (nothing to clean up)
// and the caller must keep the prior committed state.
// When `filter_overflow` is non-null and the return is nullptr due to a filter overflow, it
// receives the identity of the first offending (layer, entry, filter name). Symmetric
// contract for `color_overflow` on color-class overflow. Only one overflow is reported per
// call (whichever is hit first, in file order: filter walk before color walk).
ScenePtr BuildScene(const GuiState& state, SceneIntent intent, FilterOverflowInfo* filter_overflow = nullptr,
                    ColorClassOverflowInfo* color_overflow = nullptr);

// Format a human-readable locator ("filter \"NAME\", Layer L / Entry E", or just
// "Layer L / Entry E" when the filter is unnamed) from a FilterOverflowInfo, using 1-based
// Layer/Entry to match the panel header convention (panels.cpp "Layer %d"). Extracted as a pure
// function so the message format is unit-testable
// (test/composition-correctness/gui/test_filter_reconstruct_chain.cpp,
// FilterReconstructChain.TheLiveClauseCountIsTheSameArithmeticTheCommitEnforces) instead of only
// exercised through on-screen GUI. Returns the inner text WITHOUT surrounding parentheses so the
// caller can embed it inside its own "(limit N; ...)" grouping.
std::string FormatOverflowLocator(const FilterOverflowInfo& overflow);

// Format a human-readable, limit-bearing description of a ColorClassOverflowInfo — the
// color-class-overflow counterpart to FormatOverflowLocator above. Pure so the message format
// is unit-testable. Only valid to call when the overflow struct was actually populated (i.e.
// `overflow.class_index >= 0`, the same discriminator callers use to tell a color-class
// overflow apart from a physical-filter overflow after BuildScene returns nullptr).
std::string FormatColorOverflowLocator(const ColorClassOverflowInfo& overflow);

// Build the export Core-JSON for `state` (BuildScene with SceneIntent::kJsonExport, serialized
// via LUMICE_SceneToJson), OR reject with a user-facing warning when the state exceeds the ABI
// clause/term/color bounds. The SOLE producer of core-config JSON on the GUI side — export and
// simulation therefore cannot drift, because both encode the same LUMICE_Scene through the same
// C API encoders. Pure — no file dialog / filesystem — so the reject path is directly
// unit-testable (the file-dialog wrapper DoExportConfigJson only supplies the path). Returns
// true and fills *out_json (pretty-printed, 2-space indent) on success; returns false and fills
// *out_warning (with a FormatOverflowLocator-bearing message) on overflow, leaving *out_json
// untouched. Either out-param may be null.
bool BuildExportJsonOrWarn(const GuiState& state, std::string* out_json, std::string* out_warning);

// task-gui-feedback-affordances Step 3 (AC4): summary of how many post-Cartesian
// clauses a SoP would expand to and whether the ABI clause/term cap was hit.
// Delegates to the same ExpandSopToClauses used by BuildScene, so the live
// preview cannot drift from the commit path
// (issue.md hard constraint — no re-implementation on the GUI side). Meant
// only for cheap on-typing preview: value bounded by LUMICE_MAX_CONFIG_CLAUSES.
struct SopExpansionSummary {
  // 0..LUMICE_MAX_CONFIG_CLAUSES; degenerate default is 1. 0 means every row of the SoP was
  // blank, i.e. the filter states nothing and commits as no filter at all — a legitimate
  // result, not an error (the expansion used to substitute a match-all clause here).
  size_t clause_count = 0;
  bool overflow = false;  // true iff the expansion tripped the clause/term cap
};
SopExpansionSummary SummarizeSopExpansion(const FilterConfig& f);

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
//
// All-or-nothing on `state`: on success it is replaced wholesale by the file's document; on
// failure it is not written at all, so a caller may pass the live document straight in. The two
// halves of that are one contract — the JSON section is deserialized well before the texture
// section is even read, and the deserializer clears the document it is given, so deserializing in
// place would let a failed load leave the caller showing the file it just refused to open.
//
// `tex_data`/`tex_w`/`tex_h` carry NO such guarantee: they are cleared on entry and are only
// meaningful when this returns true. Read them only then.
bool LoadLmcFile(const std::filesystem::path& path, GuiState& state, std::vector<unsigned char>& tex_data, int& tex_w,
                 int& tex_h);

// Returns the number of crystal shape distributions downgraded to uniform since the last call, and
// resets the counter. The GUI edits uniform distributions only; non-uniform families loaded from
// JSON are converted on load (see ParseShapeDist). Call after a load to decide whether to show the
// "some distributions were simplified" notice. Call once before a load to discard any stale count.
int TakeShapeDistDowngradeCount();

// Returns the number of filters dropped since the last call because the file described no rule for
// them, and resets the counter. A `.lmc` filter object that yields no predicate (an empty
// `summands` array, a legacy form with no raypath text, a filter type the GUI no longer has) loads
// as no filter at all rather than as one that happens to match everything — under filter_out the
// latter excludes every ray. Same shape and same call discipline as TakeShapeDistDowngradeCount():
// call once before a load to discard any stale count, and again after it to decide whether to show
// the notice.
int TakeFilterNoPredicateDowngradeCount();

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
