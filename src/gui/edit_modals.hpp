#ifndef LUMICE_GUI_EDIT_MODALS_HPP
#define LUMICE_GUI_EDIT_MODALS_HPP

#include <optional>
#include <string>
#include <vector>

#include "include/lumice.h"

struct GLFWwindow;

namespace lumice::gui {

struct GuiState;
struct EntryCard;
struct FilterConfig;

// The filter pool's two write primitives, shared by the edit modal's OK path and the analysis
// panel's "Exclude this raypath" (analysis_panel.cpp) so a filter reaches the pool one way.
//
// PropagateFilterIdToLinked: after `entry`'s filter_id changed from `old_filter_id` to
// `new_filter_id`, move every other entry that was linked with it — same crystal_id, same
// old_filter_id — to the new id too, so the linked group (gui_state.hpp "Linked group
// invariants") stays one share unit when a filter is added to or removed from a previously
// filter-less group. A no-op when the id did not change (an in-place pool edit; siblings already
// see it through the shared slot).
void PropagateFilterIdToLinked(GuiState& state, int crystal_id, std::optional<int> old_filter_id,
                               std::optional<int> new_filter_id);

// WriteFilterToPool: bind `filter` to `entry` — overwrite the pool slot it already references, or
// append a new slot, point the entry at it and propagate the new id to its linked siblings.
void WriteFilterToPool(GuiState& state, EntryCard& entry, const FilterConfig& filter);

struct GuiState;
struct EditRequest;
enum class EditTarget;

// Process an edit request from the card UI. Must be called from the left panel
// after detecting a non-None EditRequest (before ResetEditRequest).
void OpenEditModal(const EditRequest& req, GuiState& state);

// Render all modal popups (Crystal/Axis/Filter). Called from the main loop
// outside any Begin/End block, in the same scope as RenderUnsavedPopup.
// `window` is used to clamp the modal's max size to the workarea of the
// monitor containing the window center; pass nullptr in headless test
// harnesses (the multi-monitor clamp then falls back to unbounded max).
void RenderEditModals(GuiState& state, GLFWwindow* window);

// Returns true when the unified edit modal is open (any tab). Used by
// visual-smoke tests to skip FBO drive while the modal owns the FBO via its
// per-frame g_crystal_renderer.Render() call — the preview is now a
// persistent left pane shared across Crystal / Axis / Filter tabs, so the
// gate condition is modal-open rather than crystal-tab-active.
bool IsEditModalOpen();

// (layer_idx, entry_idx) of the entry the open modal is bound to. Returns
// {-1, -1} as a safe sentinel when no modal is open, so callers may either
// check IsEditModalOpen() first (more readable) or just inspect the returned
// indices and treat negatives as "no target". This function is meant for UI
// lifecycle reverse-annotation (e.g. highlighting the source card while the
// modal is open), not for modal control flow.
struct EditModalTarget {
  int layer_idx;
  int entry_idx;
};
EditModalTarget GetEditModalTarget();

// Keep the open modal's (layer_idx, entry_idx) binding meaning the same ENTRY across a delete.
//
// The binding names a position, so the one operation that can change what it means is an erase:
// every later element shifts up one and the same numbers now denote a different entry. Call the
// matching function immediately after erasing, from the delete site itself — that is the only place
// that knows which index went away, and routing it through here keeps the rule in one function
// instead of a bounds check at each of the places that read the binding.
//
// Three outcomes, in both flavours: the deleted item IS the bound one (close the modal — it has
// nothing left to edit), the deleted item sat BEFORE it (decrement, so the binding follows its
// entry down), or after it (nothing to do). A no-op when no modal is open: the stale indices left
// behind are overwritten wholesale by the next OpenEditModal.
//
// Only the modal's own bounds guard in RenderEditModals catches the remaining case, the binding
// falling off the end of the vector; everything short of that is in range and silently wrong,
// because Immediate mode writes the edit buffers into the bound entry's pool slots every frame.
//
// The layer flavour has a second consumer: every ColorClassRefConfig in state.raypath_color holds
// a positional layer_idx into state.layers too, and it is NOT gated on a modal being open. After
// an erase those refs are re-indexed by the same three-outcome rule (deleted layer -> the ref is
// left dangling at -1, so ResolveColorRef reports kLayerMissing; a layer before it -> untouched;
// after it -> decrement). Without this the ref keeps its old number and silently denotes the
// layer that shifted into that slot — and when that layer happens to reuse the same crystal pool
// slot, ResolveColorRef's two checks (bounds, crystal-in-layer) both pass and nothing tells the user.
void NotifyEntryDeleted(int layer_idx, int deleted_entry_idx);
void NotifyLayerDeleted(GuiState& state, int deleted_layer_idx);

// Returns the EditTarget corresponding to the currently active tab. Returns
// EditTarget::kCrystal when no modal is open because ResetModalState() resets
// g_active_tab to kCrystal on close. Intended solely for resolving the tab in
// kCard edit requests — do not use for other purposes.
EditTarget GetActiveTabAsEditTarget();

// Arm the "Link to..." eyedropper on (layer_idx, entry_idx): the next entry card the user clicks
// becomes the model whose crystal / filter this entry adopts.
//
// Two entry points reach this — the entry card's rail button and the edit modal's own
// "Link to..." — and they must not each carry their own copy of the sequence, because the part
// that is easy to leave out is invisible when it is missing: if the modal is open, its edit
// buffers hold changes that have not been applied to the pool yet, and arming pick mode without
// committing them first silently discards whatever the user just typed. That commit is why this
// lives in edit_modals (the buffers and the open-modal statics are its), not in panels.
//
// Does NOT call ImGui::CloseCurrentPopup(): that is only legal from inside the popup's own scope,
// so the modal call site adds it and the card call site does not.
void StartLinkPickMode(GuiState& state, int layer_idx, int entry_idx);

// Reset all modal-internal static state (active modal, edit buffers, pending flags).
// Called by test teardown (ResetTestState) to prevent state leakage between tests.
void ResetModalState();

// Render the custom-spectrum editor modal (independent of the Crystal/Axis/Filter per-entry
// modal above). The Sun panel's Spectrum combo calls OpenSpectrumModal() when the user picks
// "Custom..."; this function must be called each frame (from the main loop, next to
// RenderEditModals) to actually paint the popup while it is open.
void OpenSpectrumModal(GuiState& state);
void RenderSpectrumModal(GuiState& state);

// Returns true when the committed axis config of the currently open modal entry
// meets D-symmetry conditions (az uniform 360°, roll mean a multiple of 30°).
// Returns false when no modal is open or the entry index is invalid.
// Intended for GUI test assertions; production code should not call this.
bool IsCurrentModalDApplicable();

// One row of the wedge-angle preset dropdown. It declares only the Miller indices (h, l; k is
// always 0 — that is what the {h,0,-h,l} notation means, not an omission); `label` and `value` are
// filled in from ConvertMillerIndexToWedgeAngle's answer the first time the table is asked for.
// Nothing here is a hand-written angle, which is the point: the four constants this replaced had
// been transcribed with the ratio inverted and stayed wrong from the day they were written.
//
// Declared here rather than in the .cpp's anonymous namespace so the unit test can read the same
// table the dropdown renders, instead of keeping a second copy of it to compare against.
struct WedgePreset {
  int h;
  int l;
  char label[32];
  float value;
};

// The wedge-angle presets a user may pick from: the four built-ins, then whatever this session's
// personal defaults added (GetUserWedgePresets, user_defaults.hpp), de-duplicated on Miller indices.
// Both the render path (RenderWedgeTableRow) and the unit test call this one function — there is no
// test-only variant, because a second entry point is a second thing that can be right while the
// first is wrong.
//
// RECOMPUTED ON EVERY CALL, and BY VALUE for that reason. It used to be a function-local static
// built once and handed out as a pointer, which was correct while its content was four compile-time
// constants. It no longer is: a user can add or delete an entry mid-session, so a caller holding
// the previous call's rows is holding a list that has since changed. Returning a vector makes that
// contract something the type system enforces rather than something a comment asks the reader to
// remember. The cost is a handful of atan() calls on the frames a dropdown is open.
std::vector<WedgePreset> GetWedgePresets();

// A Miller triple in the four-index notation this UI writes everywhere: "{h,k,i,l}", with the
// redundant i derived as -(h+k) rather than taken from the caller, for the same reason the custom
// input row displays it rather than accepting it.
//
// The single owner of that notation. It is spelled in three places — the dropdown's preset rows,
// the Settings panel's saved rows, and the same panel's rows for a triple the owner refuses (which
// FormatWedgePresetLabel below cannot render, since those can carry a non-zero k and have no angle
// to print). One function so the day the notation changes is one edit.
std::string FormatMillerIndices(int h, int k, int l);

// The label a preset row shows: the notation above, then the angle in the same precision the wedge
// slider's input box uses, so the dropdown and the box agree once a preset is picked. k is 0 by
// construction — a preset only exists for a triple the owner accepted, and it accepts none with a
// non-zero k.
//
// Public because the Settings panel's preset library renders saved entries too.
std::string FormatWedgePresetLabel(int h, int l, float angle_deg);

// Is this triple one of the four built-in presets? False for every k != 0 (no built-in has one).
//
// For the Settings panel, which must refuse to save a shortcut the dropdown already offers. It asks
// here rather than re-enumerating the built-in table, which is file-local to edit_modals.cpp on
// purpose: that table is the thing 523.2 had to correct, and a second listing is a second place to
// correct. (GetWedgePresets() needs no such call — by the time it merges the user's rows the
// built-ins are already in the list it de-duplicates against.)
bool IsBuiltInWedgeMillerIndex(int h, int k, int l);


// What the wedge dropdown's custom-input row should say about one Miller-index triple, in the form
// the popup renders it: an angle to show, a message to show beside it, and whether Apply is live.
//
// It holds no rule of its own. Every field below is decided by
// LUMICE_ConvertMillerIndexToWedgeAngle and then looked up on the (state, invalid_index) pair it
// returns -- see the mapping table in edit_modals.cpp. Nothing here re-reads h/k/l to form a second
// opinion about whether they are legal, which is the whole point: config, server and GUI each kept
// a transcription of the bare formula once, and the copies then disagreed with core about what
// h == 0 means.
struct CustomWedgeInputFeedback {
  LUMICE_MillerConversionState state = LUMICE_MILLER_INCOMPLETE;
  // Meaningful for LUMICE_MILLER_VALID and LUMICE_MILLER_NO_CONE only; 0 elsewhere means "no
  // opinion", not "zero degrees" -- the same contract the C API states for its out_angle_deg.
  float angle_deg = 0.0f;
  // True for LUMICE_MILLER_VALID alone. NO_CONE is excluded deliberately, not by omission: its
  // honest answer is 0 degrees, and the wedge slider's domain starts at 0.1, so writing it would
  // be silently clamped up into a different crystal. See edit_modals.cpp.
  bool can_apply = false;
  // Empty exactly when there is nothing to tell the user (LUMICE_MILLER_VALID).
  std::string message;
};

// Declared here rather than left file-local so the unit test calls the very function the popup
// calls. A test-only second copy of the mapping would be a second thing that can be right while
// the one users see is wrong.
CustomWedgeInputFeedback EvaluateCustomWedgeInput(int h, int k, int l);

// The three Miller-index boxes plus their live feedback line, with no confirm button of its own.
//
// Split out of the dropdown's custom-input row so the Settings panel's "add a preset" row is the
// SAME control rather than a second one that looks like it: what a triple means, which grade its
// message carries and when it may be committed are decided here once, and the caller only decides
// what its own button does with the verdict (write an angle into a crystal, or append a saved
// preset).
//
// `storage_prefix` keys the boxes' contents in ImGui's per-window storage. Every concurrently live
// caller must pass a distinct one — Upper A and Lower A already do, because SliderWithPresetEdit
// runs twice per frame and a shared key would make what the user typed under one reappear under the
// other.
//
// `out_h` / `out_k` / `out_l` receive what is in the boxes right now (never null); the return value
// is that triple's verdict.
CustomWedgeInputFeedback RenderMillerIndexInputRow(const char* storage_prefix, int* out_h, int* out_k, int* out_l);

}  // namespace lumice::gui

#endif  // LUMICE_GUI_EDIT_MODALS_HPP
