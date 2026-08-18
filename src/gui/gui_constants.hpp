#ifndef LUMICE_GUI_CONSTANTS_HPP
#define LUMICE_GUI_CONSTANTS_HPP

namespace lumice::gui {

// Layout constants
// Initial window size. Height is bound to the document column's content footprint:
// when adding new control groups / expanding existing groups, re-evaluate this
// constant to avoid spawning a scrollbar on fresh install. On constrained
// displays (e.g. 1080p + large Dock, Windows 125% scaling), main.cpp clamps
// the actual creation size via glfwGetMonitorWorkarea — see
// ClampInitWindowSize() in main.cpp / ClampWindowSizeToWorkarea() in
// window_sizing.hpp.
constexpr int kInitWindowWidth = 1600;
constexpr int kInitWindowHeight = 980;
constexpr int kMinWindowWidth = 1024;
constexpr int kMinWindowHeight = 640;
// Safe margin for OS window decorations (title bar + borders). Deducted from
// the monitor work area (which already excludes menubar/Dock/taskbar) to
// compute the usable creation size. 50 px covers the typical 28-32 px
// decoration on macOS/Windows/Linux with ~1.5x buffer.
constexpr int kWindowDecorationMargin = 50;
// The document column (tree above, inspector below). MEASURED against a floor rather than chosen:
// two independent constraints put the narrowest workable column at ~280 px, and both were read off
// a live frame by sweeping the dock node's width (the sweep itself was a throwaway probe; the
// invariants it established are the standing cases in test/gui/functional/).
//   - The shape tables' Value column is the one stretch column in a row of fixed ones, so it is
//     what a narrower panel takes from: at 280 px it is exactly as wide as "0.000" plus the frame's
//     padding, i.e. at the point where the cell stops showing the number it holds
//     (edit_modal's the_sync_column_leaves_the_slider_room asserts the same threshold).
//   - The property rows' control column carries the Lens Type combo, whose longest value
//     ("Dual Fisheye Stereographic") needs 190 px before ImGui ellipsises it — reached at 280 px
//     of panel.
// 330 is the prototype's anchor and sits ~50 px above both floors, which is the slack a label or a
// font a step larger would eat. It is not a minimum the layout must survive: the user can drag the
// separator further in, and nothing may overflow when they do — that is what
// test/gui/functional/test_inspector_no_hscroll.cpp asserts, at 300 px and at this value.
//
// It came down from 400. The extra 70 px were not doing anything a control asked for: before the
// width tokens landed, every control took its width from this constant, so a wider column simply
// made every field wider (doc/gui-visual-language.md §4.7).
constexpr float kLeftPanelWidth = 330.0f;
// The top bar is TWO rows: chrome (panel toggles / New / Open / Save / Colors / Settings / View) on
// the first, the execution cluster (Run / Stop / dirty chip / Revert / Rays / Max hits / Use GPU /
// run status) on the second. One row was measured not to fit: the chrome row alone is ~700 px and
// the execution cluster adds ~800 px, which overflows even the 1600 px default window and is far
// past kMinWindowWidth. Height = WindowPadding.y*2 + FrameHeight*2 + ItemSpacing.y with the theme's
// 15 px body font (theme.cpp) = 6*2 + 21*2 + 3 = 57, rounded up to 64 for the same slack the 40 px
// single-row value carried.
//
// MAINTAINER: this constant is the input to every fixed-chrome geometry calculation in the app —
// the dock host's origin and height (main.cpp / test_gui_main.cpp), the document column's y and
// height (RenderDocumentTree / RenderDocumentInspector call sites in app_panels.cpp), the
// viewport/display-strip band (GetCentralBand, app_panels.cpp), the collapsed-strip button centring
// (RenderCollapsedStrip's callers), and the aspect-fit solver (ResolveAspectFit, app.cpp).
// Changing it also changes the pixel height of the `visual` group's left_panel reference capture
// (test_gui_main.cpp's left-panel readback derives rh from it), so a change here requires a
// reference re-shoot for that group.
constexpr float kTopBarHeight = 64.0f;
constexpr float kStatusBarHeight = 28.0f;
// The display strip under the viewport (Grade / Overlays / Components tabs, RenderDisplayStrip in
// app_panels.cpp). Like kTopBarHeight this is MEASURED, not estimated: it is the height at which the
// tallest tab (Grade, with the conditional "Screen too small" warning showing) fits without a
// scrollbar — tab bar + WindowPadding.y*2 + the tab's own rows, read off a real capture.
// It is a fixed height on purpose: a strip that grew and shrank with the selected tab would move the
// viewport's bottom edge every time the user switched tabs. Tabs whose content exceeds it scroll
// inside their own child region instead.
constexpr float kDisplayStripHeight = 176.0f;

// Live-edit timing constants
// Invariant: kCommitIntervalMs >= kPollIntervalMs (commit should not be faster than poll)
constexpr int kCommitIntervalMs =
    70;  // Min interval between auto-commits (T_commit, ms).
         // 70ms gives Windows enough headroom: first_upload avg ~48ms + poll ~20ms needs >60ms window.
constexpr int kPollIntervalMs = 20;  // Server poll interval (T_poll, ms). Shorter than VSync frame (16.67ms at 60fps)
                                     // to ensure each frame has fresh data available via LoadSnapshot().
// Slow heartbeat used once the poller believes the run has reached its terminal state and has
// self-paused (ServerPoller::State::kIdleHeartbeat). Invariant I3's second clause — "idle may
// throttle to a slow heartbeat, but must never fall so silent it cannot self-heal"
// (doc/gui-preview-lifecycle-architecture.md §9) — is what this exists for: without it, the one
// poll that observes the terminal edge is the ONLY chance to get the terminal truth on screen,
// which turns a level-triggered reconciler back into an edge-triggered one.
// Invariant: kIdleHeartbeatIntervalMs > kPollIntervalMs (a heartbeat that is not a throttle down
// from the running cadence is not a heartbeat). The specific value is a pure UX/cost tradeoff with
// no correctness content — I3 demands "not silent", not any particular rate.
constexpr int kIdleHeartbeatIntervalMs = 500;
static_assert(kIdleHeartbeatIntervalMs > kPollIntervalMs,
              "the idle heartbeat must be a throttled-DOWN cadence relative to the running poll");
// Crystal inspector preview animation tick (ms between successive sample_seed advances while a
// shape distribution is active). ~3.3 Hz sits in the 2-4 Hz visual-comfort band (faster reads as
// noise); it is a pure UX cadence choice, not a correctness constraint.
constexpr int kCrystalPreviewAnimIntervalMs = 300;
constexpr int kTargetFrameTimeMs = 16;  // Fallback frame time limit (ms). Prevents busy-wait when VSync fails
                                        // (known issue on Windows+NVIDIA, GLFW #1559/#2049).
// Calibration window for quality gate threshold calculation (ms).
// Decoupled from kCommitIntervalMs so that changing commit timing doesn't
// accidentally tighten/loosen the quality gate.
constexpr int kCalibrationWindowMs = 50;

// Timeout for quality gate fallback (ms). If the quality gate continuously rejects uploads
// for this duration, force-upload the current buffer (may be empty → black screen).
// This handles edge cases where sim_ray_num grows very slowly (e.g. very few simulation threads).
// The main stale-texture fix is MarkStructHardDirty (anchor reset + display epoch floor) + the
// SyncFromPoller epoch-keyed upload gate; this timeout is defense-in-depth.
// Rationale: normal first upload takes 100-200ms; 500ms is 2.5-5x margin.
constexpr int kQualityGateTimeoutMs = 500;

// Floor for the adaptive quality gate threshold (min sim_ray_num for texture upload).
// After calibration, the actual threshold may be higher (adapted to platform throughput).
// This floor ensures Windows (lower throughput) never drops below a safe minimum.
constexpr unsigned long long kMinRaysFloor = 5000;

// SliderWithInput / control alignment layout constants
constexpr float kLabelColWidth = 70.0f;
constexpr float kInputWidth = 60.0f;

// ---------------------------------------------------------------------------
// Form width tokens
// ---------------------------------------------------------------------------
//
// Every control width in a form is one of the named tiers below, and a control that
// stretches to the container's edge is the exception, not the default. The rule exists
// because the opposite was measured: with ImGui's CalcItemWidth default (~65% of the
// available region) plus scattered full-width stretches, five controls that all edit "one
// scalar" ended up 90 / 110 / 230 / 240 / 600 px wide, and not one of those numbers had
// been chosen by anyone — they were all leaked from whatever container the control
// happened to sit in. Order (repetition + alignment + quantisation) is what reads as
// refinement, so the number of distinct widths a form uses is the thing to minimise
// (doc/gui-visual-language.md §2).
//
// Named exceptions — a control MAY fill its container when its content has no natural
// width: a file path / expression text input (the filter editor's OR rows), and the
// control column of a property row, whose left edge is already pinned by the label
// column so the fill IS the alignment.

// Label column of an inspector property row (BeginPropertyTable / PropertyRow, panels.hpp).
// This is a CONTENT width — ImGui table cell padding is added outside it — so the gap
// between the right-aligned label and the control's left edge comes out at CellPadding.x*2
// = 8 px, the same step as ItemSpacing.x in the size rhythm (doc/gui-visual-language.md §4.2).
//
// MEASURED, not chosen: 59.0 px is the widest label any of the inspector's six pages renders
// ("Lens Type") in the theme's 15 px Roboto Medium, rounded up to the rhythm's multiple of 4.
// It is deliberately NOT padded out to the prototype's ~104 px — a label column
// materially wider than its widest label re-opens the proximity gap that sank the
// "one shared label column on the right" direction (§5, falsified), just at a smaller
// scale. test/gui/functional/test_property_row.cpp recomputes the requirement from the
// live font atlas and fails if a font/label change outgrows this value.
constexpr float kPropertyLabelColWidth = 60.0f;

// Upper bound on the width of an inspector data table (the crystal shape-parameter table
// and its Face Distance sibling). The inspector is a dock node the user can widen without
// limit, and those tables have exactly one stretch column (Value), so every extra pixel of
// panel width lands in a single drag control: at a 900 px panel the height of a crystal in
// millimetres gets a 700 px track. The cap is the width at which that Value column matches
// the property rows' control column, i.e. the point past which the table has nothing left
// to say with more room. Below the cap the tables still stretch as before, so the default
// panel width is unaffected.
constexpr float kPropertyTableMaxWidth = 420.0f;

// A combo that sits in a TOOLBAR row rather than a property row — the crystal preview pane's
// render-style picker, which shares its row with a Reset View button, and the display strip's
// Grade row Resolution picker. A property row's combo takes the control column and needs no width
// of its own; a toolbar combo has no column to take, and letting it stretch to the row's end would
// put a four-item picker across the whole panel. 120 px is the width that pane has shipped with,
// kept rather than re-derived: it fits the longest style name with room for the dropdown arrow,
// and the resolution labels ("2048 x 2048" being the longest) fit inside the same box.
constexpr float kToolbarComboWidth = 120.0f;

// The Grade row's aspect-preset combo. It gets a tier of its own rather than kToolbarComboWidth
// because its longest option, "Match Background", does not fit in 120 px — and a combo whose
// selected value is ellipsised is the one case where "content width" has failed at the only job
// it has.
//
// MEASURED, not chosen, in the same sense as kPropertyLabelColWidth: it is
// CalcTextSize("Match Background").x + FramePadding.x*2 (the frame's own inset) + the arrow
// button's square side (GetFrameHeight()), taken on the theme's 15 px Roboto Medium and rounded
// up to the size rhythm's multiple of 4 — 142.0 px measured, hence 144. That is one rhythm step
// wider than kToolbarComboWidth's 120 plus the difference in longest label, and NOT padded out
// further: a combo materially wider than the value it shows is back to a width nobody chose.
// test/gui/functional/test_view_display_controls.cpp re-derives the requirement from the live font
// atlas and fails in BOTH directions if a font or a longer preset name outgrows this value, or if
// the slack over it exceeds one rhythm step.
constexpr float kAspectPresetComboWidth = 144.0f;

// Content-shrink tier for a single numeric field that sits in a horizontal row rather than a
// property table: the top bar's Max hits, and the display strip's Grade row EV and background
// Alpha. A field whose value is a handful of digits is sized to its content, not to the bar it
// happens to sit in. Bounds the FIELD ITEM only — the frame the value is typed into — and
// excludes the field's label, its ItemSpacing, and any surrounding group padding, so a caller
// sizing a whole labelled cluster must add those itself.
//
// Named for what it is rather than for where it was first needed: it was introduced as the top
// bar's tier, and the display strip then wanted the same "one scalar, a few digits, in a
// horizontal row" width. Two symbols for one concept is exactly the width proliferation the
// token table exists to stop.
constexpr float kCompactFieldWidth = 58.0f;

// The ray-budget control (RaysBudgetControl) in the top bar's execution cluster. It is the one
// control in this table that is NOT a content-shrink field: it keeps its [slider][input] pair,
// because the slider's rightmost band is the ∞ detent and a drag has no track to put a detent on
// (doc/gui-visual-language.md §4.5). The width therefore covers the whole pair.
//
// Chosen so the whole execution row fits inside kMinWindowWidth (1024): the row totals ~985 px
// with separators. A narrower window clips the tail of the row rather than reflowing it — the
// blueprint leaves narrow-window degradation to the implementation
// (doc/gui-layout-architecture.md §6), and clipping keeps Run, the dirty chip and the ray budget
// (the leftmost, most-used items) on screen.
constexpr float kRaysControlWidth = 170.0f;

// Reference drag track length, in pixels: the horizontal distance a DragFloat traverses to
// cross its whole domain. It is the width the retired [slider][input] pair's slider half
// used to get in the inspector, so a merged single control keeps the same "how far do I
// have to drag" feel rather than inheriting ImGui's default (range/100, i.e. 100 px for the
// full domain — more than twice as twitchy). Consumed by DragFloatField (panels.cpp), which
// derives v_speed = (max - min) / this. In logarithmic mode ImGui divides the delta by the
// domain size itself, so the same formula lands the same 230 px full-domain traversal there.
constexpr float kDragTrackReferenceWidth = 230.0f;

// Card thumbnail (offscreen crystal rendering)
// Currently used for both FBO render resolution and UI display size.
// If HiDPI support is needed later, split into separate render/display constants.
constexpr int kThumbnailSize = 96;
constexpr int kMaxThumbnailUpdatesPerFrame = 2;

// Vertical gap between stacked hover-action buttons (Delete on top, Duplicate below).
constexpr float kHoverBtnGap = 4.0f;

// Default camera zoom for the crystal renderer. Lower value → crystal fills
// more of the canvas (screen coverage ≈ 1/zoom). Must stay in sync between
// the thumbnail cache and the inspector's preview so the crystal does not
// visually jump when a row is selected.
constexpr float kDefaultCrystalZoom = 1.4f;

// Camera elevation (downward pitch) for the inspector/thumbnail crystal preview,
// in degrees. The camera sits at world (0, -dist, dist·tan(kCameraTiltDeg))
// looking at the origin, with world +z up. Implemented as a fixed rotation
// V_rot = Rx(+kCameraTiltDeg) inside CrystalRenderer::BuildViewRotation —
// the additional -90° remap from world (+z up) to OpenGL eye-space (+y up)
// is provided implicitly by the Y-Z swap in BuildCrystalMeshData, so V_rot
// only needs to add the camera elevation. The +sign on the Rx angle tilts
// world +z (mesh +y, the crystal's c-axis) TOWARD the camera so the top
// face is visible — matching the "elevated camera looking down" intent
// (kPlate shows face 1, not face 2). Mouse-drag rotates the crystal in
// world coordinates while the camera position stays put.
constexpr float kCameraTiltDeg = 15.0f;

// Auxiliary line overlay
constexpr int kMaxSunCircles = 16;

// Lens projection types. Order must match kLensTypeNames in gui_state.hpp
// and Core's LensParam::LensType enum. Static asserts in gui_state.hpp guard
// the array length; ordering itself must be kept in sync manually.
enum LensType : int {
  kLensTypeLinear = 0,
  kLensTypeFisheyeEqualArea = 1,
  kLensTypeFisheyeEquidist = 2,
  kLensTypeFisheyeStereographic = 3,
  kLensTypeDualFisheyeEqualArea = 4,
  kLensTypeDualFisheyeEquidist = 5,
  kLensTypeDualFisheyeStereographic = 6,
  kLensTypeRectangular = 7,
  kLensTypeFisheyeOrthographic = 8,
  kLensTypeDualFisheyeOrthographic = 9,
  kLensTypeGlobe = 10,
};

// Lenses whose shader path skips the view matrix (full-sphere mapping):
// dual fisheye (4-6), rectangular (7), dual orthographic (9). For these,
// elevation/azimuth/roll/FOV slider have no visual effect and are disabled in UI.
// SINGLE SOURCE OF TRUTH for the "full-sky" set.
//
// Two-layer guarding:
//   * kLensTypePresentationOrder's static_assert(== kLensTypeCount) in
//     gui_state.hpp forces a developer to ACK every new LensType — that's
//     where the "did you classify this lens?" prompt fires.
//   * The static_assert below only guards kFullSkyLensTypes itself: it
//     catches accidental edits to this array (size drift) and reminds the
//     reviewer to update the literal alongside any policy change here.
inline constexpr int kFullSkyLensTypes[] = {
  kLensTypeDualFisheyeEqualArea, kLensTypeDualFisheyeEquidist,     kLensTypeDualFisheyeStereographic,
  kLensTypeRectangular,          kLensTypeDualFisheyeOrthographic,
};
inline constexpr int kFullSkyLensTypeCount = sizeof(kFullSkyLensTypes) / sizeof(*kFullSkyLensTypes);
static_assert(kFullSkyLensTypeCount == 5,
              "kFullSkyLensTypes count changed: update both the array and this "
              "literal in lockstep with the policy change.");

inline constexpr bool LensIsFullSky(int lens_type) {
  for (int v : kFullSkyLensTypes) {
    if (v == lens_type) {
      return true;
    }
  }
  return false;
}

// Lenses whose default FOV is 180 degrees (full hemispheric fisheye family).
// SINGLE SOURCE OF TRUTH for the "FOV=180" set; do NOT confuse with
// kFullSkyLensTypes (which classifies skip-view-matrix shader paths). These
// two sets overlap but are not equal: e.g. kLensTypeRectangular is full-sky
// but its default FOV is 90, not 180.
//
// The static_assert below guards array size; ordering is enforced by
// kLensTypePresentationOrder in gui_state.hpp.
inline constexpr int kFov180LensTypes[] = {
  kLensTypeFisheyeEqualArea,     kLensTypeFisheyeEquidist,     kLensTypeFisheyeStereographic,
  kLensTypeDualFisheyeEqualArea, kLensTypeDualFisheyeEquidist, kLensTypeDualFisheyeStereographic,
};
inline constexpr int kFov180LensTypeCount = sizeof(kFov180LensTypes) / sizeof(*kFov180LensTypes);
static_assert(kFov180LensTypeCount == 6,
              "kFov180LensTypes count changed: update both the array and this "
              "literal in lockstep with the policy change.");

inline constexpr bool LensIsFov180(int lens_type) {
  for (int v : kFov180LensTypes) {
    if (v == lens_type) {
      return true;
    }
  }
  return false;
}

// Globe lens camera distance (eye-space). The camera sits at (0, 0, D) looking
// toward the unit sphere centered at the origin. Must match GLSL globeInverse
// in preview_renderer.cpp shader source (search "kGlobeCameraD" anchor in the
// shader string).
inline constexpr float kGlobeCameraD = 4.0f;

// Visible region selector (base hemisphere). Order must match kVisibleNames in gui_state.hpp.
// Front-hemisphere clipping is an independent flag (RenderConfig::front), not an enum value.
enum Visible : int {
  kVisibleUpper = 0,
  kVisibleLower = 1,
  kVisibleFull = 2,
};

}  // namespace lumice::gui

#endif  // LUMICE_GUI_CONSTANTS_HPP
