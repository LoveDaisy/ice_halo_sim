#include "gui/field_editor_registry.hpp"

#include <algorithm>
#include <unordered_map>
#include <utility>

#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/panels.hpp"
#include "gui/slider_format_rules.hpp"
#include "gui/theme.hpp"
#include "imgui.h"
#include "include/lumice.h"
#include "util/fatal.hpp"

namespace lumice::gui {

namespace {

// ------------------------------------------------------------------------------------------------
// Entry factories.
//
// Each factory takes the field's domain ONCE and hands the same closure to both halves of the
// entry, so the bound the range check uses and the bound the widget clamps to cannot disagree. The
// main UI's call site is a third reader of that same closure (through ConstraintFor) rather than a
// copy of it — see the header.
// ------------------------------------------------------------------------------------------------

// Whether the field applies at all in the current configuration, and what to tell the user when it
// does not. This IS the BeginDisabled(...) expression at the field's main-UI call site: that call
// site passes `!ConstraintFor(key, state).enabled` rather than recomputing the condition.
struct Applicability {
  bool enabled = true;
  // Named to match FieldEditorConstraint::disabled_reason, which this becomes: keeping the two
  // spellings in sync avoids a reader having to hold "these are the same thing" in their head.
  const char* disabled_reason = nullptr;
};
using ApplicabilityFn = std::function<Applicability(const GuiState&)>;
using FloatDomainFn = std::function<std::pair<float, float>(const GuiState&)>;

Applicability AlwaysApplies(const GuiState&) {
  return Applicability{};
}

// A constant [min,max], written once at the call site below.
FloatDomainFn FixedDomain(float min_value, float max_value) {
  return [min_value, max_value](const GuiState&) { return std::pair<float, float>(min_value, max_value); };
}

std::string HiddenLabel(const char* id_base) {
  // "##" hides the text and keeps the id: the field's name already lives in the table's key column,
  // so a second copy inside the cell would be noise.
  return "##" + std::string(id_base);
}

// Write `next` back into `slot` only if it is a genuine edit rather than the widget's own clamp.
//
// THE POINT: an override file can hold a value outside the domain (nothing rewrites it until the
// user touches that control), and the panel has to be able to SHOW that — the notice column exists
// for it. A live control renders such a value by clamping it, and clamping is a write. Comparing
// against the CLAMPED original instead of the raw one separates the two cases exactly:
//   * in-domain value (the ordinary case): clamped original == original, so any difference is the
//     user's, and an out-of-range entry lands on the same bound the main UI would clamp it to;
//   * out-of-domain value: the control's own clamp produces exactly the clamped original, which is
//     therefore NOT written back — merely opening the panel leaves the document alone.
// The residue is one edge: asking, on an out-of-domain row, for precisely the bound its value
// already clamps to is indistinguishable from not asking at all, and does nothing. Any other value
// commits and takes the field back into the domain, which is the way out of that state.
template <typename T>
bool CommitIfEdited(T& slot, T next, T low, T high) {
  const T clamped_original = std::clamp(slot, low, high);
  if (next == clamped_original) {
    return false;
  }
  slot = next;
  return true;
}

// Per-field drag scratch for the two slider field kinds — the scalar counterpart of
// ColorFieldDragState further down, and there for the same reason: the value the control edits has
// to SURVIVE the frames of a drag somewhere other than the document, because the document must not
// see any of them (see CommitIfEdited's callers and the release-only commit rule they implement).
//
// Why it cannot just be re-read from the slot every frame, which is what this replaces: on the
// frame the mouse comes up, the slider does not write the value again — it only reports that its
// interaction ended. A working copy refreshed that frame would therefore hold the PRE-drag value at
// exactly the moment the commit is finally allowed, the commit would compare equal and do nothing,
// and every drag would be silently discarded while the input box beside it kept working. Refreshing
// only on frames where nothing is being held (SliderWithInput's `active` out-param) is what makes
// the release frame read back the value the drag actually left behind.
template <typename T>
struct ScalarDragState {
  T working{};
  bool was_active = false;
};

FieldEditorEntry FloatField(float* (*access)(GuiState&), FloatDomainFn domain, const char* fmt,
                            SliderScale scale = SliderScale::kLinear, ApplicabilityFn applicable = AlwaysApplies) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kFloatSlider;
  entry.Constraint = [domain, applicable, fmt, scale](const GuiState& state) {
    const auto range = domain(state);
    const Applicability a = applicable(state);
    return FieldEditorConstraint{ a.enabled, a.disabled_reason, true, range.first, range.second, fmt, scale };
  };
  entry.Render = [access, domain, fmt, scale](GuiState& state, const char* id_base) {
    const auto range = domain(state);
    float* slot = access(state);
    // Keyed by the slot address, exactly as ColorField's map is and for the identical reason —
    // that address is stable for the life of the process, so one map keeps every float field's
    // in-progress drag independent of every other's.
    static std::unordered_map<const float*, ScalarDragState<float>> drag_states;
    ScalarDragState<float>& drag = drag_states[slot];
    // Not being held => the document is the truth and the scratch follows it. This is also the
    // first-ever call for this field (a default-constructed entry has was_active == false), so the
    // scratch never renders a zero it invented.
    if (!drag.was_active) {
      drag.working = *slot;
    }
    bool committed = false;
    bool active = false;
    SliderWithInput(id_base, &drag.working, range.first, range.second, fmt, scale, LabelPlacement::kNone, &committed,
                    &active);
    drag.was_active = active;
    if (!committed) {
      return false;
    }
    return CommitIfEdited(*slot, drag.working, range.first, range.second);
  };
  return entry;
}

FieldEditorEntry IntField(int* (*access)(GuiState&), int min_value, int max_value,
                          ApplicabilityFn applicable = AlwaysApplies) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kIntSlider;
  entry.Constraint = [min_value, max_value, applicable](const GuiState& state) {
    const Applicability a = applicable(state);
    return FieldEditorConstraint{ a.enabled, a.disabled_reason, true, static_cast<double>(min_value),
                                  static_cast<double>(max_value) };
  };
  entry.Render = [access, min_value, max_value](GuiState& state, const char* id_base) {
    int* slot = access(state);
    static std::unordered_map<const int*, ScalarDragState<int>> drag_states;  // see FloatField above
    ScalarDragState<int>& drag = drag_states[slot];
    if (!drag.was_active) {
      drag.working = *slot;
    }
    bool committed = false;
    bool active = false;
    SliderIntWithInput(id_base, &drag.working, min_value, max_value, LabelPlacement::kNone, &committed, &active);
    drag.was_active = active;
    if (!committed) {
      return false;
    }
    return CommitIfEdited(*slot, drag.working, min_value, max_value);
  };
  return entry;
}

FieldEditorEntry BoolField(bool* (*access)(GuiState&), ApplicabilityFn applicable = AlwaysApplies) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kCheckbox;
  entry.Constraint = [applicable](const GuiState& state) {
    const Applicability a = applicable(state);
    return FieldEditorConstraint{ a.enabled, a.disabled_reason, false, 0.0, 0.0 };
  };
  entry.Render = [access](GuiState& state, const char* id_base) {
    return Checkbox(HiddenLabel(id_base).c_str(), access(state));
  };
  return entry;
}

// Per-field drag scratch for ColorField, keyed by the field's own slot address below — that address
// is stable for the lifetime of the process (GuiState is one long-lived object; MakeNewDocumentState()
// assigns new field values into it but never relocates it), so a single map shared by all colour
// fields correctly keeps each field's in-progress picker state independent of the others.
struct ColorFieldDragState {
  float working[3] = { 0.0f, 0.0f, 0.0f };
  bool was_active = false;
};

// A colour triple. No numeric domain on purpose: ColorEdit3 has no min/max to restate, which is
// also why the three fields with no main-UI colour control at all (see the registry below) carry no
// risk of inventing a constraint that disagrees with one.
FieldEditorEntry ColorField(float* (*access)(GuiState&)) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kColor;
  entry.Constraint = [](const GuiState&) { return FieldEditorConstraint{}; };
  entry.Render = [access](GuiState& state, const char* id_base) {
    float* slot = access(state);
    // NoInputs — the swatch only, exactly the form the main UI's overlay rows use, and the only one
    // that fits a table cell. Clicking it opens a picker popup whose hue/sat/value bars are ordinary
    // continuous drag widgets.
    //
    // ImGui::IsItemDeactivatedAfterEdit() on THIS call does not work for that popup, unlike it does
    // for FloatField/IntField's slider: ColorEdit4 remaps its own LastItemData.ID to the popup's
    // active id only while `g.ActiveId != 0` (imgui_widgets.cpp: "so IsItemActive() will function on
    // ColorEdit4()") — but on the exact frame the mouse is released, ActiveId has already gone back
    // to 0 by the time that remap line runs, so the remap never happens for the release frame, and
    // IsItemDeactivatedAfterEdit() (which compares the current item id against *last* frame's active
    // id) never finds a match to call deactivated. The naive `if (!IsItemDeactivatedAfterEdit())
    // return false;` gate therefore never commits at all — verified by driving an actual picker drag
    // in gui_test's defaults_panel/a_colour_cell_commits_on_release_not_per_frame
    // (test/gui/functional/test_defaults_panel.cpp). IsItemActive() alone, on the other hand,
    // IS set correctly by that same remap while the drag is in progress, so track the active/inactive
    // edge by hand instead of trusting the built-in "after edit" helper.
    static std::unordered_map<const float*, ColorFieldDragState> drag_states;
    ColorFieldDragState& drag = drag_states[slot];
    if (!drag.was_active) {
      drag.working[0] = slot[0];
      drag.working[1] = slot[1];
      drag.working[2] = slot[2];
    }
    ImGui::ColorEdit3(HiddenLabel(id_base).c_str(), drag.working, ImGuiColorEditFlags_NoInputs);
    const bool active_now = ImGui::IsItemActive();
    const bool just_released = drag.was_active && !active_now;
    drag.was_active = active_now;
    if (!just_released) {
      return false;
    }
    if (drag.working[0] == slot[0] && drag.working[1] == slot[1] && drag.working[2] == slot[2]) {
      return false;
    }
    slot[0] = drag.working[0];
    slot[1] = drag.working[1];
    slot[2] = drag.working[2];
    return true;
  };
  return entry;
}

// A combo over a fixed name table, editing an index. `names[index]` is what the user sees; the
// serialized value may be something else entirely (sim_resolution stores 1024 while the control
// edits index 1) — holding that translation HERE is why entries have to be functions of the state
// rather than a static value table.
FieldEditorEntry ComboField(int* (*access)(GuiState&), const char* const* names, int count,
                            ApplicabilityFn applicable = AlwaysApplies) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kCombo;
  entry.Constraint = [applicable](const GuiState& state) {
    const Applicability a = applicable(state);
    return FieldEditorConstraint{ a.enabled, a.disabled_reason, false, 0.0, 0.0 };
  };
  entry.Render = [access, names, count](GuiState& state, const char* id_base) {
    int* slot = access(state);
    int working = *slot;
    ImGui::SetNextItemWidth(-FLT_MIN);
    // Required for every combo this panel hosts — the modal can be detached into its own OS
    // viewport, and a popup that is not marked TopMost then renders behind it (see panels.cpp).
    SetNextComboPopupTopMost();
    if (ImGui::Combo(HiddenLabel(id_base).c_str(), &working, names, count) && working != *slot) {
      *slot = working;
      return true;
    }
    return false;
  };
  return entry;
}

// ------------------------------------------------------------------------------------------------
// Shared applicability gates. Each is written once here and read by every field that shares it —
// and, through ConstraintFor, by that field's main-UI BeginDisabled(...) as well.
// ------------------------------------------------------------------------------------------------

Applicability NotUnderFullSky(const GuiState& state) {
  if (LensIsFullSky(state.renderer.lens_type)) {
    return { false, "A full-sky lens shows every direction at once, so this view angle does not apply." };
  }
  return {};
}

Applicability NotUnderFullSkyOrGlobe(const GuiState& state) {
  const Applicability full_sky = NotUnderFullSky(state);
  if (!full_sky.enabled) {
    return full_sky;
  }
  if (state.renderer.lens_type == kLensTypeGlobe) {
    return { false, "The Globe lens orbits the observer around the sphere; roll is locked to 0." };
  }
  return {};
}

Applicability WhenBackgroundLoaded(const GuiState&) {
  if (!g_preview.HasBackground()) {
    return { false, "No background image is loaded." };
  }
  return {};
}

Applicability WhenBackgroundShown(const GuiState& state) {
  const Applicability loaded = WhenBackgroundLoaded(state);
  if (!loaded.enabled) {
    return loaded;
  }
  if (!state.bg_show) {
    return { false, "The background image is hidden." };
  }
  return {};
}

// ------------------------------------------------------------------------------------------------
// Entries that need their own control rather than one of the factories above.
// ------------------------------------------------------------------------------------------------

// The lens combo, in kLensTypePresentationOrder (the enum order is NOT the display order, so
// ImGui::Combo over kLensTypeNames would list them wrong) and with the same side effects the main
// UI's combo applies on a switch — resetting fov to the new lens' default and mirroring az/el
// across the globe boundary. Those live in ApplyLensTypeSelection so both call sites run the SAME
// transition rather than two copies that agree today.
FieldEditorEntry LensTypeField() {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kCombo;
  entry.Constraint = [](const GuiState&) { return FieldEditorConstraint{}; };
  entry.Render = [](GuiState& state, const char* id_base) {
    bool changed = false;
    RenderConfig& r = state.renderer;
    ImGui::SetNextItemWidth(-FLT_MIN);
    SetNextComboPopupTopMost();
    if (ImGui::BeginCombo(HiddenLabel(id_base).c_str(), kLensTypeNames[r.lens_type])) {
      for (const int idx : kLensTypePresentationOrder) {
        const bool selected = (r.lens_type == idx);
        if (ImGui::Selectable(kLensTypeNames[idx], selected) && r.lens_type != idx) {
          ApplyLensTypeSelection(r, idx);
          changed = true;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
      }
      ImGui::EndCombo();
    }
    return changed;
  };
  return entry;
}

// The simulation grid. AC3's named special case: the document stores the VALUE (1024) while the
// control edits an INDEX into kSimResolutions. Registered as editable rather than dodged — a
// registry that could not express "serialized value != bound value" would have to leave every such
// field read-only, and this is the field that proves it can.
FieldEditorEntry SimResolutionField() {
  // Labels come from gui_state.hpp, shared with the View panel's own combo — a second copy here
  // would be a second answer to "what does index 1 mean".
  return ComboField([](GuiState& state) { return &state.renderer.sim_resolution_index; }, kSimResolutionLabels,
                    kSimResolutionCount);
}

// The preset illuminants ONLY. The main UI's combo carries a seventh item ("Custom...") that opens
// a modal editor, and this panel is itself a modal — stacking a second one is the interaction this
// deliberately avoids. A state already using a custom spectrum therefore reads as "registered but
// not applicable" (greyed, with the reason on hover) rather than being silently downgraded to
// whichever preset a 6-item combo would land on.
FieldEditorEntry SpectrumField() {
  return ComboField([](GuiState& state) { return &state.sun.spectrum_index; }, kSpectrumNames, kSpectrumCount,
                    [](const GuiState& state) -> Applicability {
                      if (state.sun.spectrum_index == kCustomSpectrumIndex) {
                        return { false, "A custom spectrum is edited from the Sun panel's spectrum editor." };
                      }
                      return {};
                    });
}

// Aspect preset. Same item set and same "Match Background needs a background" rule as the main UI's
// combo (app_panels.cpp).
//
// What is deliberately NOT reproduced: the main UI also calls ApplyAspectRatio to resize the OS
// window. There is no window handle to reach from here — and the asymmetry is pre-existing rather
// than introduced: opening a document that carries an aspect preset moves the field without
// resizing the window either. The value becomes the default correctly, which is this panel's job.
FieldEditorEntry AspectPresetField() {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kCombo;
  entry.Constraint = [](const GuiState&) { return FieldEditorConstraint{}; };
  entry.Render = [](GuiState& state, const char* id_base) {
    bool changed = false;
    const int current = static_cast<int>(state.aspect_preset);
    ImGui::SetNextItemWidth(-FLT_MIN);
    SetNextComboPopupTopMost();
    if (ImGui::BeginCombo(HiddenLabel(id_base).c_str(), kAspectPresetNames[current])) {
      for (int i = 0; i < kAspectPresetCount; i++) {
        const bool needs_background = (static_cast<AspectPreset>(i) == AspectPreset::kMatchBg);
        const bool disabled = needs_background && !g_preview.HasBackground();
        if (disabled) {
          ImGui::BeginDisabled();
        }
        const bool selected = (i == current);
        if (ImGui::Selectable(kAspectPresetNames[i], selected) && i != current) {
          state.aspect_preset = static_cast<AspectPreset>(i);
          changed = true;
        }
        if (selected) {
          ImGui::SetItemDefaultFocus();
        }
        if (disabled) {
          ImGui::EndDisabled();
        }
      }
      ImGui::EndCombo();
    }
    return changed;
  };
  return entry;
}

// ------------------------------------------------------------------------------------------------
// THE REGISTRY.
//
// Ordered as the serialized document is, so a reader can hold it against SerializeGuiStateJson and
// see at a glance which leaves have an editor. Two leaves deliberately have NO entry:
//
//   overlay_sun_circle_angles — a variable-length list. Its main UI is a whole add/remove popup
//                               with a hard cap, not a single control; there is nothing to place in
//                               a cell that would not be a worse version of it.
//   bg_path                   — a filesystem path. The main UI has no text control for it either
//                               (it is set by a file dialog), and a free-text path in a table is a
//                               way to store a path that does not exist.
// Both stay visible as read-only rows: the panel's row set is generated from the document, so a
// field with no editor is still SHOWN — the user can see it, see what it holds, and (un)check it.
// ------------------------------------------------------------------------------------------------
const std::unordered_map<std::string, FieldEditorEntry>& Registry() {
  static const std::unordered_map<std::string, FieldEditorEntry> kRegistry = [] {
    std::unordered_map<std::string, FieldEditorEntry> map;

    // ---- sun ----
    map.emplace("sun.altitude",
                FloatField([](GuiState& s) { return &s.sun.altitude; }, FixedDomain(-90.0f, 90.0f), "%.1f"));
    // The one row in this registry the pairing gate covers, and it is here on evidence rather than
    // on principle: swept at the measured 179 px slider width, this domain under "%.1f" froze for 4
    // pixels of drag -- the same order as the axis spread sliders, and nobody had noticed. Band and
    // format are named once and handed to both the gate and the field, so the assertion cannot
    // drift away from what renders.
    //
    // "%.2f" and not the "%.3g" its axis siblings took: this slider is kLinear, so it quantizes by
    // a constant AMOUNT per pixel and an absolute format is the family that matches it -- putting a
    // relative format on it would be the same mismatch as the defect, in the other direction. The
    // rest of this registry is out of the gate's reach for a structural reason, not an oversight:
    // most of its domains are FloatDomainFn closures over live GuiState (renderer.fov and friends),
    // so their bounds are not constant expressions and no static_assert can read them.
    {
      constexpr float kSunDiameterMin = 0.1f;
      constexpr float kSunDiameterMax = 5.0f;
      constexpr const char* kSunDiameterFmt = "%.2f";
      static_assert(
          slider_format::FormatIsFineEnough(kSunDiameterFmt, SliderScale::kLinear, kSunDiameterMin, kSunDiameterMax),
          "the sun diameter slider's format is coarser than its linear mapping resolves");
      map.emplace("sun.diameter", FloatField([](GuiState& s) { return &s.sun.diameter; },
                                             FixedDomain(kSunDiameterMin, kSunDiameterMax), kSunDiameterFmt));
    }
    map.emplace("sun.spectrum", SpectrumField());

    // ---- sim ----
    map.emplace("sim.ray_num_millions", FloatField([](GuiState& s) { return &s.sim.ray_num_millions; },
                                                   FixedDomain(0.1f, 100.0f), "%.1f", SliderScale::kLinear,
                                                   [](const GuiState& s) -> Applicability {
                                                     if (s.sim.infinite) {
                                                       return { false,
                                                                "Infinite rays is on, so no ray total applies." };
                                                     }
                                                     return {};
                                                   }));
    map.emplace("sim.max_hits", IntField([](GuiState& s) { return &s.sim.max_hits; }, 1, 64));
    map.emplace("sim.infinite", BoolField([](GuiState& s) { return &s.sim.infinite; }));

    // ---- renderer ----
    map.emplace("renderer.lens_type", LensTypeField());
    // The one state-dependent domain in the registry, and the reason entries are functions: the
    // upper bound is the lens' own maximum field of view.
    map.emplace("renderer.fov", FloatField([](GuiState& s) { return &s.renderer.fov; },
                                           [](const GuiState& s) {
                                             return std::pair<float, float>(
                                                 1.0f,
                                                 LUMICE_MaxFov(static_cast<LUMICE_LensType>(s.renderer.lens_type)));
                                           },
                                           "%.0f", SliderScale::kLinear, NotUnderFullSky));
    map.emplace("renderer.elevation", FloatField([](GuiState& s) { return &s.renderer.elevation; },
                                                 [](const GuiState& s) {
                                                   // Globe clamps one degree short of the pole, where the view matrix
                                                   // degenerates.
                                                   const float limit =
                                                       (s.renderer.lens_type == kLensTypeGlobe) ? 89.0f : 90.0f;
                                                   return std::pair<float, float>(-limit, limit);
                                                 },
                                                 "%.2f", SliderScale::kLinear, NotUnderFullSky));
    map.emplace("renderer.azimuth",
                FloatField([](GuiState& s) { return &s.renderer.azimuth; }, FixedDomain(-180.0f, 180.0f), "%.2f",
                           SliderScale::kLinear, NotUnderFullSky));
    map.emplace("renderer.roll", FloatField([](GuiState& s) { return &s.renderer.roll; }, FixedDomain(-180.0f, 180.0f),
                                            "%.2f", SliderScale::kLinear, NotUnderFullSkyOrGlobe));
    map.emplace("renderer.sim_resolution", SimResolutionField());
    map.emplace("renderer.visible", ComboField([](GuiState& s) { return &s.renderer.visible; }, kVisibleNames,
                                               kVisibleCount, NotUnderFullSky));
    map.emplace("renderer.front", BoolField([](GuiState& s) { return &s.renderer.front; }, NotUnderFullSkyOrGlobe));
    // `background` now HAS a main-UI control (Display > Rendering, beside EV), so this row is its
    // second editor — the one the defaults panel needs in order to edit a personal default without
    // a document open.
    map.emplace("renderer.background", ColorField([](GuiState& s) { return s.renderer.background; }));
    // No `renderer.ray_color` row: the GUI has no tint control anywhere (owner-decided — see
    // GuiState::RenderConfig::ray_color's own comment), so there is nothing here to register an
    // editor for.
    // Domain widened from the historical [-6, +6] because absolute exposure mode removed the
    // auto half of the budget. Under relative mode the reachable exposure is manual EV stacked on
    // top of `ev_auto`, so +-6 of manual travel sat on a moving anchor; under absolute mode the
    // anchor is the emitted energy and the manual slider is the ONLY term, so it has to span the
    // whole spread of scenes on its own. The spread was measured, not guessed: replaying the
    // 25-scene gold corpus under the absolute denominator, the manual EV that brings each scene to
    // the same displayed brightness lands in [-7.75, +5.48], and a sparse 77halo-class scene
    // folded onto the same calibration wants +14.67. [-8, +16] covers both with a little headroom
    // on the bright end; anything narrower than [-8, +6] clips corpus scenes outright.
    //
    // Unrelated to `ev_anchor.hpp`'s own std::clamp(ev, -6, 6) inside ComputeEvAuto — that bounds
    // the auto anchor's internal output, this bounds the manual slider's UI domain. The two
    // sharing the number 6 historically was a coincidence, not a coupling.
    map.emplace("renderer.exposure_offset",
                FloatField([](GuiState& s) { return &s.renderer.exposure_offset; }, FixedDomain(-8.0f, 16.0f), "%.1f"));
    map.emplace("renderer.ev_mode",
                ComboField([](GuiState& s) { return &s.renderer.ev_mode; }, kEvModeNames, kEvModeCount));

    // ---- aspect ratio ----
    map.emplace("aspect_ratio", AspectPresetField());
    map.emplace("aspect_portrait",
                BoolField([](GuiState& s) { return &s.aspect_portrait; },
                          [](const GuiState& s) -> Applicability {
                            if (s.aspect_preset == AspectPreset::kFree || s.aspect_preset == AspectPreset::k1x1 ||
                                s.aspect_preset == AspectPreset::kMatchBg) {
                              return { false, "This aspect preset has no portrait/landscape orientation." };
                            }
                            return {};
                          }));

    // ---- background image ----
    map.emplace("bg_show", BoolField([](GuiState& s) { return &s.bg_show; }, WhenBackgroundLoaded));
    map.emplace("bg_alpha", FloatField([](GuiState& s) { return &s.bg_alpha; }, FixedDomain(0.0f, 1.0f), "%.2f",
                                       SliderScale::kLinear, WhenBackgroundShown));
    // Pan is measured in texture widths/heights (see ComputeBgUvTransform), so +-2 lets the user
    // push the image two full frames off-centre in either direction — comfortably past any real
    // crop, while still short enough that the slider's travel is usable at photo-alignment
    // precision. Zoom is multiplicative, hence kLog: a drag from 1.0 to 2.0 should cover the same
    // slider distance as one from 0.5 to 1.0, and kLog additionally requires min_val > 0.
    // All three share bg_alpha's WhenBackgroundShown gate, so "can I move the background" answers
    // the same in the panel and on the canvas.
    map.emplace("bg_offset_x", FloatField([](GuiState& s) { return &s.bg_offset_x; }, FixedDomain(-2.0f, 2.0f), "%.3f",
                                          SliderScale::kLinear, WhenBackgroundShown));
    map.emplace("bg_offset_y", FloatField([](GuiState& s) { return &s.bg_offset_y; }, FixedDomain(-2.0f, 2.0f), "%.3f",
                                          SliderScale::kLinear, WhenBackgroundShown));
    map.emplace("bg_scale", FloatField([](GuiState& s) { return &s.bg_scale; }, FixedDomain(0.2f, 5.0f), "%.2f",
                                       SliderScale::kLog, WhenBackgroundShown));

    // ---- auxiliary line overlay ----
    map.emplace("overlay_horizon_line", BoolField([](GuiState& s) { return &s.show_horizon_line; }));
    map.emplace("overlay_horizon_label", BoolField([](GuiState& s) { return &s.show_horizon_label; }));
    map.emplace("overlay_grid_line", BoolField([](GuiState& s) { return &s.show_grid_line; }));
    map.emplace("overlay_grid_label", BoolField([](GuiState& s) { return &s.show_grid_label; }));
    map.emplace("overlay_sun_circles_line", BoolField([](GuiState& s) { return &s.show_sun_circles_line; }));
    map.emplace("overlay_sun_circles_label", BoolField([](GuiState& s) { return &s.show_sun_circles_label; }));
    map.emplace("overlay_horizon_color", ColorField([](GuiState& s) { return s.horizon_color; }));
    map.emplace("overlay_grid_color", ColorField([](GuiState& s) { return s.grid_color; }));
    map.emplace("overlay_sun_circles_color", ColorField([](GuiState& s) { return s.sun_circles_color; }));
    map.emplace("overlay_horizon_alpha",
                FloatField([](GuiState& s) { return &s.horizon_alpha; }, FixedDomain(0.0f, 1.0f), "%.2f"));
    map.emplace("overlay_grid_alpha",
                FloatField([](GuiState& s) { return &s.grid_alpha; }, FixedDomain(0.0f, 1.0f), "%.2f"));
    map.emplace("overlay_sun_circles_alpha",
                FloatField([](GuiState& s) { return &s.sun_circles_alpha; }, FixedDomain(0.0f, 1.0f), "%.2f"));
    map.emplace("overlay_zenith_nadir_line", BoolField([](GuiState& s) { return &s.show_zenith_nadir_line; }));
    map.emplace("overlay_zenith_nadir_color", ColorField([](GuiState& s) { return s.zenith_nadir_color; }));
    map.emplace("overlay_zenith_nadir_alpha",
                FloatField([](GuiState& s) { return &s.zenith_nadir_alpha; }, FixedDomain(0.0f, 1.0f), "%.2f"));
    map.emplace("overlay_zenith_nadir_radius_px",
                FloatField([](GuiState& s) { return &s.zenith_nadir_radius_px; }, FixedDomain(2.0f, 20.0f), "%.1f px"));
    map.emplace("overlay_lens_border_line", BoolField([](GuiState& s) { return &s.show_lens_border_line; }));
    map.emplace("overlay_lens_border_color", ColorField([](GuiState& s) { return s.lens_border_color; }));
    map.emplace("overlay_lens_border_alpha",
                FloatField([](GuiState& s) { return &s.lens_border_alpha; }, FixedDomain(0.0f, 1.0f), "%.2f"));

    // ---- panel state ----
    map.emplace("right_panel_collapsed", BoolField([](GuiState& s) { return &s.right_panel_collapsed; }));
    map.emplace("modal_layout_vertical", BoolField([](GuiState& s) { return &s.modal_layout_vertical; }));

    return map;
  }();
  return kRegistry;
}

}  // namespace

const FieldEditorEntry* FindFieldEditor(const std::string& key_path) {
  const auto& registry = Registry();
  const auto it = registry.find(key_path);
  return it != registry.end() ? &it->second : nullptr;
}

FieldEditorConstraint ConstraintFor(const std::string& key_path, const GuiState& state) {
  const FieldEditorEntry* entry = FindFieldEditor(key_path);
  if (entry == nullptr) {
    // Not an assert: the gate has to hold in the release build the GUI actually ships, where NDEBUG
    // would strip one and leave a null dereference with no diagnostic in its place.
    FatalAbort("field editor registry has no entry for key path \"%s\"", key_path.c_str());
  }
  return entry->Constraint(state);
}

std::vector<std::string> RegisteredFieldEditorKeyPaths() {
  const auto& registry = Registry();
  std::vector<std::string> keys;
  keys.reserve(registry.size());
  for (const auto& [key, entry] : registry) {
    keys.push_back(key);
  }
  std::sort(keys.begin(), keys.end());
  return keys;
}

}  // namespace lumice::gui
