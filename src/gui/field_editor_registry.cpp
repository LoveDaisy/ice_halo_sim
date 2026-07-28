#include "gui/field_editor_registry.hpp"

#include <algorithm>
#include <unordered_map>
#include <utility>

#include "gui/app.hpp"
#include "gui/gui_constants.hpp"
#include "gui/panels.hpp"
#include "imgui.h"
#include "include/lumice.h"

namespace lumice::gui {

namespace {

// ------------------------------------------------------------------------------------------------
// Entry factories.
//
// Each factory takes the field's domain ONCE and hands the same closure to both halves of the
// entry, so the bound the range check uses and the bound the widget clamps to cannot disagree. That
// is the only duplication this file actively prevents; the duplication against the MAIN UI's call
// site is the acknowledged debt described in the header.
// ------------------------------------------------------------------------------------------------

// Whether the field applies at all in the current configuration, and what to tell the user when it
// does not. Mirrors the BeginDisabled(...) expression at the field's main-UI call site.
struct Applicability {
  bool enabled = true;
  const char* reason = nullptr;
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

FieldEditorEntry FloatField(float* (*access)(GuiState&), FloatDomainFn domain, const char* fmt,
                            SliderScale scale = SliderScale::kLinear, ApplicabilityFn applicable = AlwaysApplies) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kFloatSlider;
  entry.Constraint = [domain, applicable](const GuiState& state) {
    const auto range = domain(state);
    const Applicability a = applicable(state);
    return FieldEditorConstraint{ a.enabled, a.reason, true, range.first, range.second };
  };
  entry.Render = [access, domain, fmt, scale](GuiState& state, const char* id_base) {
    const auto range = domain(state);
    float* slot = access(state);
    float working = *slot;
    SliderWithInput(id_base, &working, range.first, range.second, fmt, scale, /*trailing_label=*/false);
    return CommitIfEdited(*slot, working, range.first, range.second);
  };
  return entry;
}

FieldEditorEntry IntField(int* (*access)(GuiState&), int min_value, int max_value,
                          ApplicabilityFn applicable = AlwaysApplies) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kIntSlider;
  entry.Constraint = [min_value, max_value, applicable](const GuiState& state) {
    const Applicability a = applicable(state);
    return FieldEditorConstraint{ a.enabled, a.reason, true, static_cast<double>(min_value),
                                  static_cast<double>(max_value) };
  };
  entry.Render = [access, min_value, max_value](GuiState& state, const char* id_base) {
    int* slot = access(state);
    int working = *slot;
    SliderIntWithInput(id_base, &working, min_value, max_value, /*trailing_label=*/false);
    return CommitIfEdited(*slot, working, min_value, max_value);
  };
  return entry;
}

FieldEditorEntry BoolField(bool* (*access)(GuiState&), ApplicabilityFn applicable = AlwaysApplies) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kCheckbox;
  entry.Constraint = [applicable](const GuiState& state) {
    const Applicability a = applicable(state);
    return FieldEditorConstraint{ a.enabled, a.reason, false, 0.0, 0.0 };
  };
  entry.Render = [access](GuiState& state, const char* id_base) {
    return ImGui::Checkbox(HiddenLabel(id_base).c_str(), access(state));
  };
  return entry;
}

// A colour triple. No numeric domain on purpose: ColorEdit3 has no min/max to restate, which is
// also why the three fields with no main-UI colour control at all (see the registry below) carry no
// risk of inventing a constraint that disagrees with one.
FieldEditorEntry ColorField(float* (*access)(GuiState&)) {
  FieldEditorEntry entry;
  entry.kind = FieldEditorKind::kColor;
  entry.Constraint = [](const GuiState&) { return FieldEditorConstraint{}; };
  entry.Render = [access](GuiState& state, const char* id_base) {
    // NoInputs — the swatch only, exactly the form the main UI's overlay rows use, and the only one
    // that fits a table cell.
    return ImGui::ColorEdit3(HiddenLabel(id_base).c_str(), access(state), ImGuiColorEditFlags_NoInputs);
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
    return FieldEditorConstraint{ a.enabled, a.reason, false, 0.0, 0.0 };
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
// Shared applicability gates. Each is the main-UI BeginDisabled(...) expression, restated once
// here rather than once per field that shares it.
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
  static const char* const kLabels[] = { "512", "1024", "2048", "4096" };
  static_assert(sizeof(kLabels) / sizeof(kLabels[0]) == kSimResolutionCount,
                "resolution labels must cover kSimResolutions");
  return ComboField([](GuiState& state) { return &state.renderer.sim_resolution_index; }, kLabels, kSimResolutionCount);
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
    map.emplace("sun.diameter",
                FloatField([](GuiState& s) { return &s.sun.diameter; }, FixedDomain(0.1f, 5.0f), "%.1f"));
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
    // background / ray_color / opacity have NO control anywhere in the main UI — the table is
    // their first editor rather than a second one. For the two colours that costs nothing (a
    // colour has no domain to disagree about). `opacity`'s [0,1] IS a new statement, made here
    // because the value is a multiplier the renderer applies directly; nothing else in the repo
    // constrains it, so this is the definition rather than a copy of one.
    map.emplace("renderer.background", ColorField([](GuiState& s) { return s.renderer.background; }));
    map.emplace("renderer.ray_color", ColorField([](GuiState& s) { return s.renderer.ray_color; }));
    map.emplace("renderer.opacity",
                FloatField([](GuiState& s) { return &s.renderer.opacity; }, FixedDomain(0.0f, 1.0f), "%.2f"));
    map.emplace("renderer.exposure_offset",
                FloatField([](GuiState& s) { return &s.renderer.exposure_offset; }, FixedDomain(-6.0f, 6.0f), "%.1f"));

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
