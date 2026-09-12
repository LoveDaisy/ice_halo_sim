#include "gui/analysis_panel.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <numeric>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "IconsFontAwesome6.h"
#include "gui/annotation_anchors.hpp"
#include "gui/app.hpp"
#include "gui/destructive_style.hpp"
#include "gui/edit_modals.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_constants.hpp"
#include "gui/gui_logger.hpp"
#include "gui/panels.hpp"  // SliderWithInput
#include "gui/ray_num_domain.hpp"
#include "gui/raypath_segments.hpp"
#include "gui/semantic_colors.hpp"
#include "gui/server_poller.hpp"
#include "gui/sim_state_rules.hpp"
#include "gui/symmetry_ui.hpp"
#include "gui/theme.hpp"
#include "imgui.h"
#include "include/lumice.h"
#include "util/result_frame.hpp"

namespace lumice::gui {

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;

// The neighbour offset the ring's local scale is measured over. Small enough to be local on
// every lens the GUI draws, large enough that the angle between the two directions is well above
// float noise at the narrowest FOV.
constexpr int kRingScaleProbePx = 8;
constexpr float kRoiRingThicknessPt = 2.0f;
constexpr int kRoiRingSegments = 96;
constexpr float kRoiMarkerDotRadiusPt = 3.0f;
// Two unit directions with a dot product at or above this are "the same centre" for the moved-
// since-the-result hint: 1e-6 in the dot is ~0.08 degrees, an order of magnitude past the float
// noise of one LUMICE_UnprojectPixel round trip and far under anything a drag produces.
constexpr float kConeCenterSameDirDot = 1.0f - 1e-6f;

// The Point-mode centre as the user reads it: the altitude and azimuth of the direction light
// comes FROM, i.e. of the sky point clicked, from a direction light TRAVELS (altitude = asin(-z),
// azimuth measured as the sun's is, so the sun at azimuth 0 sits at lon 180 — the same formula
// the annotation sun direction uses, inverted).
void DirToAltAz(const float dir[3], float* alt_deg, float* az_deg) {
  const float z = std::max(-1.0f, std::min(1.0f, dir[2]));
  *alt_deg = std::asin(-z) * kRad2Deg;
  float az = std::atan2(dir[1], dir[0]) * kRad2Deg - 180.0f;
  while (az > 180.0f) {
    az -= 360.0f;
  }
  while (az < -180.0f) {
    az += 360.0f;
  }
  *az_deg = az;
}

// "P|B|D", "P|B", ..., or "no symmetry": the bits as the user reads them.
std::string SymmetryBitsLabel(uint8_t bits) {
  std::string out;
  for (const auto& [bit, name] : { std::pair<int, const char*>{ LUMICE_RAYPATH_SYMMETRY_P, "P" },
                                   std::pair<int, const char*>{ LUMICE_RAYPATH_SYMMETRY_B, "B" },
                                   std::pair<int, const char*>{ LUMICE_RAYPATH_SYMMETRY_D, "D" } }) {
    if (bits & bit) {
      if (!out.empty()) {
        out += '|';
      }
      out += name;
    }
  }
  return out.empty() ? "no symmetry" : out;
}

const char* RoiModeLabel(int mode) {
  switch (mode) {
    case LUMICE_RAYPATH_ROI_IN_FRAME:
      return "in frame";
    case LUMICE_RAYPATH_ROI_CONE:
      return "point";
    default:
      return "whole sky";
  }
}

}  // namespace

// ---- Lifecycle -----------------------------------------------------------------------------------

bool DeriveAnalysisInProgress(bool started, const PreviewSnapshot* snap) {
  if (!started) {
    return false;
  }
  if (snap == nullptr || !snap->valid) {
    return true;  // submitted; the wake edge has not been observed yet
  }
  return snap->lifecycle == LUMICE_LIFECYCLE_RUNNING;
}

// ---- Result adoption and the display-time projection ----------------------------------------------

bool AdoptAnalysisPayloadIfNew(GuiState& state, const std::shared_ptr<const AnalysisPayload>& payload) {
  if (!payload) {
    return false;
  }
  const unsigned long long held =
      state.analysis_result.payload ? state.analysis_result.payload->snapshot_generation : 0ULL;
  if (payload->snapshot_generation <= held) {
    return false;
  }
  state.analysis_result.payload = payload;
  state.analysis.selected_entry.reset();
  RecomputeAnalysisDisplayOrder(state);
  return true;
}

int RingsWithinRadius(float radius_deg, float cone_radius_deg, int ring_count) {
  if (ring_count <= 0) {
    return 0;
  }
  if (!(cone_radius_deg > 0.0f) || !(radius_deg > 0.0f)) {
    return 1;
  }
  // k rings cover k * (cone / ring_count) degrees; the smallest k reaching radius_deg. A tiny
  // epsilon keeps "exactly on a ring boundary" on the ring it names rather than one past it.
  const float per_ring = cone_radius_deg / static_cast<float>(ring_count);
  const int k = static_cast<int>(std::ceil(radius_deg / per_ring - 1e-4f));
  return std::max(1, std::min(ring_count, k));
}

double SumRingEnergy(const LUMICE_RaypathHistogramEntry& entry, int rings) {
  const int n = std::max(0, std::min({ rings, entry.ring_count, LUMICE_MAX_RAYPATH_CONE_RINGS }));
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += entry.ring_energy[i];
  }
  return sum;
}

void RecomputeAnalysisDisplayOrder(GuiState& state) {
  auto& view = state.analysis_result;
  view.display_energy.clear();
  view.display_order.clear();
  view.display_cumulative_pct.clear();
  view.display_total = 0.0;
  view.display_ring_count = 0;
  if (!view.payload) {
    return;
  }
  const auto& entries = view.payload->entries;
  const bool cone = view.payload->roi_mode == LUMICE_RAYPATH_ROI_CONE && view.payload->cone_ring_count > 0;
  if (cone) {
    view.display_ring_count = RingsWithinRadius(
        state.analysis.cone_radius_deg, view.payload->cone_radius_rad * kRad2Deg, view.payload->cone_ring_count);
  }
  view.display_energy.reserve(entries.size());
  for (const auto& e : entries) {
    view.display_energy.push_back(cone ? SumRingEnergy(e, view.display_ring_count) : e.energy);
    view.display_total += view.display_energy.back();
  }
  view.display_order.resize(entries.size());
  std::iota(view.display_order.begin(), view.display_order.end(), 0);
  // Descending by the displayed energy; ties keep the C API's order (energy descending, display
  // ascending), so the list is deterministic for equal sums too.
  std::stable_sort(view.display_order.begin(), view.display_order.end(),
                   [&](int a, int b) { return view.display_energy[a] > view.display_energy[b]; });
  // The other bucket is in the denominator whole — see AnalysisResultView::display_total — and the
  // cumulative column runs down the sorted rows, so it is monotone by construction and its last
  // value plus the "other" line's share is 100.
  view.display_total += view.payload->other_energy;
  view.display_cumulative_pct.reserve(view.display_order.size());
  double running = 0.0;
  for (const int idx : view.display_order) {
    running += view.display_energy[static_cast<size_t>(idx)];
    view.display_cumulative_pct.push_back(view.display_total > 0.0 ? running / view.display_total * 100.0 : 0.0);
  }
}

double AnalysisOtherPct(const GuiState& state) {
  const auto& view = state.analysis_result;
  if (!view.payload || !(view.display_total > 0.0)) {
    return 0.0;
  }
  return view.payload->other_energy / view.display_total * 100.0;
}

// ---- The symmetry, and the read of the entries under it ------------------------------------------

uint8_t AnalysisSymmetryBits(const GuiState& state) {
  const auto& a = state.analysis;
  return static_cast<uint8_t>((a.symmetry_p ? LUMICE_RAYPATH_SYMMETRY_P : 0) |
                              (a.symmetry_b ? LUMICE_RAYPATH_SYMMETRY_B : 0) |
                              (a.symmetry_d ? LUMICE_RAYPATH_SYMMETRY_D : 0));
}

bool AnalysisEntriesNeedRefresh(const GuiState& state) {
  const auto& payload = state.analysis_result.payload;
  if (!payload) {
    return false;
  }
  const auto& a = state.analysis;
  return !(a.fetched_once && a.fetched_generation == payload->snapshot_generation &&
           a.fetched_symmetry == AnalysisSymmetryBits(state));
}

bool RefreshAnalysisEntries(GuiState& state, LUMICE_Server* server) {
  if (server == nullptr || !AnalysisEntriesNeedRefresh(state)) {
    return false;
  }
  auto& a = state.analysis;
  const uint8_t symmetry = AnalysisSymmetryBits(state);
  // Recorded before the read, whatever it finds: a frame that cannot be read now will not be
  // readable next frame either, and the record is what keeps this from being a per-frame poll.
  a.fetched_once = true;
  a.fetched_generation = state.analysis_result.payload->snapshot_generation;
  a.fetched_symmetry = symmetry;

  LUMICE_ResultFrame* raw_frame = nullptr;
  if (LUMICE_AcquireResultFrame(server, &raw_frame) != LUMICE_OK || raw_frame == nullptr) {
    return false;
  }
  lumice::ResultFramePtr frame(raw_frame);
  LUMICE_RaypathAnalysisInfo info{};
  if (LUMICE_FrameGetRaypathAnalysisInfo(frame.get(), symmetry, &info) != LUMICE_OK || info.present == 0) {
    GUI_LOG_INFO("[Analysis] the server holds no analysis frame; the list keeps the entries on hand (symmetry {})",
                 static_cast<int>(state.analysis_result.entries_symmetry));
    return false;
  }
  auto payload = std::make_shared<AnalysisPayload>();
  payload->snapshot_generation = info.snapshot_generation;
  payload->roi_mode = info.roi_mode;
  payload->cone_ring_count = info.cone_ring_count;
  payload->cone_radius_rad = info.cone_radius_rad;
  payload->other_energy = info.other_energy;
  payload->other_count = info.other_count;
  payload->truncated_chain_count = info.truncated_chain_count;
  payload->max_row_error = info.max_row_error;
  // One more slot than entries: the sentinel (count == 0) lands at [entry_count] when the frame
  // holds exactly entry_count entries, and the read below stops at it in every case.
  std::vector<LUMICE_RaypathHistogramEntry> raw(static_cast<size_t>(std::max(info.entry_count, 0)) + 1);
  if (LUMICE_FrameGetRaypathAnalysis(frame.get(), symmetry, raw.data(), info.entry_count) != LUMICE_OK) {
    return false;
  }
  size_t n = 0;
  while (n < raw.size() && raw[n].count != 0) {
    ++n;
  }
  raw.resize(n);
  payload->entries = std::move(raw);
  // The frame may be a newer snapshot than the payload the poller published; what is shown is
  // this frame, so the record follows it — and AdoptAnalysisPayloadIfNew, seeing the same
  // generation from the poller later, will not clear the selection for a result already on show.
  a.fetched_generation = info.snapshot_generation;
  state.analysis_result.payload = std::move(payload);
  state.analysis_result.entries_symmetry = symmetry;
  RecomputeAnalysisDisplayOrder(state);
  GUI_LOG_VERBOSE("[Analysis] entries read under symmetry {}: {} rows, gen={}", static_cast<int>(symmetry),
                  state.analysis_result.payload->entries.size(), info.snapshot_generation);
  return true;
}

const LUMICE_RaypathHistogramEntry* SelectedAnalysisEntry(const GuiState& state) {
  const auto& sel = state.analysis.selected_entry;
  const auto& payload = state.analysis_result.payload;
  if (!sel.has_value() || !payload) {
    return nullptr;
  }
  for (const auto& e : payload->entries) {
    if (*sel == e.display) {
      return &e;
    }
  }
  return nullptr;
}

// ---- ROI: the click on the preview ---------------------------------------------------------------

std::optional<CanvasPixel> PreviewPointToCanvasPixel(float rel_x_pt, float rel_y_pt, float dpi_scale_x,
                                                     float dpi_scale_y, int vp_w, int vp_h) {
  const float fx = rel_x_pt * dpi_scale_x;
  const float fy = rel_y_pt * dpi_scale_y;
  if (!(fx >= 0.0f) || !(fy >= 0.0f)) {
    return std::nullopt;
  }
  const int px = static_cast<int>(std::floor(fx));
  const int py = static_cast<int>(std::floor(fy));
  if (px >= vp_w || py >= vp_h) {
    return std::nullopt;
  }
  return CanvasPixel{ px, py };
}

void CanvasPixelToPreviewPoint(int px, int py, float dpi_scale_x, float dpi_scale_y, float* out_x_pt, float* out_y_pt) {
  *out_x_pt = (static_cast<float>(px) + 0.5f) / dpi_scale_x;
  *out_y_pt = (static_cast<float>(py) + 0.5f) / dpi_scale_y;
}

LUMICE_AnnotationView PreviewAnnotationView(const GuiState& state, int canvas_w, int canvas_h) {
  return BuildAnnotationView(MakeAnnotationViewKey(AnnotationViewInputFor(state, state.renderer), canvas_w, canvas_h));
}

bool PickAnalysisConeCenter(GuiState& state, const LUMICE_AnnotationView& view, int px, int py) {
  float dir[3] = { 0.0f, 0.0f, 0.0f };
  int valid = 0;
  const LUMICE_ErrorCode err = LUMICE_UnprojectPixel(&view, px, py, dir, &valid);
  if (err != LUMICE_OK) {
    GUI_LOG_WARNING("[Analysis] LUMICE_UnprojectPixel failed: {} (pixel {},{} on {}x{})", static_cast<int>(err), px, py,
                    view.width, view.height);
    return false;
  }
  if (valid == 0) {
    GUI_LOG_INFO("[Analysis] pick at ({},{}) is not on the sky; still armed — click on the picture", px, py);
    return false;
  }
  auto& a = state.analysis;
  std::copy(dir, dir + 3, a.cone_center_dir);
  a.cone_center_valid = true;
  a.pick_armed = false;
  float alt = 0.0f;
  float az = 0.0f;
  DirToAltAz(dir, &alt, &az);
  GUI_LOG_INFO("[Analysis] cone centre picked at pixel ({},{}): altitude {:.2f} deg, azimuth {:.2f} deg", px, py, alt,
               az);
  return true;
}

std::optional<CanvasPixel> ProjectConeCenterMarker(const GuiState& state, const LUMICE_AnnotationView& view) {
  const auto& a = state.analysis;
  if (!a.cone_center_valid) {
    return std::nullopt;
  }
  float fx = 0.0f;
  float fy = 0.0f;
  int valid = 0;
  if (LUMICE_ProjectDirection(&view, a.cone_center_dir, &fx, &fy, &valid) != LUMICE_OK || valid == 0) {
    return std::nullopt;
  }
  // The C API already clamps into [0, w-1] x [0, h-1]; the clamp here only guards the rounding of
  // a value sitting exactly on the far edge from stepping one past it.
  const int px = std::max(0, std::min(view.width - 1, static_cast<int>(std::lround(fx))));
  const int py = std::max(0, std::min(view.height - 1, static_cast<int>(std::lround(fy))));
  return CanvasPixel{ px, py };
}

ConeInputOwner ArbitrateConeInput(bool marker_hover_or_dragging, bool pick_armed) {
  if (marker_hover_or_dragging) {
    return ConeInputOwner::kMarkerDrag;
  }
  if (pick_armed) {
    return ConeInputOwner::kPickClick;
  }
  return ConeInputOwner::kCamera;
}

bool DragAnalysisConeCenter(GuiState& state, const LUMICE_AnnotationView& view, int px, int py) {
  float dir[3] = { 0.0f, 0.0f, 0.0f };
  int valid = 0;
  if (LUMICE_UnprojectPixel(&view, px, py, dir, &valid) != LUMICE_OK || valid == 0) {
    return false;
  }
  std::copy(dir, dir + 3, state.analysis.cone_center_dir);
  return true;
}

bool EnsureDefaultConeCenter(GuiState& state, const LUMICE_AnnotationView& view, int vp_w, int vp_h) {
  float dir[3] = { 0.0f, 0.0f, 0.0f };
  int valid = 0;
  const int px = vp_w / 2;
  const int py = vp_h / 2;
  if (LUMICE_UnprojectPixel(&view, px, py, dir, &valid) != LUMICE_OK || valid == 0) {
    return false;
  }
  auto& a = state.analysis;
  std::copy(dir, dir + 3, a.cone_center_dir);
  a.cone_center_valid = true;
  float alt = 0.0f;
  float az = 0.0f;
  DirToAltAz(dir, &alt, &az);
  GUI_LOG_INFO("[Analysis] cone centre defaulted to the view centre ({},{}): altitude {:.2f} deg, azimuth {:.2f} deg",
               px, py, alt, az);
  return true;
}

bool ConeCenterDriftedFromResult(const GuiState& state) {
  const auto& a = state.analysis;
  const auto& payload = state.analysis_result.payload;
  if (!payload || payload->roi_mode != LUMICE_RAYPATH_ROI_CONE || !a.cone_center_valid) {
    return false;
  }
  const float dot = a.cone_center_dir[0] * a.analyzed_cone_center_dir[0] +
                    a.cone_center_dir[1] * a.analyzed_cone_center_dir[1] +
                    a.cone_center_dir[2] * a.analyzed_cone_center_dir[2];
  return dot < kConeCenterSameDirDot;
}

std::optional<float> ConeRingRadiusCanvasPx(const LUMICE_AnnotationView& view, int px, int py, float radius_deg) {
  float d0[3] = { 0.0f, 0.0f, 0.0f };
  int v0 = 0;
  if (LUMICE_UnprojectPixel(&view, px, py, d0, &v0) != LUMICE_OK || v0 == 0) {
    return std::nullopt;
  }
  // Probe to the right, or to the left when the right neighbour leaves the image (near a rim).
  for (const int step : { kRingScaleProbePx, -kRingScaleProbePx }) {
    float d1[3] = { 0.0f, 0.0f, 0.0f };
    int v1 = 0;
    if (LUMICE_UnprojectPixel(&view, px + step, py, d1, &v1) != LUMICE_OK || v1 == 0) {
      continue;
    }
    const float dot = std::max(-1.0f, std::min(1.0f, d0[0] * d1[0] + d0[1] * d1[1] + d0[2] * d1[2]));
    const float angle_deg = std::acos(dot) * kRad2Deg;
    if (!(angle_deg > 1e-5f)) {
      continue;
    }
    const float deg_per_px = angle_deg / static_cast<float>(std::abs(step));
    return radius_deg / deg_per_px;
  }
  return std::nullopt;
}

// ---- The request ---------------------------------------------------------------------------------

void EnsureDefaultAnalysisRayBudget(GuiState& state) {
  if (state.analysis.ray_budget_initialized) {
    return;
  }
  state.analysis.ray_num_millions = state.sim.ray_num_millions;
  state.analysis.infinite = state.sim.infinite;
  state.analysis.ray_budget_initialized = true;
}

LUMICE_RaypathAnalysisRequest BuildAnalysisRequest(const GuiState& state, int canvas_w, int canvas_h) {
  LUMICE_RaypathAnalysisRequest req{};
  req.roi_mode = state.analysis.roi_mode;
  // The budget applies to every ROI mode, so it sits outside the switch. Millions -> rays in
  // double, as the document's own field is converted, so 12.5 M is 12500000 and not a float
  // rounding of it.
  req.infinite = state.analysis.infinite ? 1 : 0;
  req.ray_num = static_cast<LUMICE_RayCount>(static_cast<double>(state.analysis.ray_num_millions) * 1e6);
  switch (state.analysis.roi_mode) {
    case LUMICE_RAYPATH_ROI_IN_FRAME:
      req.frame_view = PreviewAnnotationView(state, canvas_w, canvas_h);
      break;
    case LUMICE_RAYPATH_ROI_CONE:
      std::copy(state.analysis.cone_center_dir, state.analysis.cone_center_dir + 3, req.cone_center);
      req.cone_radius_rad = kAnalysisConeMaxRadiusDeg * kDeg2Rad;
      req.cone_ring_count = kAnalysisConeRingCount;
      break;
    default:
      break;
  }
  return req;
}

// ---- Exclude this raypath ------------------------------------------------------------------------

namespace {

// The pool slot a scene crystal id came from, through the same map BuildScene commits with.
std::optional<int> PoolSlotForSceneCrystal(const GuiState& state, int scene_crystal_id) {
  const std::map<int, int> pool_to_core = ComputeCrystalPoolToCoreIdMap(state);
  for (const auto& [pool, core] : pool_to_core) {
    if (core == scene_crystal_id) {
      return pool;  // unique: the map is injective by construction (file_io.hpp)
    }
  }
  return std::nullopt;
}

// Every entry (in layer order) that uses crystal pool slot `pool_id`. The histogram counted the
// crystal wherever it was used, so the exclusion has to reach all of them — and they need not
// share one filter_id: the linked-group invariant (gui_state.hpp) is about entries that share
// BOTH ids, and a document may well hold one entry of a crystal with a filter and another without
// (a loaded config, a Duplicate). The non-const overload delegates to the const one so the walk
// exists once.
std::vector<const EntryCard*> EntriesForPoolCrystal(const GuiState& state, int pool_id) {
  std::vector<const EntryCard*> out;
  for (const auto& layer : state.layers) {
    for (const auto& entry : layer.entries) {
      if (entry.crystal_id == pool_id) {
        out.push_back(&entry);
      }
    }
  }
  return out;
}

std::vector<EntryCard*> EntriesForPoolCrystal(GuiState& state, int pool_id) {
  std::vector<EntryCard*> out;
  for (const EntryCard* entry : EntriesForPoolCrystal(static_cast<const GuiState&>(state), pool_id)) {
    out.push_back(const_cast<EntryCard*>(entry));
  }
  return out;
}

// The entry's filter when it has one that the pool holds, else nullptr. The one place the
// (entry -> pool slot) read is bounds-checked: a filter_id the pool does not hold reads as "no
// filter" here and is left alone by the write below (never written through).
const FilterConfig* FilterOfEntry(const GuiState& state, const EntryCard& entry) {
  if (!entry.filter_id.has_value() || *entry.filter_id < 0 ||
      *entry.filter_id >= static_cast<int>(state.filters.size())) {
    return nullptr;
  }
  return &state.filters[static_cast<size_t>(*entry.filter_id)];
}

// What the crystal's entries carry today, read once for the eligibility, the tooltip and the
// apply: the distinct pool slots holding an Out filter (first-appearance order), whether any
// entry holds an In filter, and how many entries hold no filter at all.
struct CrystalFilterCensus {
  std::vector<int> out_slots;
  bool any_in = false;
  int unfiltered = 0;
};

CrystalFilterCensus CensusForPoolCrystal(const GuiState& state, int pool_id) {
  CrystalFilterCensus census;
  for (const EntryCard* entry : EntriesForPoolCrystal(state, pool_id)) {
    const FilterConfig* existing = FilterOfEntry(state, *entry);
    if (existing == nullptr) {
      if (!entry->filter_id.has_value()) {
        ++census.unfiltered;
      }
      // else: entry->filter_id is set but out of bounds (FilterOfEntry's bounds check failed) —
      // an id the pool does not hold cannot be resolved to Out or In, and is not "no filter"
      // either, so it lands in neither bucket. Deliberate: this is not a state the census claims
      // to describe, not an entry the loop forgot.
      continue;
    }
    if (existing->action != 1) {
      census.any_in = true;
    } else if (std::find(census.out_slots.begin(), census.out_slots.end(), *entry->filter_id) ==
               census.out_slots.end()) {
      census.out_slots.push_back(*entry->filter_id);
    }
  }
  return census;
}

// The pool slot of the selected chain's crystal when the selection is a single-segment chain of a
// crystal the document commits; nullopt otherwise. The two checks EvaluateExcludeEligibility
// makes before it looks at filters, for the callers that only need to know where to look.
std::optional<int> SelectedChainPoolSlot(const GuiState& state) {
  const LUMICE_RaypathHistogramEntry* e = SelectedAnalysisEntry(state);
  if (e == nullptr || e->chain_len != 1) {
    return std::nullopt;
  }
  const std::optional<int> pool = PoolSlotForSceneCrystal(state, e->chain[0].crystal_id);
  if (!pool.has_value() || *pool < 0 || *pool >= static_cast<int>(state.crystals.size())) {
    return std::nullopt;
  }
  return pool;
}

std::string Plural(int n, const char* one, const char* many) {
  return std::to_string(n) + " " + (n == 1 ? one : many);
}

}  // namespace

ExcludeEligibility EvaluateExcludeEligibility(const GuiState& state, std::string* why) {
  const LUMICE_RaypathHistogramEntry* e = SelectedAnalysisEntry(state);
  if (e == nullptr) {
    if (why) {
      *why = "Select a raypath in the list first.";
    }
    return ExcludeEligibility::kNoSelection;
  }
  if (e->chain_len != 1) {
    if (why) {
      *why =
          "This chain crosses scattering layers. A filter belongs to one crystal, so the current filter "
          "model cannot express excluding a multi-layer chain.";
    }
    return ExcludeEligibility::kMultiSegment;
  }
  const std::optional<int> pool = SelectedChainPoolSlot(state);
  if (!pool.has_value()) {
    if (why) {
      *why =
          "The crystal this chain went through is not in the current document (the crystal list changed "
          "since the analysis). Run and analyze again.";
    }
    return ExcludeEligibility::kCrystalNotInScene;
  }
  // The filters the crystal's entries carry: an Out filter is extended, so it does not deny; an
  // In filter on any of them does.
  if (CensusForPoolCrystal(state, *pool).any_in) {
    if (why) {
      *why =
          "An entry using this crystal already has an In filter (filter_in). Exclude can only add to an "
          "existing Out filter; edit that filter directly, or change its action to Out, to continue.";
    }
    return ExcludeEligibility::kEntryHasInFilter;
  }
  if (why) {
    why->clear();
  }
  return ExcludeEligibility::kOk;
}

std::string ExcludeAppendNotice(const GuiState& state) {
  const std::optional<int> pool = SelectedChainPoolSlot(state);
  if (!pool.has_value()) {
    return std::string();
  }
  const CrystalFilterCensus census = CensusForPoolCrystal(state, *pool);
  if (census.any_in || census.out_slots.empty()) {
    return std::string();
  }
  std::string notice;
  if (census.out_slots.size() == 1) {
    const int slot = census.out_slots.front();
    notice = "This crystal already has an Out filter (\"" + state.filters[static_cast<size_t>(slot)].name +
             "\"); the raypath is added to it as one more alternative.";
    // The others on the same pool slot, counted the way the entry card's link badge counts them
    // (panels.cpp CountEntriesSharing), so the two say the same number.
    const int others = CountEntriesSharing(state, *pool, slot) - 1;
    if (others > 0) {
      notice += " Shared with " + Plural(others, "other entry", "other entries") + ": the change reaches all of them.";
    }
  } else {
    notice = "This crystal's entries already carry " +
             Plural(static_cast<int>(census.out_slots.size()), "Out filter", "Out filters") +
             "; the raypath is added to each as one more alternative.";
  }
  if (census.unfiltered > 0) {
    notice += " Its " + Plural(census.unfiltered, "entry", "entries") + " without a filter " +
              (census.unfiltered == 1 ? "gets" : "get") + " a new one.";
  }
  return notice;
}

std::string FormatSegmentRaypathText(const LUMICE_RaypathChainSegment& segment) {
  std::string out;
  const int n = std::max(0, std::min(segment.segment_len, LUMICE_MAX_RAYPATH_SEGMENT_LEN));
  for (int i = 0; i < n; ++i) {
    if (i > 0) {
      out += '-';
    }
    out += std::to_string(segment.segment[i]);
  }
  return out;
}

std::string JoinerForDisplay(std::string_view display) {
  static constexpr std::string_view kCoreJoiner = " -> ";
  static const std::string kGlyphJoiner = std::string(" ") + ICON_FA_ARROW_RIGHT + " ";
  std::string out;
  out.reserve(display.size());
  size_t pos = 0;
  while (pos < display.size()) {
    const size_t found = display.find(kCoreJoiner, pos);
    if (found == std::string_view::npos) {
      out.append(display.substr(pos));
      break;
    }
    out.append(display.substr(pos, found - pos));
    out += kGlyphJoiner;
    pos = found + kCoreJoiner.size();
  }
  return out;
}

bool ApplyExcludeSelectedRaypath(GuiState& state) {
  std::string why;
  if (EvaluateExcludeEligibility(state, &why) != ExcludeEligibility::kOk) {
    GUI_LOG_WARNING("[Analysis] exclude refused: {}", why);
    return false;
  }
  const LUMICE_RaypathHistogramEntry* e = SelectedAnalysisEntry(state);
  const std::optional<int> pool = SelectedChainPoolSlot(state);
  // Eligibility above guarantees both; a second check costs nothing and keeps this function safe
  // to call on its own.
  if (e == nullptr || !pool.has_value()) {
    return false;
  }
  const std::vector<EntryCard*> entries = EntriesForPoolCrystal(state, *pool);
  if (entries.empty()) {
    // Eligibility above permits a pool slot with no current entry (CensusForPoolCrystal's walk
    // over an empty entries list default-constructs to any_in=false, so it never denies this
    // state on its own) — a document edited between the analysis snapshot and this click could in
    // principle leave the chain's crystal referenced by nothing. There is then nothing here to
    // carry the exclusion, so — unlike the idempotent case below, where the exclusion already
    // holds — nothing has actually happened.
    return false;
  }
  RaypathParams rp;
  rp.raypath_text = FormatSegmentRaypathText(e->chain[0]);
  // The chain in the filter editor's own OR-row form, so the export walks it exactly as it walks
  // a row typed there (one summand per ';'-separated alternative; this text never has one).
  const SumOfProducts rows = FromLegacyRaypath(rp);

  // The same classification EvaluateExcludeEligibility/ExcludeAppendNotice already computed over
  // this crystal — which pool slots hold a distinct Out filter, and how many entries hold none —
  // drives the write loop below, so "which slots are already handled" has one authority instead
  // of a second, hand-rolled dedup living here.
  const CrystalFilterCensus census = CensusForPoolCrystal(state, *pool);

  // Every distinct Out filter on this crystal: append. The name, the action and the symmetry are
  // deliberately the FILTER's own, not the list's — this chain joins a filter that already governs
  // other rows under those bits, and giving one OR row its own symmetry is not something the
  // filter model can say (per-row In/Out and symmetry are out of scope, by decision). A row
  // already in it is not added twice, and when every row is already there nothing is written at
  // all, so a second click is a true no-op to the frame-tail reconciler and not a same-content
  // write it has to diff.
  for (int slot : census.out_slots) {
    EntryCard* rep = nullptr;
    for (EntryCard* entry : entries) {
      if (entry->filter_id.has_value() && *entry->filter_id == slot) {
        rep = entry;
        break;
      }
    }
    if (rep == nullptr) {
      continue;  // census was built from these same entries; this cannot happen
    }
    FilterConfig filter = state.filters[static_cast<size_t>(slot)];
    size_t added = 0;
    for (const SummandText& row : rows) {
      if (std::find(filter.param.begin(), filter.param.end(), row) == filter.param.end()) {
        filter.param.push_back(row);
        ++added;
      }
    }
    if (added == 0) {
      GUI_LOG_INFO("[Analysis] raypath {} is already excluded by filter \"{}\" on crystal pool {}; nothing to add",
                   rp.raypath_text, filter.name, *pool);
      continue;
    }
    WriteFilterToPool(state, *rep, filter);  // in place: every entry sharing the slot sees it
    GUI_LOG_INFO("[Analysis] appended raypath {} to filter \"{}\" on crystal pool {} ({} rows now)", rp.raypath_text,
                 filter.name, *pool, filter.param.size());
  }

  if (census.unfiltered > 0) {
    EntryCard* first_unfiltered = nullptr;
    for (EntryCard* entry : entries) {
      if (!entry->filter_id.has_value()) {
        first_unfiltered = entry;
        break;
      }
    }
    FilterConfig filter;
    filter.name = std::string("Exclude ") + e->display;
    filter.action = 1;  // filter_out
    // The row was counted under the symmetry the list on show was reduced with, so the filter
    // matches it under the same bits: fewer and "3-5" would leave orientation-equivalent paths
    // the row merged in the picture; more and it would remove paths the user saw as separate
    // rows.
    const uint8_t sym = state.analysis_result.entries_symmetry;
    filter.sym_p = (sym & LUMICE_RAYPATH_SYMMETRY_P) != 0;
    filter.sym_b = (sym & LUMICE_RAYPATH_SYMMETRY_B) != 0;
    filter.sym_d = (sym & LUMICE_RAYPATH_SYMMETRY_D) != 0;
    filter.param = rows;
    // The first filter-less entry is written through the pool primitive, which appends the slot
    // and propagates the id to the rest of the (crystal, no-filter) group — i.e. to every other
    // entry of the crystal that had no filter.
    WriteFilterToPool(state, *first_unfiltered, filter);
    GUI_LOG_INFO("[Analysis] excluded raypath {} on crystal pool {} (filter \"{}\", symmetry {})", rp.raypath_text,
                 *pool, filter.name, static_cast<int>(sym));
  }
  return true;
}

// ---- Rendering -----------------------------------------------------------------------------------

namespace {

void RenderRoiControls(GuiState& state) {
  auto& a = state.analysis;
  ImGui::TextUnformatted("Region");
  ImGui::SameLine();
  const int prev_mode = a.roi_mode;
  ImGui::RadioButton("Whole sky", &a.roi_mode, LUMICE_RAYPATH_ROI_FULL_SKY);
  ImGui::SameLine();
  // "In frame" is the frame on screen; the radio waits for a preview so the choice is made
  // against a picture. A mode already selected still analyses without one — DoAnalyze sizes the
  // frame at the document's own resolution then — so this gates the radio, not the button.
  ImGui::BeginDisabled(!g_preview_vp.active);
  ImGui::RadioButton("In frame", &a.roi_mode, LUMICE_RAYPATH_ROI_IN_FRAME);
  ImGui::EndDisabled();
  if (!g_preview_vp.active && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip("Needs a preview on screen: 'in frame' means inside the picture as it is shown.");
  }
  ImGui::SameLine();
  ImGui::RadioButton("Point", &a.roi_mode, LUMICE_RAYPATH_ROI_CONE);
  if (a.roi_mode != LUMICE_RAYPATH_ROI_CONE && prev_mode == LUMICE_RAYPATH_ROI_CONE) {
    a.pick_armed = false;  // never leave a click armed for a mode that does not read it
    a.cone_marker_dragging = false;
  }

  if (a.roi_mode == LUMICE_RAYPATH_ROI_CONE) {
    // A centre from the moment the mode is entered, so the marker is on screen to be dragged
    // before any pick. Level-triggered: "no valid centre and a preview to project on", not "the
    // frame the radio was clicked" — a switch made before the first picture arrives is served the
    // frame the picture does. This window renders after the preview panel, so the viewport read
    // here is this frame's.
    if (!a.cone_center_valid && g_preview_vp.active) {
      EnsureDefaultConeCenter(state, PreviewAnnotationView(state, g_preview_vp.vp_w, g_preview_vp.vp_h),
                              g_preview_vp.vp_w, g_preview_vp.vp_h);
    }
    const bool can_pick = g_preview_vp.active;
    ImGui::BeginDisabled(!can_pick);
    // Read once: the button below flips pick_armed, and the pop must match the push made
    // for the value the frame STARTED with, not the one the click just wrote.
    const bool armed_style = a.pick_armed;
    if (armed_style) {
      ImGui::PushStyleColor(ImGuiCol_Button, AccentColor(0.55f));
      ImGui::PushStyleColor(ImGuiCol_ButtonHovered, AccentColor(0.75f));
      ImGui::PushStyleColor(ImGuiCol_ButtonActive, AccentColor(0.40f));
    }
    if (ImGui::Button(ICON_FA_CROSSHAIRS " Pick on preview")) {
      a.pick_armed = !a.pick_armed;
    }
    if (armed_style) {
      ImGui::PopStyleColor(3);
    }
    ImGui::EndDisabled();
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip(can_pick ? "Then click a point on the preview. Esc cancels.\n"
                                   "Or drag the marker on the preview directly." :
                                   "Needs a preview on screen to click on.");
    }
    ImGui::SameLine();
    if (a.cone_center_valid) {
      float alt = 0.0f;
      float az = 0.0f;
      DirToAltAz(a.cone_center_dir, &alt, &az);
      ImGui::Text("Centre: altitude %.1f deg, azimuth %.1f deg", alt, az);
    } else {
      ImGui::TextDisabled("No centre yet");
    }
  }
}

// The request's parameters that are not the region: the ray budget. Session state, sent with the
// next Analyze — editing it starts nothing, like the region.
void RenderRequestParamsControls(GuiState& state) {
  auto& a = state.analysis;
  EnsureDefaultAnalysisRayBudget(state);
  // The document's Rays row, re-drawn for the session's own field: a checkbox that turns the
  // total off and a slider for the total. The RENDERING is kept in step with panels.cpp by hand —
  // evaluated and not shared, because the two rows read different fields under different enable
  // gates (the registry's Applicability versus a.infinite), and a helper would need both spelled
  // as parameters. The DOMAIN is not hand-copied: range, format and scale are the same four symbols
  // the registry row reads (gui/ray_num_domain.hpp), so the two cannot disagree on what a budget is.
  PushLabelColumnItemWidth();
  Checkbox("Infinite rays", &a.infinite);
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip("Trace until stopped.");
  }
  ImGui::PopItemWidth();
  ImGui::BeginGroup();
  ImGui::BeginDisabled(a.infinite);
  SliderWithInput("Rays(M)", &a.ray_num_millions, kRayNumMinMillions, kRayNumMaxMillions, kRayNumSliderFmt,
                  kRayNumSliderScale);
  ImGui::EndDisabled();
  ImGui::EndGroup();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    ImGui::SetTooltip(
        "Total rays the analysis traces across all wavelengths, in millions.\n"
        "Starts from the document's Rays; independent of it from then on.");
  }
}

// The persistent "you are in pick mode" line at the top of the window while a pick is armed. A
// bordered child rather than a bare TextColored, so a test can find it by name — an ImGui text
// item has no id of its own (ItemInfo cannot address it), a child window does.
void RenderPickBanner(const GuiState& state) {
  if (!state.analysis.pick_armed) {
    return;
  }
  ImGui::PushStyleColor(ImGuiCol_ChildBg, AccentColor(0.18f));
  ImGui::PushStyleColor(ImGuiCol_Border, AccentColor(0.9f));
  ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
  const float h = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y;
  if (ImGui::BeginChild("##pick_banner", ImVec2(0.0f, h), ImGuiChildFlags_Borders)) {
    ImGui::AlignTextToFramePadding();
    ImGui::TextColored(AccentColor(),
                       ICON_FA_CROSSHAIRS " Click on the preview to set the centre  \xe2\x80\x94  Esc to cancel");
  }
  ImGui::EndChild();
  ImGui::PopStyleVar();
  ImGui::PopStyleColor(2);
}

void RenderRunControls(GuiState& state, LUMICE_Server* server) {
  const bool in_progress = state.analysis_run_in_progress;
  const bool needs_centre = state.analysis.roi_mode == LUMICE_RAYPATH_ROI_CONE && !state.analysis.cone_center_valid;
  // No frame gate for IN_FRAME: the frame is the document's own view at the preview's canvas
  // when there is one and at the document's own resolution otherwise (DoAnalyze), so the mode
  // always names a frame — a mode left selected before the preview went away still analyses.
  const bool can_start = CanStartAnalysis(server != nullptr, state.sim_state, in_progress) && !needs_centre;

  if (in_progress) {
    PushDestructiveStyle();
    if (ImGui::Button(ICON_FA_STOP " Stop")) {
      DoStop();
    }
    PopDestructiveStyle();
    ImGui::SameLine();
    LUMICE_RayCount rays = 0;
    if (server != nullptr) {
      LUMICE_GetSimRayCount(server, &rays);
    }
    ImGui::TextColored(AccentColor(), "Analyzing (%s)... %llu rays", RoiModeLabel(state.analysis.roi_mode),
                       static_cast<unsigned long long>(rays));
  } else {
    ImGui::BeginDisabled(!can_start);
    PushGoodButtonStyle();
    if (ImGui::Button(ICON_FA_PLAY " Analyze")) {
      DoAnalyze();
    }
    PopGoodButtonStyle();
    ImGui::EndDisabled();
    if (!can_start && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      const char* why = "";
      if (server == nullptr) {
        why = "No simulation server.";
      } else if (IsBusy(state.sim_state)) {
        why = "A render is in progress. Wait for it, or stop it.";
      } else if (needs_centre) {
        why = "Pick the point on the preview first.";
      }
      ImGui::SetTooltip("%s", why);
    }
    if (state.analysis_result.payload) {
      ImGui::SameLine();
      ImGui::TextDisabled("%zu raypaths (%s)", state.analysis_result.payload->entries.size(),
                          RoiModeLabel(state.analysis_result.payload->roi_mode));
      // The record is bounded; when it cut something, say so where the row count is, so the
      // count above is not read as "every raypath there was".
      if (state.analysis_result.payload->truncated_chain_count > 0) {
        ImGui::SameLine();
        ImGui::TextDisabled("; record full (%d hits)", state.analysis_result.payload->truncated_chain_count);
        if (ImGui::IsItemHovered()) {
          ImGui::SetTooltip(
              "The record keeps a fixed number of distinct raypaths per worker. This many times a ray reached a\n"
              "raypath the record had no room for; those rays' energy is the list's \"other\" line, so the\n"
              "percentages still add up to 100.");
        }
      }
    }
    // The marker moved (a drag, a pick) since the result on show was asked for: the list is
    // about the old centre. A hint only — Analyze stays the one way to re-run.
    if (ConeCenterDriftedFromResult(state)) {
      ImGui::TextColored(WarningTextColor(), ICON_FA_TRIANGLE_EXCLAMATION
                         " Centre has moved \xe2\x80\x94 press Analyze to update the list.");
    }
  }
  // What the list describes is the configured document, always; when the picture on screen is
  // not of that document, say so here — a line of its own that needs no hover, not a tooltip,
  // and not a reason to refuse (AnalysisPictureNotice says which two cases there are). Shown in
  // progress too: the list filling in is still not the picture's.
  if (const char* notice = AnalysisPictureNotice(state.run_intent, state.sim_state)) {
    ImGui::TextDisabled("%s", notice);
  }
}

void RenderRadiusSlider(GuiState& state) {
  const auto& payload = state.analysis_result.payload;
  const bool cone_result = payload && payload->roi_mode == LUMICE_RAYPATH_ROI_CONE && payload->cone_ring_count > 0;
  if (state.analysis.roi_mode != LUMICE_RAYPATH_ROI_CONE && !cone_result) {
    return;
  }
  // The slider's range is the result's cone when there is one, the request's otherwise (they are
  // the same numbers unless the constants changed between the run and now).
  const float cone_deg = cone_result ? payload->cone_radius_rad * kRad2Deg : kAnalysisConeMaxRadiusDeg;
  const int rings = cone_result ? payload->cone_ring_count : kAnalysisConeRingCount;
  const float min_deg = cone_deg / static_cast<float>(std::max(rings, 1));
  // Never disabled: the radius is display-time state whether or not a result exists yet. Before
  // one, it sizes the ring on the preview (DrawAnalysisRoiRing reads it every frame) so the
  // user sees the region the list will be summed over; after one, it re-sums the rings on hand.
  // Either way the request is the full cone and nothing here starts a run.
  ImGui::SetNextItemWidth(220.0f);
  if (ImGui::SliderFloat("Radius", &state.analysis.cone_radius_deg, min_deg, cone_deg, "%.1f deg")) {
    // Display-time only: re-sums the rings already on hand (a no-op without one), starts nothing.
    RecomputeAnalysisDisplayOrder(state);
  }
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(cone_result ?
                          "How far from the picked point counts. Re-sums the result on hand; does not re-run." :
                          "How far from the picked point will count, shown as the ring on the preview.\n"
                          "The analysis always traces the whole cone; this picks the part the list sums.");
  }
  if (cone_result) {
    ImGui::SameLine();
    ImGui::TextDisabled("%d / %d rings", state.analysis_result.display_ring_count, rings);
  }
}

// The P/B/D checkboxes — the filter editor's own widget — and the read they drive: a change
// re-reads the result on hand under the new bits (RefreshAnalysisEntries short-circuits when
// nothing changed, so calling it every frame costs nothing). `d_applicable` is passed as true:
// the list spans the whole scene, and "some crystal here has no D" is not "D does nothing".
void RenderSymmetryControls(GuiState& state, LUMICE_Server* server) {
  auto& a = state.analysis;
  ImGui::AlignTextToFramePadding();
  ImGui::TextUnformatted("Symmetry");
  ImGui::SameLine();
  RenderSymmetryCheckboxes(a.symmetry_p, a.symmetry_b, a.symmetry_d, /*d_applicable=*/true, "analysis_symmetry");
  if (ImGui::IsItemHovered()) {
    ImGui::SetTooltip(
        "Merge raypaths that are the same up to this symmetry. Re-reads the result on hand; does not re-run.");
  }
  RefreshAnalysisEntries(state, server);
  // The list could not be re-read under the bits asked for (the server has left the analysis
  // session): say which bits it IS showing rather than let the checkboxes claim otherwise.
  const auto& payload = state.analysis_result.payload;
  if (payload && !payload->entries.empty() && state.analysis_result.entries_symmetry != AnalysisSymmetryBits(state)) {
    ImGui::SameLine();
    ImGui::TextColored(WarningTextColor(), ICON_FA_TRIANGLE_EXCLAMATION " List shows %s; analyze again to apply.",
                       SymmetryBitsLabel(state.analysis_result.entries_symmetry).c_str());
  }
}

void RenderResultList(GuiState& state) {
  const auto& view = state.analysis_result;
  if (!view.payload) {
    ImGui::TextDisabled(state.analysis_run_in_progress ? "Waiting for the first result..." :
                                                         "No result yet. Choose a region and press Analyze.");
    return;
  }
  if (view.payload->entries.empty()) {
    ImGui::TextDisabled("No ray reached this region.");
    return;
  }
  const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY |
                                ImGuiTableFlags_SizingStretchProp;
  const float avail_h = ImGui::GetContentRegionAvail().y - ImGui::GetFrameHeightWithSpacing() * 2.0f;
  if (!ImGui::BeginTable("##analysis_rows", 5, flags, ImVec2(0.0f, std::max(avail_h, 120.0f)))) {
    return;
  }
  ImGui::TableSetupScrollFreeze(0, 1);
  ImGui::TableSetupColumn("Raypath", ImGuiTableColumnFlags_WidthStretch, 3.0f);
  ImGui::TableSetupColumn("Energy", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("Cumulative %", ImGuiTableColumnFlags_WidthStretch, 1.2f);
  ImGui::TableSetupColumn("Rays", ImGuiTableColumnFlags_WidthStretch, 1.0f);
  ImGui::TableSetupColumn("+/-", ImGuiTableColumnFlags_WidthStretch, 0.8f);
  ImGui::TableHeadersRow();

  const double total = view.display_total;
  for (size_t row = 0; row < view.display_order.size(); ++row) {
    const int idx = view.display_order[row];
    const auto& e = view.payload->entries[static_cast<size_t>(idx)];
    const double energy = view.display_energy[static_cast<size_t>(idx)];
    if (!(energy > 0.0)) {
      continue;  // outside the slider's radius: nothing to report for this chain
    }
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    // The id is the ORIGINAL index, so a re-sort moves the row and not the widget; the selection
    // itself is the chain's text, so it survives a re-read under another symmetry too.
    ImGui::PushID(idx);
    // The label is the presentation (arrow glyph for the layer joiner); the selection stays the
    // raw `display`, the text every other reader of the entry compares against.
    const std::string label = JoinerForDisplay(e.display);
    const bool selected = state.analysis.selected_entry.has_value() && *state.analysis.selected_entry == e.display;
    if (ImGui::Selectable(label.c_str(), selected, ImGuiSelectableFlags_SpanAllColumns)) {
      state.analysis.selected_entry = std::string(e.display);
    }
    ImGui::PopID();
    ImGui::TableSetColumnIndex(1);
    ImGui::Text("%.2f%%", total > 0.0 ? energy / total * 100.0 : 0.0);
    ImGui::TableSetColumnIndex(2);
    ImGui::Text("%.1f%%", view.display_cumulative_pct[row]);
    ImGui::TableSetColumnIndex(3);
    ImGui::Text("%llu", static_cast<unsigned long long>(e.count));
    ImGui::TableSetColumnIndex(4);
    // 1/sqrt(N): the relative statistical error of the count, so the noise in the tail reads as
    // noise rather than as signal. A row that took over an evicted slot (error_bound > 0) may
    // also hold energy of some other chain; that bound is shown beside it, as a share of the row.
    const double rel = e.count > 0 ? 1.0 / std::sqrt(static_cast<double>(e.count)) : 1.0;
    if (e.error_bound > 0.0 && e.energy > 0.0) {
      ImGui::Text("%.0f%% (-%.0f%%)", rel * 100.0, e.error_bound / e.energy * 100.0);
      if (ImGui::IsItemHovered()) {
        ImGui::SetTooltip(
            "Up to this share of the row may belong to another raypath: the record was full and this\n"
            "raypath took over the smallest row's energy when it first appeared.");
      }
    } else {
      ImGui::Text("%.0f%%", rel * 100.0);
    }
  }
  // The fixed "other" line: what the record had no room for, so the column above reaches 100. Not
  // a raypath — a DISABLED selectable (addressable, so a test can click it; never pressed, and its
  // press is not read anyway), so it can never become the selection, and the Exclude button (which
  // needs a selected row that IS a chain) is disabled for it by construction.
  if (view.payload->other_count > 0) {
    ImGui::TableNextRow();
    ImGui::TableSetColumnIndex(0);
    ImGui::Selectable(kAnalysisOtherRowLabel, false,
                      ImGuiSelectableFlags_Disabled | ImGuiSelectableFlags_SpanAllColumns);
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
      ImGui::SetTooltip(
          "Rays whose raypath the record had no room for (%d hits on the full record); their energy\n"
          "is counted here as one, so the column above closes at 100.",
          view.payload->truncated_chain_count);
    }
    const double other_pct = AnalysisOtherPct(state);
    ImGui::TableSetColumnIndex(1);
    ImGui::TextDisabled("%.2f%%", other_pct);
    ImGui::TableSetColumnIndex(2);
    ImGui::TextDisabled("%.1f%%",
                        (view.display_cumulative_pct.empty() ? 0.0 : view.display_cumulative_pct.back()) + other_pct);
    ImGui::TableSetColumnIndex(3);
    ImGui::TextDisabled("%llu", static_cast<unsigned long long>(view.payload->other_count));
    ImGui::TableSetColumnIndex(4);
    ImGui::TextDisabled("-");
  }
  ImGui::EndTable();
}

void RenderExcludeButton(GuiState& state) {
  std::string why;
  const ExcludeEligibility elig = EvaluateExcludeEligibility(state, &why);
  ImGui::BeginDisabled(elig != ExcludeEligibility::kOk);
  if (ImGui::Button(ICON_FA_BAN " Exclude this raypath")) {
    ApplyExcludeSelectedRaypath(state);
  }
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) {
    if (elig == ExcludeEligibility::kOk) {
      std::string tip =
          "Add a filter on this chain's crystal that removes rays taking this path, under the symmetry\n"
          "the list is shown with. The document becomes modified; press Run to see the picture without it.";
      const std::string notice = ExcludeAppendNotice(state);
      if (!notice.empty()) {
        tip += "\n" + notice;
      }
      ImGui::SetTooltip("%s", tip.c_str());
    } else {
      ImGui::SetTooltip("%s", why.c_str());
    }
  }
}

}  // namespace

void RenderAnalysisPanel(GuiState& state, LUMICE_Server* server) {
  if (!state.analysis.window_open) {
    return;
  }
  ImGui::SetNextWindowSize(ImVec2(640, 420), ImGuiCond_FirstUseEver);
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_FirstUseEver, ImVec2(0.5f, 0.5f));
  if (!ImGui::Begin(ICON_FA_ROUTE " Raypath Analysis", &state.analysis.window_open,
                    ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoDocking)) {
    ImGui::End();
    return;
  }
  RenderPickBanner(state);
  RenderRoiControls(state);
  RenderRequestParamsControls(state);
  RenderRunControls(state, server);
  RenderRadiusSlider(state);
  RenderSymmetryControls(state, server);
  ImGui::Separator();
  RenderResultList(state);
  RenderExcludeButton(state);
  ImGui::End();
}

void DrawAnalysisRoiRing(const GuiState& state, const LUMICE_AnnotationView& view, const ImVec2& origin,
                         float dpi_scale_x, float dpi_scale_y) {
  const auto& a = state.analysis;
  if (!a.window_open || a.roi_mode != LUMICE_RAYPATH_ROI_CONE) {
    return;
  }
  // This frame's projection of the direction: off the picture (behind the camera, outside the
  // lens's circle, the clipped hemisphere) means no marker and no ring — the direction is still
  // the request's, and the list is unaffected.
  const std::optional<CanvasPixel> marker = ProjectConeCenterMarker(state, view);
  if (!marker.has_value()) {
    return;
  }
  // The ring's radius is the local scale at the marker's pixel — a linearisation, as
  // ConeRingRadiusCanvasPx says; that approximation is unchanged by placing the centre per frame.
  const std::optional<float> radius_px = ConeRingRadiusCanvasPx(view, marker->px, marker->py, a.cone_radius_deg);
  float cx = 0.0f;
  float cy = 0.0f;
  CanvasPixelToPreviewPoint(marker->px, marker->py, dpi_scale_x, dpi_scale_y, &cx, &cy);
  const ImVec2 centre(origin.x + cx, origin.y + cy);
  ImDrawList* fg = ImGui::GetForegroundDrawList();
  const ImU32 colour = ImGui::ColorConvertFloat4ToU32(AccentColor());
  // The centre mark is always drawn; the ring only when the local scale could be measured. A drag
  // in flight draws the dot larger, so the grab reads as taken.
  fg->AddCircleFilled(centre, a.cone_marker_dragging ? kRoiMarkerDotRadiusPt * 1.6f : kRoiMarkerDotRadiusPt, colour);
  if (radius_px.has_value()) {
    fg->AddCircle(centre, *radius_px / dpi_scale_x, colour, kRoiRingSegments, kRoiRingThicknessPt);
  }
}

}  // namespace lumice::gui
