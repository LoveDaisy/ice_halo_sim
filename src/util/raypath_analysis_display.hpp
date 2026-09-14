#pragma once

// The display-time projection of a raypath-analysis result and its CSV form — the ONE
// implementation both consumers of LUMICE_FrameGetRaypathAnalysis print through: the GUI's
// Raypath Analysis window (src/gui/analysis_panel.cpp, its list and its Export CSV button) and
// the CLI's `analyze` subcommand (src/main.cpp). Same entries in, same bytes out, so a file the
// CLI writes and a file the GUI exports for the same run agree by construction rather than by
// two implementations being kept in step.
//
// Lives in src/util/ under the shape AGENTS.md admits there: pure, stateless, no core or config
// type — it reads only lumice.h types, like util/result_frame.hpp beside it. Every function is
// `inline` on purpose: src/gui/ and its test targets link `lumice` / `lumice_gui_obj`, never
// `lumice_obj`, and the shared build's -fvisibility=hidden leaves a .cpp definition here with no
// exported symbol for them to reach (src/gui/color_space.hpp records the same lesson).
//
// What is deliberately NOT here: the GUI's AnalysisResultView (it holds this projection beside
// the payload so a slider drag never touches the data a running analysis is compared against),
// its selection, and the "no result" branch of its export — the CLI always has a real result.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <numeric>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "lumice.h"
#include "util/sky_direction.hpp"

namespace lumice {

// The ring split of every CONE request either consumer sends: the full cone the GUI's slider or
// the CLI's --radius names, split into this many equal angular-distance rings (0.5 degrees per
// ring at the GUI's 15-degree maximum), so that the GUI can re-sum a narrower radius at display
// time without a new run. The CLI sums every ring (display radius = request radius), so for it
// the count only sets the resolution of `cone_rings_summed` in the CSV head.
constexpr int kRaypathAnalysisConeRingCount = 30;
static_assert(kRaypathAnalysisConeRingCount >= 1 && kRaypathAnalysisConeRingCount <= LUMICE_MAX_RAYPATH_CONE_RINGS,
              "the cone request must fit the C API's ring cap");

// The fixed last line of the list and of the CSV: the record's "other" bucket
// (LUMICE_RaypathAnalysisInfo other_energy / other_count). No chain formats to this — a chain's
// text is digits, dashes, parentheses, "C<id>" and " -> " — so it can never name an entry.
inline constexpr const char* kRaypathAnalysisOtherRowLabel = "other (not recorded)";

// How many of a CONE result's rings lie within `radius_deg` of the centre, given the request's
// full cone of `cone_radius_deg` split into `ring_count` equal rings: the smallest k such that
// k rings cover the radius, clamped to [1, ring_count]. ring_count <= 0 answers 0.
inline int RingsWithinRadius(float radius_deg, float cone_radius_deg, int ring_count) {
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

// Energy of the first `rings` rings of an entry (clamped to what the entry holds). Rings <= 0 is 0.
inline double SumRingEnergy(const LUMICE_RaypathHistogramEntry& entry, int rings) {
  const int n = std::max(0, std::min({ rings, entry.ring_count, LUMICE_MAX_RAYPATH_CONE_RINGS }));
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += entry.ring_energy[i];
  }
  return sum;
}

// "P|B|D", "P|B", ..., or "no symmetry": a LUMICE_RAYPATH_SYMMETRY_* bit set as the user reads it.
inline std::string SymmetryBitsLabel(std::uint8_t bits) {
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

// The LUMICE_RAYPATH_ROI_* mode as the user reads it.
inline const char* RoiModeLabel(int mode) {
  switch (mode) {
    case LUMICE_RAYPATH_ROI_IN_FRAME:
      return "in frame";
    case LUMICE_RAYPATH_ROI_CONE:
      return "point";
    default:
      return "whole sky";
  }
}

// The list as shown: each entry's displayed energy, the rows in display order, the cumulative
// percentage down that order, and the denominator they are percentages of.
struct RaypathDisplayOrder {
  // Per entry (indexed like the entries): the energy shown — the rings inside the display radius
  // for a CONE result, `energy` as delivered otherwise.
  std::vector<double> display_energy;
  // Indices into the entries, descending by display_energy; ties keep the C API's order (energy
  // descending, display ascending), so the list is deterministic for equal sums too.
  std::vector<int> display_order;
  // Per ROW of display_order: the running share of display_total, in percent. Monotone by
  // construction; its last value plus the "other" line's share is 100.
  std::vector<double> display_cumulative_pct;
  // The sum of every display_energy PLUS the other bucket's energy whole — the bucket is not
  // ring-split, so it enters the denominator in full in every mode.
  double display_total = 0.0;
  // How many rings the display radius covers (CONE only, else 0).
  int display_ring_count = 0;
};

// Rebuild the display order from the entries and the display radius: a CONE result
// (roi_mode == LUMICE_RAYPATH_ROI_CONE with cone_ring_count > 0) sums the rings inside
// `display_radius_deg` of a `cone_radius_rad` cone split into `cone_ring_count` rings; every other
// mode shows `energy` as delivered. Pure re-projection of data already on hand.
inline RaypathDisplayOrder ComputeRaypathDisplayOrder(const std::vector<LUMICE_RaypathHistogramEntry>& entries,
                                                      int roi_mode, int cone_ring_count, float cone_radius_rad,
                                                      float display_radius_deg, double other_energy) {
  RaypathDisplayOrder view;
  const bool cone = roi_mode == LUMICE_RAYPATH_ROI_CONE && cone_ring_count > 0;
  if (cone) {
    view.display_ring_count = RingsWithinRadius(display_radius_deg, cone_radius_rad * kRad2Deg, cone_ring_count);
  }
  view.display_energy.reserve(entries.size());
  for (const auto& e : entries) {
    view.display_energy.push_back(cone ? SumRingEnergy(e, view.display_ring_count) : e.energy);
    view.display_total += view.display_energy.back();
  }
  view.display_order.resize(entries.size());
  std::iota(view.display_order.begin(), view.display_order.end(), 0);
  std::stable_sort(view.display_order.begin(), view.display_order.end(),
                   [&](int a, int b) { return view.display_energy[a] > view.display_energy[b]; });
  view.display_total += other_energy;
  view.display_cumulative_pct.reserve(view.display_order.size());
  double running = 0.0;
  for (const int idx : view.display_order) {
    running += view.display_energy[static_cast<size_t>(idx)];
    view.display_cumulative_pct.push_back(view.display_total > 0.0 ? running / view.display_total * 100.0 : 0.0);
  }
  return view;
}

// The share of the total that the fixed "other" line shows: the bucket's energy over
// display_total, as a percentage; 0 with an empty total.
inline double RaypathOtherPct(const RaypathDisplayOrder& view, double other_energy) {
  if (!(view.display_total > 0.0)) {
    return 0.0;
  }
  return other_energy / view.display_total * 100.0;
}

// Everything the CSV head echoes about the run beyond the entries themselves. Named fields
// rather than a positional list on purpose: three of them are floats in degrees or radians, and
// the request radius and the display radius are the pair most easily passed in the wrong order.
struct RaypathAnalysisCsvInputs {
  int roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  // CONE only: the centre the result was ASKED for (not a live marker that may have moved since),
  // the request's radius and ring count as the frame echoes them, and the radius on show.
  float cone_center_dir[3] = { 0.0f, 0.0f, 0.0f };
  float cone_request_radius_rad = 0.0f;
  int cone_ring_count = 0;
  float cone_display_radius_deg = 0.0f;
  // The LUMICE_RAYPATH_SYMMETRY_* bits the entries were READ under.
  std::uint8_t symmetry_bits = 0;
  // LUMICE_RaypathAnalysisInfo's account of what the record could not keep as rows.
  double other_energy = 0.0;
  LUMICE_RayCount other_count = 0;
  int truncated_chain_count = 0;
};

namespace raypath_analysis_display_detail {

// RFC 4180 quoting, applied only when the field needs it. Today no field does — a chain's display
// text is digits, '-', parentheses, "C<id>" and " -> " — but the rule is cheap and the text's
// grammar is core's to grow.
inline std::string EscapeCsvField(std::string_view field) {
  if (field.find_first_of(",\"\n\r") == std::string_view::npos) {
    return std::string(field);
  }
  std::string out = "\"";
  for (const char c : field) {
    if (c == '"') {
      out += '"';
    }
    out += c;
  }
  out += '"';
  return out;
}

// printf into a std::string; every number in the file goes through one of these three so the
// precision rules are stated once. Percentages carry two more decimals than the GUI's table (the
// file is for a tool, the table for an eye); energies are %g at six digits, the raw double's shape.
inline std::string Fmt(const char* fmt, double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), fmt, v);
  return buf;
}
inline std::string Pct(double v) {
  return Fmt("%.4f", v);
}
inline std::string Energy(double v) {
  return Fmt("%.6g", v);
}

}  // namespace raypath_analysis_display_detail

// The result as CSV text: a metadata head of `#` lines makes the file self-describing (region,
// cone centre / radius / rings, symmetry, ray total, record-full count, export time), then one
// column-header row, the rows of `view.display_order` with their display energies as percentages
// of `view.display_total`, and the fixed "other" line when the record had one. Rows with a display
// energy of 0 (outside the radius) are hidden, as the GUI's table hides them. `view` must have
// been computed from `entries` (ComputeRaypathDisplayOrder) with the same display radius
// `in.cone_display_radius_deg` names, so the head and the rows describe one projection.
// `exported_at` is a parameter rather than a clock read inside, so a test's expectation does
// not need to mock time.
//
// Four columns, matching the GUI table's header one for one: the file's schema is a single
// authoritative definition, not a superset the table happens to be a projection of. "Energy" and
// "+/-" are the same percentages the table cell shows, not the underlying raw doubles; "+/-"
// embeds the takeover figure in parentheses exactly as the table cell does, so a row that took
// over an evicted slot still carries that information in its one cell. The per-row hit count is
// not a column; the run-level total_rays line in the head is the one ray count the file carries.
inline std::string BuildRaypathAnalysisCsv(const std::vector<LUMICE_RaypathHistogramEntry>& entries,
                                           const RaypathDisplayOrder& view, const RaypathAnalysisCsvInputs& in,
                                           std::string_view exported_at) {
  using raypath_analysis_display_detail::Energy;
  using raypath_analysis_display_detail::EscapeCsvField;
  using raypath_analysis_display_detail::Fmt;
  using raypath_analysis_display_detail::Pct;
  std::string out;
  out += "# Lumice raypath analysis\n";
  out += "# exported_at: " + std::string(exported_at) + "\n";
  out += std::string("# region: ") + RoiModeLabel(in.roi_mode) + "\n";
  if (in.roi_mode == LUMICE_RAYPATH_ROI_CONE) {
    float alt = 0.0f;
    float az = 0.0f;
    DirToAltAz(in.cone_center_dir, &alt, &az);
    out += "# cone_centre_altitude_deg: " + Fmt("%.2f", alt) + "\n";
    out += "# cone_centre_azimuth_deg: " + Fmt("%.2f", az) + "\n";
    out += "# cone_request_radius_deg: " + Fmt("%.1f", in.cone_request_radius_rad * kRad2Deg) + "\n";
    out += "# cone_display_radius_deg: " + Fmt("%.1f", in.cone_display_radius_deg) + "\n";
    out += "# cone_rings_summed: " + std::to_string(view.display_ring_count) + " / " +
           std::to_string(in.cone_ring_count) + "\n";
  }
  out += "# symmetry: " + SymmetryBitsLabel(in.symmetry_bits) + "\n";
  std::uint64_t total_rays = static_cast<std::uint64_t>(in.other_count);
  for (const auto& e : entries) {
    total_rays += static_cast<std::uint64_t>(e.count);
  }
  out += "# total_rays: " + std::to_string(total_rays) + "\n";
  out += "# total_energy: " + Energy(view.display_total) + "\n";
  out += "# record_full_hits: " + std::to_string(in.truncated_chain_count) + "\n";
  out += "Raypath,Energy,Cumulative %,+/-\n";
  const double total = view.display_total;
  for (size_t row = 0; row < view.display_order.size(); ++row) {
    const int idx = view.display_order[row];
    const auto& e = entries[static_cast<size_t>(idx)];
    const double energy = view.display_energy[static_cast<size_t>(idx)];
    if (!(energy > 0.0)) {
      continue;  // the table hides these rows too
    }
    const double rel = e.count > 0 ? 1.0 / std::sqrt(static_cast<double>(e.count)) : 1.0;
    out += EscapeCsvField(e.display);
    out += ',' + Pct(total > 0.0 ? energy / total * 100.0 : 0.0);
    out += ',' + Pct(view.display_cumulative_pct[row]);
    out += ',' + Fmt("%.2f", rel * 100.0);
    if (e.error_bound > 0.0 && e.energy > 0.0) {
      out += " (-" + Fmt("%.2f", e.error_bound / e.energy * 100.0) + ")";
    }
    out += '\n';
  }
  if (in.other_count > 0) {
    const double other_pct = RaypathOtherPct(view, in.other_energy);
    const double cum = (view.display_cumulative_pct.empty() ? 0.0 : view.display_cumulative_pct.back()) + other_pct;
    out += std::string(kRaypathAnalysisOtherRowLabel) + ',' + Pct(other_pct) + ',' + Pct(cum) + ",-\n";
  }
  return out;
}

}  // namespace lumice
