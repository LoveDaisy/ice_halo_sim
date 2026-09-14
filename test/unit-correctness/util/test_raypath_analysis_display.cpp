#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "util/raypath_analysis_display.hpp"

// The single CSV / display-order implementation both the GUI's Export CSV and the CLI's `analyze`
// print through (src/util/raypath_analysis_display.hpp). The expectations below are the SAME bytes
// the GUI-side cases in test/unit-correctness/gui/test_analysis_panel_logic.cpp hold
// BuildAnalysisResultsCsv to, on the same entries: that pair is the mechanical statement that
// lifting the formatter out of the GUI changed nothing, and that the CLI — which calls this
// function on entries it read itself — writes what the GUI would have exported.

namespace {

// Mirror of the GUI test's MakePayload: `n` single-crystal entries whose energies are NOT in
// descending order (the C API would deliver them sorted; the sort has to be proven here, not
// assumed). Entry i: crystal 0, faces (i+1)-(i+2), energy given, count 100*(i+1), and with rings
// all of the energy in ring (i mod ring_count).
std::vector<LUMICE_RaypathHistogramEntry> MakeEntries(const std::vector<double>& energies, int ring_count = 0) {
  std::vector<LUMICE_RaypathHistogramEntry> out;
  for (size_t i = 0; i < energies.size(); ++i) {
    LUMICE_RaypathHistogramEntry e{};
    e.chain_len = 1;
    e.chain[0].crystal_id = 0;
    e.chain[0].segment_len = 2;
    e.chain[0].segment[0] = static_cast<int>(i) + 1;
    e.chain[0].segment[1] = static_cast<int>(i) + 2;
    std::snprintf(e.display, sizeof(e.display), "%d-%d", e.chain[0].segment[0], e.chain[0].segment[1]);
    e.energy = energies[i];
    e.count = 100 * (static_cast<LUMICE_RayCount>(i) + 1);
    e.ring_count = ring_count;
    if (ring_count > 0) {
      e.ring_energy[static_cast<size_t>(i) % static_cast<size_t>(ring_count)] = energies[i];
    }
    out.push_back(e);
  }
  return out;
}

constexpr float kDeg2Rad = 3.14159265358979323846f / 180.0f;

}  // namespace

TEST(RaypathAnalysisDisplay, WholeSkyCsvMatchesTheGuiExpectationByteForByte) {
  auto entries = MakeEntries({ 1.0, 5.0, 3.0 });
  // Row "3-4" (energy 3, count 300) took over an evicted slot: a takeover bound of 0.6 of its 3.0.
  entries[2].error_bound = 0.6;
  lumice::RaypathAnalysisCsvInputs in;
  in.roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  in.symmetry_bits = LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_D;
  in.other_energy = 3.0;
  in.other_count = 30;
  in.truncated_chain_count = 4;
  const auto view = lumice::ComputeRaypathDisplayOrder(entries, in.roi_mode, 0, 0.0f, 0.0f, in.other_energy);
  EXPECT_DOUBLE_EQ(view.display_total, 12.0);
  const std::string csv = lumice::BuildRaypathAnalysisCsv(entries, view, in, "2026-09-13 10:30:00");
  // total 12: rows 5/12, 3/12, 1/12 -> 41.6667, 25.0000, 8.3333; cumulative 41.6667, 66.6667, 75;
  // other 25 closes at 100. Counts 200, 300, 100 set +/- 1/sqrt: 7.07, 5.77, 10.00.
  const std::string want =
      "# Lumice raypath analysis\n"
      "# exported_at: 2026-09-13 10:30:00\n"
      "# region: whole sky\n"
      "# symmetry: P|D\n"
      "# total_rays: 630\n"
      "# total_energy: 12\n"
      "# record_full_hits: 4\n"
      "Raypath,Energy,Cumulative %,+/-\n"
      "2-3,41.6667,41.6667,7.07\n"
      "3-4,25.0000,66.6667,5.77 (-20.00)\n"
      "1-2,8.3333,75.0000,10.00\n"
      "other (not recorded),25.0000,100.0000,-\n";
  EXPECT_EQ(csv, want);
}

TEST(RaypathAnalysisDisplay, ConeCsvMatchesTheGuiExpectationByteForByte) {
  auto entries = MakeEntries({ 2.0, 9.0, 50.0, 60.0 }, 4);
  lumice::RaypathAnalysisCsvInputs in;
  in.roi_mode = LUMICE_RAYPATH_ROI_CONE;
  // The centre the result was requested with: straight down the +x axis, i.e. light travelling
  // horizontally from the sky point at altitude 0, azimuth 180 (DirToAltAz's convention).
  in.cone_center_dir[0] = 1.0f;
  in.cone_request_radius_rad = 4.0f * kDeg2Rad;
  in.cone_ring_count = 4;
  in.cone_display_radius_deg = 1.0f;  // ring 0 of 4 only
  in.symmetry_bits = 0;
  in.other_energy = 8.0;
  in.other_count = 80;
  const auto view =
      lumice::ComputeRaypathDisplayOrder(entries, in.roi_mode, in.cone_ring_count, in.cone_request_radius_rad,
                                         in.cone_display_radius_deg, in.other_energy);
  EXPECT_EQ(view.display_ring_count, 1);
  EXPECT_DOUBLE_EQ(view.display_total, 10.0);
  const std::string csv = lumice::BuildRaypathAnalysisCsv(entries, view, in, "t");
  // Only entry 0 (ring 0) shows energy: 2 of a 10 total; the three rows outside the radius are
  // absent; the bucket enters whole and closes at 100.
  const std::string want =
      "# Lumice raypath analysis\n"
      "# exported_at: t\n"
      "# region: point\n"
      "# cone_centre_altitude_deg: -0.00\n"
      "# cone_centre_azimuth_deg: -180.00\n"
      "# cone_request_radius_deg: 4.0\n"
      "# cone_display_radius_deg: 1.0\n"
      "# cone_rings_summed: 1 / 4\n"
      "# symmetry: no symmetry\n"
      "# total_rays: 1080\n"
      "# total_energy: 10\n"
      "# record_full_hits: 0\n"
      "Raypath,Energy,Cumulative %,+/-\n"
      "1-2,20.0000,20.0000,10.00\n"
      "other (not recorded),80.0000,100.0000,-\n";
  EXPECT_EQ(csv, want);
}

// The CLI's shape of a cone read: display radius = request radius, so every ring is summed and
// every row shows; the head says so in cone_rings_summed.
TEST(RaypathAnalysisDisplay, ConeWithDisplayRadiusEqualToRequestSumsEveryRing) {
  auto entries = MakeEntries({ 2.0, 9.0, 50.0, 60.0 }, 4);
  lumice::RaypathAnalysisCsvInputs in;
  in.roi_mode = LUMICE_RAYPATH_ROI_CONE;
  in.cone_center_dir[2] = -1.0f;  // the zenith
  in.cone_request_radius_rad = 4.0f * kDeg2Rad;
  in.cone_ring_count = 4;
  in.cone_display_radius_deg = 4.0f;
  in.symmetry_bits = LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D;
  const auto view =
      lumice::ComputeRaypathDisplayOrder(entries, in.roi_mode, in.cone_ring_count, in.cone_request_radius_rad,
                                         in.cone_display_radius_deg, in.other_energy);
  EXPECT_EQ(view.display_ring_count, 4);
  EXPECT_DOUBLE_EQ(view.display_total, 121.0);
  ASSERT_EQ(view.display_order.size(), 4u);
  EXPECT_EQ(view.display_order[0], 3);  // 60 first, then 50, 9, 2
  EXPECT_EQ(view.display_order[3], 0);
  const std::string csv = lumice::BuildRaypathAnalysisCsv(entries, view, in, "t");
  EXPECT_NE(csv.find("# cone_centre_altitude_deg: 90.00\n"), std::string::npos);
  EXPECT_NE(csv.find("# cone_rings_summed: 4 / 4\n"), std::string::npos);
  EXPECT_NE(csv.find("# symmetry: P|B|D\n"), std::string::npos);
  EXPECT_NE(csv.find("# total_energy: 121\n"), std::string::npos);
  EXPECT_EQ(csv.find("other (not recorded)"), std::string::npos) << "no bucket, no other line";
  // Four rows after the column header, in descending energy.
  const auto header = csv.find("Raypath,Energy,Cumulative %,+/-\n");
  ASSERT_NE(header, std::string::npos);
  const std::string rows = csv.substr(header + std::string("Raypath,Energy,Cumulative %,+/-\n").size());
  EXPECT_EQ(rows,
            "4-5,49.5868,49.5868,5.00\n"
            "3-4,41.3223,90.9091,5.77\n"
            "2-3,7.4380,98.3471,7.07\n"
            "1-2,1.6529,100.0000,10.00\n");
}

TEST(RaypathAnalysisDisplay, LabelsAndOtherPct) {
  EXPECT_STREQ(lumice::RoiModeLabel(LUMICE_RAYPATH_ROI_FULL_SKY), "whole sky");
  EXPECT_STREQ(lumice::RoiModeLabel(LUMICE_RAYPATH_ROI_IN_FRAME), "in frame");
  EXPECT_STREQ(lumice::RoiModeLabel(LUMICE_RAYPATH_ROI_CONE), "point");
  EXPECT_EQ(lumice::SymmetryBitsLabel(0), "no symmetry");
  EXPECT_EQ(lumice::SymmetryBitsLabel(LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B), "P|B");
  EXPECT_EQ(lumice::SymmetryBitsLabel(7), "P|B|D");
  lumice::RaypathDisplayOrder empty;
  EXPECT_DOUBLE_EQ(lumice::RaypathOtherPct(empty, 5.0), 0.0) << "an empty total is 0, not a division";
  empty.display_total = 20.0;
  EXPECT_DOUBLE_EQ(lumice::RaypathOtherPct(empty, 5.0), 25.0);
}

TEST(RaypathAnalysisDisplay, CsvFieldQuotingIsRfc4180) {
  using lumice::raypath_analysis_display_detail::EscapeCsvField;
  EXPECT_EQ(EscapeCsvField("3-5"), "3-5");
  EXPECT_EQ(EscapeCsvField("(3-5) -> (1-3)"), "(3-5) -> (1-3)");
  EXPECT_EQ(EscapeCsvField("a,b"), "\"a,b\"");
  EXPECT_EQ(EscapeCsvField("say \"hi\""), "\"say \"\"hi\"\"\"");
}
