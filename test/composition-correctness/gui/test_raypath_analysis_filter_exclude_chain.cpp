// The exclude closure end to end across the units it wires together: an analysis result in
// GuiState -> "Exclude this raypath" (analysis_panel.cpp) -> the filter pool through the edit
// modal's write primitive (edit_modals.cpp) -> the exported core JSON (file_io.cpp). What the CLI
// would be handed is the proposition, so the assertion is on the JSON the export produces, not on
// the GuiState the panel wrote (that half is the unit layer's).

#include <gtest/gtest.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "gui/analysis_panel.hpp"
#include "gui/analysis_result.hpp"
#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/gui_state_reconcile.hpp"
#include "lumice.h"
#include "support/scene_json_helpers.hpp"

namespace lumice::gui {
namespace {

using lumice::test::CoreJson;

// One prism, no filter, one entry — the shape the 22-degree halo's chain comes from.
void SeedUnfilteredPrismDocument() {
  DoNew();
  g_state.crystals.assign(1, CrystalConfig{});
  g_state.crystals[0].type = CrystalType::kPrism;
  g_state.crystals[0].height = 1.0f;
  for (int i = 0; i < 6; ++i) {
    g_state.crystals[0].face_distance[i] = 1.0f;
  }
  g_state.filters.clear();
  Layer layer;
  layer.probability = 1.0f;
  EntryCard entry;
  entry.crystal_id = 0;
  entry.proportion = 100.0f;
  layer.entries.assign(1, entry);
  g_state.layers.assign(1, layer);
}

std::shared_ptr<AnalysisPayload> OneChainResult(int chain_len) {
  auto p = std::make_shared<AnalysisPayload>();
  p->snapshot_generation = 1;
  p->roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  LUMICE_RaypathHistogramEntry e{};
  e.chain_len = chain_len;
  for (int l = 0; l < chain_len; ++l) {
    e.chain[l].crystal_id = 0;  // the one committed crystal is scene id 0
    e.chain[l].segment_len = 2;
    e.chain[l].segment[0] = 3;
    e.chain[l].segment[1] = 5;
  }
  snprintf(e.display, sizeof(e.display), "%s", chain_len == 1 ? "3-5" : "(3-5) -> (3-5)");
  e.energy = 1.0;
  e.count = 1000;
  p->entries.push_back(e);
  return p;
}

TEST(RaypathAnalysisFilterExcludeChain, SingleSegmentExcludeReachesTheExportedConfigAsFilterOut) {
  SeedUnfilteredPrismDocument();
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult(1)));
  // As RefreshAnalysisEntries leaves it after a read under the panel's default: the list on show
  // is P|B|D-reduced.
  g_state.analysis_result.entries_symmetry =
      LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D;
  g_state.analysis.selected_entry = "3-5";
  ASSERT_EQ(EvaluateExcludeEligibility(g_state, nullptr), ExcludeEligibility::kOk);
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(g_state));

  nlohmann::json doc;
  ASSERT_NO_THROW(doc = nlohmann::json::parse(CoreJson(g_state)));
  ASSERT_TRUE(doc.contains("filter"));
  ASSERT_EQ(doc["filter"].size(), 1u);
  const nlohmann::json& f = doc["filter"][0];
  EXPECT_EQ(f["type"], "raypath");
  EXPECT_EQ(f["action"], "filter_out");
  EXPECT_EQ(f["raypath"], nlohmann::json({ 3, 5 }));
  // The symmetry the list on show was reduced under, so the exclusion covers every
  // orientation-equivalent 3-5 the row merged and not just the one face labelling it printed.
  EXPECT_EQ(f["symmetry"], "PBD");
  // And the entry references it: the export's scattering entry carries the filter id.
  ASSERT_TRUE(doc.contains("scene") && doc["scene"].contains("scattering"));
  const nlohmann::json& entry = doc["scene"]["scattering"][0]["entries"][0];
  EXPECT_EQ(entry["filter"], f["id"]);
}

// The document is modified by the exclusion the way any filter edit modifies it: the frame-tail
// reconciler diffs the filter pool against the Revert baseline and asks for a hard reset. Pinned
// here because the panel itself calls no MarkDirty — that is the governance rule, and this is
// where it would silently break.
TEST(RaypathAnalysisFilterExcludeChain, ExcludeIsAHardStructChangeToTheReconciler) {
  SeedUnfilteredPrismDocument();
  g_state.last_committed_state = GuiState::ConfigSnapshot::From(g_state);
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult(1)));
  g_state.analysis.selected_entry = "3-5";
  const GuiEffects before = ReconcileGuiEffects(g_state);
  EXPECT_FALSE(before.need_resim);
  EXPECT_FALSE(before.need_hard_reset);
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(g_state));
  const GuiEffects after = ReconcileGuiEffects(g_state);
  EXPECT_TRUE(after.need_resim);
  EXPECT_TRUE(after.need_hard_reset);
}

TEST(RaypathAnalysisFilterExcludeChain, MultiSegmentChainIsRefusedAndExportsNoFilter) {
  SeedUnfilteredPrismDocument();
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult(2)));
  g_state.analysis.selected_entry = "(3-5) -> (3-5)";
  std::string why;
  EXPECT_EQ(EvaluateExcludeEligibility(g_state, &why), ExcludeEligibility::kMultiSegment);
  EXPECT_NE(why.find("scattering layers"), std::string::npos);
  EXPECT_FALSE(ApplyExcludeSelectedRaypath(g_state));
  nlohmann::json doc;
  ASSERT_NO_THROW(doc = nlohmann::json::parse(CoreJson(g_state)));
  EXPECT_TRUE(!doc.contains("filter") || doc["filter"].empty());
}

}  // namespace
}  // namespace lumice::gui
