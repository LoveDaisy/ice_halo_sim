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
#include "gui/raypath_segments.hpp"
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

// The entry already carries an Out filter on 1-3. Excluding 3-5 does not stack a second filter
// and does not refuse: the chain becomes one more OR alternative of the filter there, which the
// export writes the way it writes any two-row raypath filter (test_scene_commit_chain.cpp
// MultiSegmentRaypathBecomesChildFiltersPlusOneComposition): two raypath children plus one complex
// composing them, and the entry points at the composed one. The filter's own symmetry (P only,
// set here to differ from the list's P|B|D) is what the export carries, on both children — an
// Out filter keeps governing its rows under the bits it was written with. And a second Exclude of
// the same chain changes nothing the export can see.
TEST(RaypathAnalysisFilterExcludeChain, AppendsOntoExistingOutFilterAsASecondSummandAndIsIdempotent) {
  SeedUnfilteredPrismDocument();
  FilterConfig existing;
  existing.name = "drop 1-3";
  existing.action = 1;
  existing.sym_p = true;
  existing.sym_b = false;
  existing.sym_d = false;
  existing.param = FromLegacyRaypath(RaypathParams{ "1-3" });
  g_state.filters.assign(1, existing);
  g_state.layers[0].entries[0].filter_id = 0;
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult(1)));
  g_state.analysis_result.entries_symmetry =
      LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D;
  g_state.analysis.selected_entry = "3-5";
  std::string why;
  ASSERT_EQ(EvaluateExcludeEligibility(g_state, &why), ExcludeEligibility::kOk) << why;
  const std::string notice = ExcludeAppendNotice(g_state);
  EXPECT_NE(notice.find("\"drop 1-3\""), std::string::npos) << notice;
  EXPECT_EQ(notice.find("Shared with"), std::string::npos) << "one entry, nothing shared: " << notice;
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(g_state));

  auto exported = [] {
    nlohmann::json doc;
    EXPECT_NO_THROW(doc = nlohmann::json::parse(CoreJson(g_state)));
    return doc;
  };
  const nlohmann::json doc = exported();
  ASSERT_TRUE(doc.contains("filter"));
  const nlohmann::json& filters = doc["filter"];
  ASSERT_EQ(filters.size(), 3u) << filters.dump();
  EXPECT_EQ(filters[0]["type"], "raypath");
  EXPECT_EQ(filters[0]["raypath"], nlohmann::json({ 1, 3 })) << "the row that was there first";
  EXPECT_EQ(filters[0]["symmetry"], "P");
  EXPECT_EQ(filters[1]["type"], "raypath");
  EXPECT_EQ(filters[1]["raypath"], nlohmann::json({ 3, 5 })) << "the excluded chain, after it";
  EXPECT_EQ(filters[1]["symmetry"], "P") << "the filter's own bits, not the list's P|B|D";
  EXPECT_EQ(filters[2]["type"], "complex");
  EXPECT_EQ(filters[2]["action"], "filter_out");
  EXPECT_EQ(filters[2]["composition"], nlohmann::json({ filters[0]["id"].get<int>(), filters[1]["id"].get<int>() }));
  const nlohmann::json& entry = doc["scene"]["scattering"][0]["entries"][0];
  EXPECT_EQ(entry["filter"], filters[2]["id"]) << "the entry applies the composition, not one child";
  // The GuiState side of the same statement: one pool slot, extended in place.
  ASSERT_EQ(g_state.filters.size(), 1u);
  EXPECT_EQ(g_state.filters[0].name, "drop 1-3");

  // Again: the same chain is already a row; nothing is added and the export is byte-identical.
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(g_state));
  EXPECT_EQ(g_state.filters.size(), 1u);
  EXPECT_EQ(g_state.filters[0].param.size(), 2u);
  EXPECT_EQ(exported(), doc);
}

// Two entries on two layers share the crystal AND its Out filter (one pool slot, referenced
// twice — the linked group gui_state.hpp describes). Excluding through the analysis panel
// extends that one slot, so both entries export the same composed filter; and the tooltip says
// so before the click.
TEST(RaypathAnalysisFilterExcludeChain, AppendPropagatesToEveryLinkedEntrySharingTheFilter) {
  SeedUnfilteredPrismDocument();
  FilterConfig existing;
  existing.name = "shared";
  existing.action = 1;
  existing.param = FromLegacyRaypath(RaypathParams{ "1-3" });
  g_state.filters.assign(1, existing);
  g_state.layers[0].entries[0].filter_id = 0;
  Layer second = g_state.layers[0];
  second.probability = 0.5f;
  g_state.layers.push_back(second);
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult(1)));
  g_state.analysis_result.entries_symmetry =
      LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D;
  g_state.analysis.selected_entry = "3-5";
  ASSERT_EQ(EvaluateExcludeEligibility(g_state, nullptr), ExcludeEligibility::kOk);
  EXPECT_NE(ExcludeAppendNotice(g_state).find("Shared with 1 other entry"), std::string::npos)
      << ExcludeAppendNotice(g_state);
  ASSERT_TRUE(ApplyExcludeSelectedRaypath(g_state));

  nlohmann::json doc;
  ASSERT_NO_THROW(doc = nlohmann::json::parse(CoreJson(g_state)));
  const nlohmann::json& filters = doc["filter"];
  ASSERT_EQ(filters.size(), 3u) << filters.dump();
  EXPECT_EQ(filters[2]["type"], "complex");
  const nlohmann::json& scattering = doc["scene"]["scattering"];
  ASSERT_EQ(scattering.size(), 2u);
  EXPECT_EQ(scattering[0]["entries"][0]["filter"], filters[2]["id"]);
  EXPECT_EQ(scattering[1]["entries"][0]["filter"], filters[2]["id"]) << "the linked sibling sees the same slot";
  // Not a new pool slot: both entries still reference slot 0.
  ASSERT_EQ(g_state.filters.size(), 1u);
  EXPECT_EQ(*g_state.layers[0].entries[0].filter_id, 0);
  EXPECT_EQ(*g_state.layers[1].entries[0].filter_id, 0);
}

// An In filter on the crystal denies: the two do not compose into one filter, and nothing is
// written.
TEST(RaypathAnalysisFilterExcludeChain, InFilterOnTheCrystalRefusesAndLeavesTheExportAlone) {
  SeedUnfilteredPrismDocument();
  FilterConfig existing;
  existing.name = "keep 1-3";
  existing.action = 0;
  existing.param = FromLegacyRaypath(RaypathParams{ "1-3" });
  g_state.filters.assign(1, existing);
  g_state.layers[0].entries[0].filter_id = 0;
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult(1)));
  g_state.analysis.selected_entry = "3-5";
  const std::string before = CoreJson(g_state);
  std::string why;
  EXPECT_EQ(EvaluateExcludeEligibility(g_state, &why), ExcludeEligibility::kEntryHasInFilter);
  EXPECT_NE(why.find("In filter"), std::string::npos) << why;
  EXPECT_TRUE(ExcludeAppendNotice(g_state).empty());
  EXPECT_FALSE(ApplyExcludeSelectedRaypath(g_state));
  EXPECT_EQ(CoreJson(g_state), before);
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
