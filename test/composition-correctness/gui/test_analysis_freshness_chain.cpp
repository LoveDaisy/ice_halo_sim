// Composition chain: the Raypath Analysis list's freshness banner across the two lifecycle paths
// that make a list go stale.
//
// Units in the chain: app (DoAnalyze / DoRun / DoRevert, with a live server on the far side) ×
// gui_state (AnalysisSceneIdentity, ConfigSnapshot) × sim_state_rules (the predicate the banner
// reads) × analysis_panel (payload adoption).
//
// What the collaboration produces that is observable: a list captured from one scene is called
// stale once the document on the panels is a different scene, whichever way it got there — a Run
// that committed an edit made after Analyze, or a Revert that took the document back behind an
// Analyze made while dirty. The second path is the one nothing used to detect: Revert clears
// dirty, so every "is the picture of this document" line falls silent exactly when the list is
// wrong. Both paths are asserted through the real operations rather than by writing the
// comparison's inputs by hand, because the proposition is that those operations reach the
// comparison at all — Revert restores the snapshot AND keeps the result, Run keeps the result AND
// moves the committed scene — not that the comparison works on inputs it was handed.
//
// The analysis run itself is real (LUMICE_StartRaypathAnalysis on the server), but its result is
// adopted by hand: no frame runs here, so nothing polls the server for the payload. What DoAnalyze
// does synchronously — capture the scene identity and clear the previous view — is what the
// chain depends on, and that is what the real call exercises.

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "gui/analysis_panel.hpp"
#include "gui/analysis_result.hpp"
#include "gui/app.hpp"
#include "gui/gui_state.hpp"
#include "gui/sim_state_rules.hpp"
#include "lumice.h"

namespace lumice::gui {
namespace {

// The app's server global, for the duration of one case (the shape test_run_warning_chain.cpp
// uses, plus the analysis view: a result left behind here would be the next case's "fresh" list).
class ScopedAppServer {
 public:
  ScopedAppServer() {
    DoNew();
    ClearGuiWarning();
    g_server = LUMICE_CreateServer();
  }
  ~ScopedAppServer() {
    ClearGuiWarning();
    g_server_poller.Stop();
    if (g_server != nullptr) {
      LUMICE_StopServer(g_server);
      LUMICE_DestroyServer(g_server);
      g_server = nullptr;
    }
    g_state.run_intent = RunIntent::kNone;
    g_state.committed_epoch = 0;
    g_state.dirty = false;
    g_state.analysis = GuiState::RaypathAnalysisSession{};
    g_state.analysis_result = GuiState::AnalysisResultView{};
    g_state.analysis_run_in_progress = false;
  }
  ScopedAppServer(const ScopedAppServer&) = delete;
  ScopedAppServer& operator=(const ScopedAppServer&) = delete;

  bool ok() const { return g_server != nullptr; }
};

// One prism, one layer, and budgets small enough that neither the render nor the analysis the
// chain starts outlives the case. The analysis budget is the session's own (not the document's),
// so it is set here explicitly rather than seeded by a panel frame that never runs.
void SeedOnePrismDocument() {
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
  g_state.sim.infinite = false;
  g_state.sim.ray_num_millions = 0.001f;
  g_state.analysis.infinite = false;
  g_state.analysis.ray_num_millions = 0.001f;
  g_state.analysis.ray_budget_initialized = true;
}

// A minimal adopted result: one chain, generation 1. What the freshness question needs from the
// payload is only that there is one.
std::shared_ptr<AnalysisPayload> OneChainResult() {
  auto p = std::make_shared<AnalysisPayload>();
  p->snapshot_generation = 1;
  p->roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  LUMICE_RaypathHistogramEntry e{};
  e.chain_len = 1;
  e.chain[0].crystal_id = 0;
  e.chain[0].segment_len = 2;
  e.chain[0].segment[0] = 3;
  e.chain[0].segment[1] = 5;
  snprintf(e.display, sizeof(e.display), "3-5");
  e.energy = 1.0;
  e.count = 1000;
  p->entries.push_back(e);
  return p;
}

// The server refuses an analysis while a render is in flight (the same mutual exclusion the
// panel surfaces as a disabled Analyze button), so the chain waits for the finite run DoRun
// started to complete before pressing Analyze — the ordering a user's click would have. Bounded:
// a run of 1000 rays completes in milliseconds, and a run that does not is a failure with a
// message rather than a hang.
bool WaitForRenderToFinish() {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < deadline) {
    LUMICE_SimLifecycleResult lc{};
    LUMICE_GetSimLifecycle(g_server, &lc);
    if (lc.lifecycle != LUMICE_LIFECYCLE_RUNNING) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

// The banner's inputs, computed the way the banner computes them.
AnalysisListFreshness ListFreshness(const GuiState& state) {
  const bool has_result = state.analysis_result.payload != nullptr;
  const bool scene_still_matches =
      has_result && state.analysis_result.analyzed_scene && state.analysis_result.analyzed_scene->Matches(state);
  return ComputeAnalysisListFreshness(has_result, scene_still_matches);
}

// An edit to the scene proper — a crystal dimension — made by assignment, the way an edit modal's
// commit lands it, with the dirty flag the modal would set.
void EditCrystalHeight(float height) {
  g_state.crystals[0].height = height;
  g_state.dirty = true;
}

}  // namespace

// Analyze while dirty, then Revert. The list describes the edited scene; Revert takes the document
// back to the committed one and clears dirty, and the list is still there — so the only thing
// left that can say "this list is not of this document" is the scene comparison. The kFresh
// assertion before the Revert is the positive control: a predicate that answered kStale to
// everything would pass the final assertion alone.
TEST(AnalysisFreshnessChain, AnalyzeWhileDirtyThenRevertMarksTheListStale) {
  ScopedAppServer server;
  ASSERT_TRUE(server.ok());
  SeedOnePrismDocument();

  ASSERT_TRUE(DoRun(/*user_initiated=*/true));
  ASSERT_TRUE(g_state.last_committed_state.has_value()) << "the Run did not establish a commit baseline";
  ASSERT_TRUE(WaitForRenderToFinish()) << "the finite render did not complete";
  EXPECT_EQ(ListFreshness(g_state), AnalysisListFreshness::kNone) << "a list before any Analyze";

  EditCrystalHeight(2.0f);
  ASSERT_TRUE(DoAnalyze());
  ASSERT_TRUE(g_state.analysis_result.analyzed_scene.has_value()) << "DoAnalyze did not record the scene";
  EXPECT_EQ(ListFreshness(g_state), AnalysisListFreshness::kNone) << "a list before the result arrived";
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult()));
  EXPECT_EQ(ListFreshness(g_state), AnalysisListFreshness::kFresh) << "the list is of the scene on the panels";

  DoRevert();
  ASSERT_FLOAT_EQ(g_state.crystals[0].height.center, 1.0f) << "Revert did not restore the committed crystal";
  EXPECT_FALSE(g_state.dirty) << "Revert left the document dirty";
  ASSERT_NE(g_state.analysis_result.payload, nullptr) << "Revert cleared the list; the exclude path needs it kept";
  EXPECT_EQ(ListFreshness(g_state), AnalysisListFreshness::kStale)
      << "the list is of the edited scene, the document is the committed one, and nothing said so";
}

// Analyze on a clean document, edit, Run. The Run commits the edit and — by design — keeps the
// list; the list is now of the previous commit. Same positive control before the edit.
TEST(AnalysisFreshnessChain, AnalyzeThenEditThenRunMarksTheListStale) {
  ScopedAppServer server;
  ASSERT_TRUE(server.ok());
  SeedOnePrismDocument();

  ASSERT_TRUE(DoRun(/*user_initiated=*/true));
  ASSERT_TRUE(WaitForRenderToFinish()) << "the finite render did not complete";
  ASSERT_TRUE(DoAnalyze());
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult()));
  EXPECT_EQ(ListFreshness(g_state), AnalysisListFreshness::kFresh) << "the list is of the scene just committed";

  EditCrystalHeight(2.0f);
  ASSERT_TRUE(DoRun(/*user_initiated=*/true));
  ASSERT_NE(g_state.analysis_result.payload, nullptr) << "Run cleared the list; it is meant to keep it";
  EXPECT_EQ(ListFreshness(g_state), AnalysisListFreshness::kStale)
      << "the list is of the previous commit, the document is the new one, and nothing said so";
}

// The identity is the scene, not the document: a colour-class edit classifies raypaths that were
// already traced and changes nothing the list counts, so it must not read as stale — the same
// boundary the commit's own overflow warning draws ("only color assignment degrades"). Pinned
// here so widening the identity to ConfigSnapshot, which carries raypath_color, is a deliberate
// change rather than a convenient one.
TEST(AnalysisFreshnessChain, AColourClassEditLeavesTheListFresh) {
  ScopedAppServer server;
  ASSERT_TRUE(server.ok());
  SeedOnePrismDocument();

  ASSERT_TRUE(DoRun(/*user_initiated=*/true));
  ASSERT_TRUE(WaitForRenderToFinish()) << "the finite render did not complete";
  ASSERT_TRUE(DoAnalyze());
  ASSERT_TRUE(AdoptAnalysisPayloadIfNew(g_state, OneChainResult()));
  ASSERT_EQ(ListFreshness(g_state), AnalysisListFreshness::kFresh);

  ColorClassConfig cls;
  ColorClassRefConfig ref;
  ref.layer_idx = 0;
  ref.crystal_pool_id = 0;
  ref.match_all = true;
  cls.match.push_back(ref);
  g_state.raypath_color.push_back(cls);
  g_state.dirty = true;
  EXPECT_EQ(ListFreshness(g_state), AnalysisListFreshness::kFresh) << "a colour edit does not change the raypaths";
  g_state.raypath_color.clear();
}

}  // namespace lumice::gui
