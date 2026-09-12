// Composition chain: the modal that tells the user their configuration did not fit.
//
// Units in the chain: app × gui_state × panels (and the server on the far side of the commit).
//
// What the collaboration produces that is observable: a configuration the engine cannot take is
// reported once, and reported again when the user asks again. Two different limits produce it —
// a filter with more clauses than the wire format holds, which makes the commit fail outright, and
// a colour configuration with more distinct predicates than the engine has bits for, which commits
// successfully and quietly drops the excess. They arrive at the same modal by different branches.
//
// The timing is the whole difficulty. The app re-commits roughly every 70 ms while the user drags a
// slider, so a persistent overflow is re-detected on every tick. Announcing it each time re-opens
// the modal continuously and the window stops responding — that regression is the reason the
// de-duplication exists. Announcing it only once has the opposite failure: the user reads the
// message, dismisses it, presses Run deliberately, and nothing happens at all.
//
// So the rule has two halves that pull against each other, and both are pinned here per trigger.
// They resolve the same way on both branches: which limit was hit decides what the message says,
// not whether the modal comes back.

#include <gtest/gtest.h>

#include <string>

#include "gui/app.hpp"
#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"
#include "gui/panels.hpp"
#include "lumice.h"

namespace lumice::gui {
namespace {

// A colour configuration with more distinct predicates than the engine has bits for.
//
// Three classes of twenty-two refs: the per-class ref cap and the class cap both allow this, so the
// commit is accepted and the loss happens further down, where the engine de-duplicates predicates
// across classes and runs out of bits. Every predicate is spelled distinctly (a grid of two-face
// paths, then three-face paths past the end of the grid) because structurally equal predicates
// would be merged and the overflow would never be reached.
constexpr int kBitBudget = 64;  // ComponentTable::kMaxBits
constexpr int kColourClasses = 3;
constexpr int kRefsPerClass = 22;
static_assert(kColourClasses * kRefsPerClass > kBitBudget, "the setup must actually overflow");

void SeedColourPredicateOverflow() {
  g_state.raypath_color.clear();
  int uid = 0;
  for (int c = 0; c < kColourClasses; ++c) {
    ColorClassConfig cls;
    cls.color[0] = 1.0f - static_cast<float>(c) * 0.2f;
    cls.color[1] = 0.5f;
    cls.color[2] = static_cast<float>(c) * 0.2f;
    for (int k = 0; k < kRefsPerClass; ++k, ++uid) {
      ColorClassRefConfig ref;
      ref.layer_idx = 0;
      ref.crystal_pool_id = 0;
      ref.match_all = false;
      if (uid < kBitBudget) {
        ref.predicate_text = std::to_string(1 + uid % 8) + "-" + std::to_string(1 + uid / 8);
      } else {
        ref.predicate_text = "1-1-" + std::to_string(uid - kBitBudget + 1);
      }
      cls.match.push_back(ref);
    }
    g_state.raypath_color.push_back(cls);
  }
  g_state.sim.infinite = false;
  g_state.sim.ray_num_millions = 0.001f;  // finishes immediately if it does start
}

// A filter with more clauses than the wire format holds: four factors of nine alternatives each,
// 9^4 = 6561 against a bound of 4096. The same recipe the commit-path tests use, so there is one
// source of truth for "what an over-large filter looks like".
void SeedClauseOverflowFilter() {
  const std::string alt = "1;2;3;4;5;6;7;8;3-4";
  SummandText row;
  row.text = alt + " & " + alt + " & " + alt + " & " + alt;
  row.factors = { Factor{ RaypathParams{ alt } }, Factor{ RaypathParams{ alt } }, Factor{ RaypathParams{ alt } },
                  Factor{ RaypathParams{ alt } } };
  FilterConfig f;
  f.name = "OverflowFilter";
  f.param = SumOfProducts{ row };
  g_state.filters.push_back(f);
  SetFilter(g_state, g_state.layers.at(0).entries.at(0), g_state.filters.back());
}

// The app's server global, for the duration of one case. Held by an object because several of the
// assertions below are EXPECT rather than ASSERT, so a failure keeps running to the end of the case
// and a trailing teardown would be skipped exactly when the state is worst.
class ScopedAppServer {
 public:
  ScopedAppServer() {
    DoNew();
    ClearGuiWarning();
    g_server = LUMICE_CreateServer();
  }
  ~ScopedAppServer() {
    ClearGuiWarning();
    g_state.raypath_color.clear();
    g_server_poller.Stop();
    if (g_server != nullptr) {
      LUMICE_StopServer(g_server);
      LUMICE_DestroyServer(g_server);
      g_server = nullptr;
    }
    g_state.run_intent = RunIntent::kNone;
    g_state.committed_epoch = 0;
    g_state.dirty = false;
  }
  ScopedAppServer(const ScopedAppServer&) = delete;
  ScopedAppServer& operator=(const ScopedAppServer&) = delete;

  bool ok() const { return g_server != nullptr; }
};

}  // namespace

// The two halves of the rule, over both triggers. `user_initiated` is the only thing that differs
// between the rows of each pair, which is what makes the pair a statement about the trigger rather
// than about the overflow.
TEST(RunWarningChain, ARepeatedOverflowReopensOnlyWhenTheUserAsksAgain) {
  struct Case {
    const char* name;
    void (*seed)();
    bool user_initiated;
    bool expect_second_run_reopens;
  };
  const Case kCases[] = {
    { "a rejected filter, re-committed automatically", SeedClauseOverflowFilter, false, false },
    { "a rejected filter, Run pressed again", SeedClauseOverflowFilter, true, true },
    { "a degraded colour set, re-committed automatically", SeedColourPredicateOverflow, false, false },
    // This row used to be pinned at `false`, on the reading that the degradation branch skips the
    // clear-then-set that the rejection branch performs. It does not: both branches end in the
    // same two statements, `if (user_initiated) ClearGuiWarning();` followed by
    // `SetGuiWarning(msg)`. What actually differed was upstream of them — the rejection branch
    // returns before LUMICE_CommitScene, so no simulation ever starts and DoRun's backpressure
    // gate is never armed, while the degradation branch commits and leaves a Run in flight. The
    // second DoRun then met a gate that swallowed the whole commit, and whether it did was a race
    // against a background poller consuming the first batch, so the row was pinning a coincidence
    // of scheduling and not a property of either branch.
    //
    // The gate now applies to the auto-commit path only (see DoRun in app.cpp), which is what
    // makes this row decidable at all: a deliberate Run reaches its branch whatever the running
    // simulation is doing. Both branches then reopen, which is the half of the rule this file's
    // header calls "the user reads the message, dismisses it, presses Run, and nothing happens".
    { "a degraded colour set, Run pressed again", SeedColourPredicateOverflow, true, true },
  };

  for (const Case& c : kCases) {
    SCOPED_TRACE(c.name);
    ScopedAppServer server;
    if (!server.ok()) {
      ADD_FAILURE() << c.name << ": failed to create the app server";
      continue;  // no server to run against for this row; the rest still get checked
    }
    c.seed();

    DoRun(c.user_initiated);
    const std::string first = PeekGuiWarning();
    EXPECT_FALSE(first.empty()) << "the overflow was not reported at all";
    EXPECT_TRUE(IsGuiWarningPending()) << "the first detection did not open the modal";

    // What a rendered frame does: it opens the popup and clears the trigger, leaving the message
    // itself in flight. Driving a real frame instead would put the harness in a fight with the
    // modal's input capture over an invariant that lives entirely in this flag.
    internal_test::ConsumeGuiWarningPending();
    if (IsGuiWarningPending()) {
      ADD_FAILURE() << c.name << ": consuming the pending warning did not clear it";
      continue;  // the second-run premise is broken for this row; the rest still get checked
    }

    DoRun(c.user_initiated);
    EXPECT_EQ(PeekGuiWarning(), first) << "the same condition produced a different message";
    EXPECT_EQ(IsGuiWarningPending(), c.expect_second_run_reopens)
        << (c.expect_second_run_reopens ? "a deliberate Run was swallowed by the de-duplication" :
                                          "the second commit re-opened the modal; under a slider drag that is a "
                                          "window that stops responding");
  }
}

// The colour overflow's own contract, which the shared case above deliberately does not touch: the
// commit SUCCEEDS and the loss is a display-layer degradation. Two things follow, and both matter.
//
// The message must be distinguishable from the two rejection messages, or the de-duplication —
// which compares messages, not conditions — would treat a degradation arriving after a rejection as
// "already reported" and never show it.
//
// And the number of dropped predicates is asserted exactly rather than as "some": 66 distinct
// predicates against 64 bits is 2, and a report that said "overflow" without the count would pass
// just as well on an implementation that dropped all 66.
TEST(RunWarningChain, ADegradedColourSetCommitsAndReportsHowMuchItLost) {
  ScopedAppServer server;
  ASSERT_TRUE(server.ok());
  SeedColourPredicateOverflow();

  DoRun(/*user_initiated=*/true);

  const std::string warning = PeekGuiWarning();
  EXPECT_NE(warning.find("This raypath color configuration exceeds its predicate"), std::string::npos)
      << "got: " << warning;
  EXPECT_NE(warning.find("Simplify the color configuration"), std::string::npos) << "got: " << warning;

  LUMICE_ColorOverflowInfo info{};
  ASSERT_EQ(LUMICE_GetColorOverflowInfo(g_server, &info), LUMICE_OK);
  EXPECT_EQ(info.component_overflow_count, kColourClasses * kRefsPerClass - kBitBudget);
}

// The refusal is the emitter's, not the button's: Run and Analyze hand the same document to the
// same BuildCommitSceneOrWarn, so a document too large for the ABI is refused by both, with one
// message, naming the filter reference that tripped the bound — and neither submits anything.
// The locator is asserted, not just "some warning": a warning that named nothing would pass a
// weaker check just as well, and the locator is what tells the user which filter to simplify.
// This is the observable half of "one emitter": a second BuildScene call in DoAnalyze that
// skipped the overflow report would either analyse a truncated document or refuse it silently.
TEST(RunWarningChain, RunAndAnalyzeRefuseAnOverflowingDocumentTheSameWay) {
  ScopedAppServer server;
  ASSERT_TRUE(server.ok());
  SeedClauseOverflowFilter();

  // The locator the emitter is expected to name, from the same BuildScene call it makes.
  FilterOverflowInfo overflow;
  ASSERT_EQ(BuildScene(g_state, SceneIntent::kSimCommit, &overflow, nullptr), nullptr) << "positive control";
  const std::string locator = FormatOverflowLocator(overflow);
  ASSERT_FALSE(locator.empty());

  DoRun(/*user_initiated=*/true);
  const std::string run_warning = PeekGuiWarning();
  EXPECT_NE(run_warning.find(locator), std::string::npos) << "Run's warning does not name the filter: " << run_warning;
  EXPECT_EQ(g_state.run_intent, RunIntent::kNone) << "a refused Run commits nothing";
  internal_test::ConsumeGuiWarningPending();
  ClearGuiWarning();

  g_state.analysis.started = false;  // the assertion below is on THIS call's doing
  EXPECT_FALSE(DoAnalyze()) << "a document the emitter refuses is not analysed";
  EXPECT_EQ(PeekGuiWarning(), run_warning) << "Analyze's refusal is Run's, word for word";
  EXPECT_TRUE(IsGuiWarningPending()) << "an Analyze is a deliberate click: the modal opens";
  EXPECT_FALSE(g_state.analysis.started) << "no analysis intent was raised";
  LUMICE_SimLifecycleResult lc{};
  ASSERT_EQ(LUMICE_GetSimLifecycle(g_server, &lc), LUMICE_OK);
  EXPECT_EQ(lc.epoch, 0u) << "nothing reached the server from either button";
}

// The edit flag itself: something the user changed marks the document, and starting a new one
// clears it. It is one line of state, but it is the input the whole warning path above hangs off —
// a flag that never clears means every new document opens claiming unsaved changes.
TEST(RunWarningChain, AnEditMarksTheDocumentAndANewDocumentClearsIt) {
  DoNew();
  ASSERT_FALSE(g_state.dirty);
  g_state.MarkDirty();
  EXPECT_TRUE(g_state.dirty);
  DoNew();
  EXPECT_FALSE(g_state.dirty);
}

}  // namespace lumice::gui
