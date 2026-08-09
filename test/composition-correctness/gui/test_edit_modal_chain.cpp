// Composition chain: the edit modal's commit gate and its crystal preview.
//
// Units in the chain: edit_modals × edit_modal_rules × crystal_preview × gui_state.
//
// What the collaboration produces that is observable: whether the OK button is live, what it says
// when it is not, and whether the 3D preview beside it is showing the crystal currently being
// edited. The two halves fail in opposite directions and both fail quietly. A commit gate that is
// stricter than the validator leaves the user with a button that does nothing and no sentence
// saying what is missing; a preview that does not rebuild leaves them editing one crystal while
// looking at another.
//
// Derived from the src call graph: edit_modals.cpp -> panels is 15 call sites, -> edit_modal_rules
// is 10 and -> crystal_preview is 8.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gui/crystal_preview.hpp"
#include "gui/edit_modal_rules.hpp"
#include "gui/gui_state.hpp"
#include "gui/raypath_segments.hpp"
#include "include/lumice.h"

namespace lumice::gui {
namespace {

// ---------------------------------------------------------------------------------------------
// E17 — the commit gate and the tooltip beside it are two consumers of ONE verdict.
//
// A disabled button whose tooltip disagrees with why it is disabled is worse than no tooltip: the
// user follows the sentence, fixes what it names, and the button stays dead.

TEST(EditModalChain, TheCommitGateAndItsTooltipAgreeOnEveryValidationState) {
  struct Case {
    LUMICE_RaypathValidationState state;
    const char* message;
    bool expect_blocked;
    // The phrasing the tooltip must and must not carry. "Still typing" is not an error, and saying
    // so is the difference between a state every half-typed raypath passes through and a state the
    // user thinks they have got wrong.
    const char* expect_phrase;
    const char* forbid_phrase;
  };
  const Case kCases[] = {
    // Blank rows validate as valid and are stripped at commit time; resolving "empty means no
    // filter" in this gate instead would make an untouched row un-committable.
    { LUMICE_RAYPATH_VALID, "", false, nullptr, nullptr },
    { LUMICE_RAYPATH_INCOMPLETE, "", true, "finish typing", "invalid" },
    { LUMICE_RAYPATH_INVALID, "Face 13 is not legal on this crystal type", true, nullptr, nullptr },
    // An invalid row whose validator produced no message still has to say something; the fallback
    // is what stops the tooltip from being "Row 3: ".
    { LUMICE_RAYPATH_INVALID, "", true, "invalid", nullptr },
  };

  for (const Case& c : kCases) {
    GuiValidationResult v;
    v.state = c.state;
    v.message = c.message;

    const bool blocked = SummandRowBlocksCommit(v.state);
    const std::string tooltip = SummandRowOkTooltip(/*row_index=*/2, v);

    EXPECT_EQ(blocked, c.expect_blocked) << "state " << static_cast<int>(c.state);
    // The tooltip is non-empty exactly when the row blocks. Either direction of disagreement is a
    // user-visible dead end: a silent disabled button, or a warning next to a button that works.
    EXPECT_EQ(!tooltip.empty(), blocked) << "state " << static_cast<int>(c.state) << " tooltip='" << tooltip << "'";
    if (blocked) {
      EXPECT_NE(tooltip.find("Row 3"), std::string::npos)
          << "the tooltip does not name the row it is about, so a user with 40 rows cannot act on it";
    }
    if (c.expect_phrase != nullptr) {
      EXPECT_NE(tooltip.find(c.expect_phrase), std::string::npos) << "got: " << tooltip;
    }
    if (c.forbid_phrase != nullptr) {
      EXPECT_EQ(tooltip.find(c.forbid_phrase), std::string::npos) << "got: " << tooltip;
    }
  }
}

// ---------------------------------------------------------------------------------------------
// E17 (row-count half) — the caps and the delete rule are boundary conditions, so they are checked
// at their boundaries rather than in the middle where any off-by-one still passes.

TEST(EditModalChain, TheRowCapsAndTheDeleteRuleHoldAtTheirBoundaries) {
  EXPECT_FALSE(AtSummandRowCap(kMaxSummandRows - 1));
  EXPECT_TRUE(AtSummandRowCap(kMaxSummandRows));
  EXPECT_TRUE(AtSummandRowCap(kMaxSummandRows + 1));
  // The cap is a UI budget, not the ABI limit, and it must stay above the pre-widening hard cap —
  // a change that quietly walked it back to 16 would undo the widening with nothing to notice.
  EXPECT_GT(kMaxSummandRows, 16u);

  // An empty list has no way back to being a filter, so the editor keeps one row the user can
  // blank instead. A delete button live on the last row is a one-way door out of the editor.
  EXPECT_FALSE(CanDeleteSummandRow(0));
  EXPECT_FALSE(CanDeleteSummandRow(1));
  EXPECT_TRUE(CanDeleteSummandRow(2));

  // The spectrum's own pair, and the invariant behind the first one: spectrum_index == custom
  // implies a non-empty buffer, so committing an empty one leaves a document that names a spectrum
  // with nothing in it.
  EXPECT_TRUE(SpectrumCommitBlocked(0));
  EXPECT_FALSE(SpectrumCommitBlocked(1));
  EXPECT_FALSE(AtSpectrumRowCap(kSpectrumHardMax - 1));
  EXPECT_TRUE(AtSpectrumRowCap(kSpectrumHardMax));
}

// ---------------------------------------------------------------------------------------------
// E17 (clamp half) — the seed for a newly added row and the sanitize pass on OK must clamp the same
// way, or "Add row" proposes a value the commit then silently moves.

TEST(EditModalChain, SpectrumClampsAreIdempotentAndCoverBothEnds) {
  const float kInputs[] = { -1e6f,  0.0f, 379.0f, kSpectrumWavelengthMinNm, 550.0f, kSpectrumWavelengthMaxNm,
                            781.0f, 1e6f };
  for (float nm : kInputs) {
    const float once = ClampSpectrumWavelengthNm(nm);
    EXPECT_GE(once, kSpectrumWavelengthMinNm) << "input " << nm;
    EXPECT_LE(once, kSpectrumWavelengthMaxNm) << "input " << nm;
    // Idempotence is what makes "seed then sanitize" a no-op for an untouched row. Without it a
    // row the user never edited would move on every OK.
    EXPECT_FLOAT_EQ(ClampSpectrumWavelengthNm(once), once) << "input " << nm;
  }

  // Weights are amplitudes. A negative one is not a dim row — it is a row that removes light from
  // the others, which is not a state the editor has any way to show.
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(-1.0f), 0.0f);
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(ClampSpectrumWeight(2.5f), 2.5f);
}

// ---------------------------------------------------------------------------------------------
// E19 — the crystal preview rebuilds when, and only when, the crystal it draws has changed.
//
// The hash is the whole mechanism: it decides both halves. Too insensitive and the user edits a
// parameter while looking at the previous shape; too sensitive and the mesh is rebuilt every frame.

TEST(EditModalChain, TheCrystalParamHashMovesOnEveryEditAndOnNothingElse) {
  CrystalConfig base;
  const int base_hash = CrystalParamHash(base);

  // Stable first: two default-constructed crystals, and the same object hashed twice — the
  // per-frame comparison the preview actually makes. Without this the "changes" half below would
  // be satisfied by a hash that moves every call.
  EXPECT_EQ(base_hash, CrystalParamHash(CrystalConfig{}));
  EXPECT_EQ(base_hash, CrystalParamHash(base));

  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    CrystalConfig edited = base;
    ShapeScalarAt(edited, slot).center += 0.25f;
    EXPECT_NE(CrystalParamHash(edited), base_hash)
        << "editing shape scalar " << slot << " leaves the preview showing the previous crystal";
  }

  // The crystal kind is part of it too, and it is not a shape scalar.
  CrystalConfig prism = base;
  prism.type = CrystalType::kPrism;
  CrystalConfig pyramid = prism;
  pyramid.type = CrystalType::kPyramid;
  EXPECT_NE(CrystalParamHash(prism), CrystalParamHash(pyramid))
      << "switching crystal type leaves the previous shape on screen";
}

// Shape randomization draws a new geometry per sample, so a crystal with it active cannot be cached
// on parameters alone: two consecutive frames of the same parameters are two different crystals.
//
// The predicate's rule is type != kNoRandom, and it is deliberately CONSERVATIVE — a uniform family
// with zero spread draws the same value every time and would be safe to cache, but it still reads
// as randomized here. Asserted as the contract rather than as the tighter thing it could be:
// over-triggering costs a mesh rebuild, under-triggering shows the user a stale crystal.
TEST(EditModalChain, ShapeRandomizationIsDecidedByDistributionFamilyNotBySpread) {
  CrystalConfig deterministic;
  EXPECT_FALSE(HasActiveShapeRandomization(deterministic));

  CrystalConfig randomized = deterministic;
  randomized.height = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.5f };
  EXPECT_TRUE(HasActiveShapeRandomization(randomized));

  CrystalConfig zero_spread = deterministic;
  zero_spread.height = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.0f };
  EXPECT_TRUE(HasActiveShapeRandomization(zero_spread))
      << "the rule is the family, not the spread; a zero-spread uniform is still not kNoRandom";

  // The half that makes this a composition claim rather than a field test: the predicate mirrors
  // the mesh builder's own type branch, so it must ignore exactly the fields that crystal kind does
  // not feed into the mesh. A prism whose pyramid-only heights carry a distribution has nothing
  // varying in its preview, and animating it would spin the mesh builder forever on a field it
  // ignores.
  CrystalConfig prism;
  prism.type = CrystalType::kPrism;
  prism.upper_h = ShapeDist{ ShapeDistType::kUniform, 0.2f, 0.1f };
  prism.lower_h = ShapeDist{ ShapeDistType::kUniform, 0.2f, 0.1f };
  prism.prism_h = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.1f };
  EXPECT_FALSE(HasActiveShapeRandomization(prism))
      << "a prism does not consume the pyramid height family, so nothing in its preview varies";

  CrystalConfig pyramid;
  pyramid.type = CrystalType::kPyramid;
  pyramid.height = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.5f };
  EXPECT_FALSE(HasActiveShapeRandomization(pyramid))
      << "a pyramid does not consume `height`, so randomizing it changes nothing on screen";

  // face_distance is consumed by both kinds, so it must animate either one.
  for (CrystalType kind : { CrystalType::kPrism, CrystalType::kPyramid }) {
    CrystalConfig c;
    c.type = kind;
    c.face_distance[3] = ShapeDist{ ShapeDistType::kUniform, 1.0f, 0.2f };
    EXPECT_TRUE(HasActiveShapeRandomization(c)) << "kind " << static_cast<int>(kind);
  }
}

}  // namespace
}  // namespace lumice::gui
