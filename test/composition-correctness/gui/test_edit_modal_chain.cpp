// Composition chain: the edit modal's crystal preview rebuild trigger.
//
// Units in the chain: crystal_preview × gui_state. (This chain used to include a second half, the
// commit gate — edit_modals × edit_modal_rules — but that moved to the unit layer in E17 below;
// edit_modals.cpp is a panel-rendering unit with no call surface a non-ImGui test can drive, so it
// was never invoked directly here even before that move. Keeping it in this "units in the chain"
// line after the move made the line describe a chain this file no longer tests.)
//
// What the collaboration produces that is observable: whether the 3D preview beside the edit modal
// is showing the crystal currently being edited. A preview that does not rebuild on every edit, or
// rebuilds on edits that do not touch the fields it draws, leaves the user looking at the wrong
// crystal.
//
// Derived from the src call graph: edit_modals.cpp -> crystal_preview is 8 call sites.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gui/crystal_preview.hpp"
#include "gui/gui_state.hpp"
#include "include/lumice.h"

namespace lumice::gui {
namespace {

// ---------------------------------------------------------------------------------------------
// E17 — the commit gate, the row/spectrum caps and the value clamps are single-unit rules of
// edit_modal_rules.hpp, and they are asserted over their whole domain one layer down, in
// test/unit-correctness/gui/test_gui_widget_rules.cpp (EditModalRules.*). They lived here as well
// until this wave; two layers stating the same proposition is the redundancy this rebuild was told
// to remove, and the unit layer is the one whose subject they are — it pins the tooltip's exact
// wording, which is what the gate and its message have to agree on.
//
// What is left in this file is the part no single unit can answer.

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
