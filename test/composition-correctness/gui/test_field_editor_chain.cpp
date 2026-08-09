// Composition chain: a numeric field's domain, from the registry to the control that draws it.
//
// Units in the chain: app_panels × field_editor_registry × panels × gui_state.
//
// What the collaboration produces that is observable: what a control lets the user enter, and where
// the value lands when they do. The registry owns the domain; the panel's slider consumes it; the
// scalar accessor decides which field the number reaches. A break anywhere along that line has the
// same shape of consequence — the user is allowed to build a document the engine will refuse, and
// finds out when they press Run, with nothing pointing at which control did it.
//
// Derived from the src call graph: app_panels.cpp calls ConstraintFor 13 times and SliderWithInput
// 11 times, and panels.cpp reaches gui_state through ShapeScalarAt 10 times. Those three counts are
// one story, not three.

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "gui/field_editor_registry.hpp"
#include "gui/gui_state.hpp"
#include "gui/shape_scalar_domain.hpp"

namespace lumice::gui {
namespace {

// ---------------------------------------------------------------------------------------------
// E14 — every registered field answers with a usable domain, and the answer is total.
//
// The registry is the single owner of "what may this field hold". A control drawn from an entry
// that reports a degenerate or inverted interval is a control the user cannot use correctly, and
// nothing about it looks broken on screen: a slider with min == max simply refuses to move.

TEST(FieldEditorChain, EveryRegisteredFieldHasACoherentDomainAndAnEditorToDrawIt) {
  const GuiState state;
  const std::vector<std::string> keys = RegisteredFieldEditorKeyPaths();
  ASSERT_FALSE(keys.empty()) << "the registry is empty, so every case below would pass vacuously";

  for (const std::string& key : keys) {
    // Every registered key must have an editor to draw it. The two lookups are separate functions
    // on purpose — one is total and answers "does this leaf happen to have an editor", the other
    // aborts on a miss because its caller ships a control for that exact field — but a key that
    // appears in the enumeration and has no editor is a row the panel will list and be unable to
    // render.
    EXPECT_NE(FindFieldEditor(key), nullptr) << key << " is enumerated but has no editor entry";

    const FieldEditorConstraint c = ConstraintFor(key, state);
    if (!c.has_numeric_domain) {
      // Booleans, colours and combos have no interval, and reporting one would invite range checks
      // that can only produce false alarms. Their bounds are meaningless, so nothing is asserted.
      continue;
    }
    EXPECT_LT(c.min_value, c.max_value) << key << ": a slider over this domain cannot move";
    EXPECT_NE(c.fmt, nullptr) << key << ": a numeric field with no display format";
    EXPECT_NE(std::string(c.fmt).find('%'), std::string::npos)
        << key << ": display format '" << c.fmt << "' is not a format string";
  }

  // The nullptr answer is designed, not a lookup failure, so it has to actually happen for
  // something. If FindFieldEditor never returned nullptr the panel's "read-only here" path would be
  // dead code that no longer describes anything.
  EXPECT_EQ(FindFieldEditor("this.key.is.not.registered"), nullptr);
}

// ---------------------------------------------------------------------------------------------
// E14 (the disabled half) — "cannot be edited here at all" and "cannot be edited in THIS
// configuration" are different answers and must stay distinguishable.
//
// A field that does not currently apply (a full-sky lens has no roll) is greyed with a reason on
// hover. A disabled control with no reason is indistinguishable from a bug to the person looking
// at it.
TEST(FieldEditorChain, ADisabledFieldAlwaysCarriesAReason) {
  const GuiState state;
  int disabled_seen = 0;
  for (const std::string& key : RegisteredFieldEditorKeyPaths()) {
    const FieldEditorConstraint c = ConstraintFor(key, state);
    if (c.enabled) {
      continue;
    }
    ++disabled_seen;
    EXPECT_NE(c.disabled_reason, nullptr) << key << " is greyed out with nothing to say why";
    if (c.disabled_reason != nullptr) {
      EXPECT_FALSE(std::string(c.disabled_reason).empty()) << key << " is greyed out with an empty reason";
    }
  }
  // Not an assertion that any given key is disabled in the default document — only a record of
  // whether the loop above examined anything, so a future registry change that leaves nothing
  // disabled cannot turn this into a silently vacuous case.
  if (disabled_seen == 0) {
    GTEST_SKIP() << "no registered field is disabled in the default document; nothing to check";
  }
}

// ---------------------------------------------------------------------------------------------
// E15 — the scalar accessor reaches the field its label names.
//
// ShapeScalarAt is the single mapping authority between a slot index and a member, in both
// directions and at both call sites. Getting it wrong swaps two pyramid heights that then move
// together — a document that round-trips perfectly and simulates the wrong crystal, which the user
// reads as "these controls are linked" rather than as a defect.

TEST(FieldEditorChain, EveryShapeScalarSlotReachesADistinctFieldInBothDirections) {
  CrystalConfig crystal;
  // Stamp each slot with its own index through the mutable accessor, then read every slot back
  // through the const one. Any two slots aliasing the same member show up as a repeated value.
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    ShapeScalarAt(crystal, slot).center = static_cast<float>(100 + slot);
  }
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    EXPECT_FLOAT_EQ(ShapeScalarAt(static_cast<const CrystalConfig&>(crystal), slot).center,
                    static_cast<float>(100 + slot))
        << "slot " << slot << " does not read back what was written to it, so two slots share a member";
  }

  // The face-distance slots are six contiguous entries at the end of the index space, and the
  // header asserts that statically. Asserting it dynamically as well is what catches a mapping that
  // compiles (the static_assert is about the constants) but routes an index to the wrong element.
  for (int i = 0; i < 6; ++i) {
    crystal.face_distance[i].center = static_cast<float>(10 + i);
  }
  for (int i = 0; i < 6; ++i) {
    EXPECT_FLOAT_EQ(ShapeScalarAt(static_cast<const CrystalConfig&>(crystal), LUMICE_SHAPE_SCALAR_FACE_0 + i).center,
                    static_cast<float>(10 + i))
        << "face slot " << i << " does not reach face_distance[" << i << "]";
  }

  // The sync-group array has its own copy of the same mapping, in both directions. If the two
  // disagree the preview and the committed scene draw different crystals.
  int written[LUMICE_SHAPE_SCALAR_COUNT] = {};
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    written[slot] = slot + 1;
  }
  ApplySyncGroupArray(written, crystal);
  int read_back[LUMICE_SHAPE_SCALAR_COUNT] = {};
  FillSyncGroupArray(crystal, read_back);
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    EXPECT_EQ(read_back[slot], written[slot]) << "sync-group slot " << slot;
  }
}

// ---------------------------------------------------------------------------------------------
// E14 (the shape-table half) — the shape scalars' domains are a table, and every slot the mapping
// authority indexes has a row in it.
//
// This table used to be prose in a header comment with the numbers hand-written at each of the ten
// call sites. A comment cannot be checked against its call sites, which is why the numbers were
// free to drift. Enumerating the table is the cheapest thing that notices.

TEST(FieldEditorChain, EveryShapeScalarSlotHasAUsableDomainAndAScaleThatCanReachIt) {
  for (int slot = 0; slot < LUMICE_SHAPE_SCALAR_COUNT; ++slot) {
    const ShapeScalarDomain& domain = ShapeScalarDomainFor(slot);
    EXPECT_LT(domain.min_value, domain.max_value) << "slot " << slot << " has an unusable domain";
    EXPECT_NE(domain.fmt, nullptr) << "slot " << slot;
    EXPECT_NE(std::string(domain.fmt).find('%'), std::string::npos)
        << "slot " << slot << ": display format '" << domain.fmt << "' is not a format string";
    // The domain and the scale that traverses it are one answer, not two, and they can contradict
    // each other: a logarithmic slider cannot reach zero, so a slot whose minimum IS zero and whose
    // scale is pure log offers a bound the control can never produce. prism_h is the concrete case
    // — it must allow exactly 0 (a degenerate zero-height prism section) AND stay finely
    // controllable near 0 over a wide range, which is why it is the one slot on the log-linear
    // scale rather than the log one.
    if (domain.min_value == 0.0f) {
      EXPECT_NE(domain.scale, SliderScale::kLog)
          << "slot " << slot << " advertises a minimum of 0 on a logarithmic scale, so the bound it "
          << "shows the user is one the control cannot reach";
    }
  }

  // The six face slots share one band today, and that is a deliberate reflection of what the call
  // site does rather than an accident of the table. Pinning it keeps a future per-face domain an
  // intentional data edit instead of something that drifts in unnoticed.
  const ShapeScalarDomain& first = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_FACE_0);
  for (int i = 1; i < 6; ++i) {
    const ShapeScalarDomain& other = ShapeScalarDomainFor(LUMICE_SHAPE_SCALAR_FACE_0 + i);
    EXPECT_FLOAT_EQ(other.min_value, first.min_value) << "face slot " << i;
    EXPECT_FLOAT_EQ(other.max_value, first.max_value) << "face slot " << i;
    EXPECT_STREQ(other.fmt, first.fmt) << "face slot " << i;
  }
}

}  // namespace
}  // namespace lumice::gui
