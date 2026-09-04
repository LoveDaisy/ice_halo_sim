// Opening a document written before the reference points existed.
//
// What this is for. The six markers replaced a single zenith/nadir PAIR whose four `.lmc` keys
// (`overlay_zenith_nadir_line` / `_color` / `_alpha` / `_radius_px`) are in every document saved
// before this change. The loader still reads them, and this file is what says so — a claim no other
// test in the tree makes, because every other document test round-trips through the CURRENT
// serializer and so never produces a document in the old shape at all.
//
// The mapping is per marker, not per document. The loader does not decide "this is an old file" and
// switch wholesale; it asks, for each marker, whether THIS document carries that marker's own new
// key, and only falls back to the legacy pair when it does not. The finer grain is what makes a
// hand-edited or half-migrated document behave, and the cases below are written against that grain
// rather than against the two extremes.
//
// Deliberately NOT here: the round trip of the new keys (composition-correctness/gui/
// test_document_roundtrip_chain.cpp probes two of them), and which keys the serializer emits at all
// (unit-correctness/gui/test_defaults_diff.cpp enumerates them against a real factory document).
// This file is only about documents the current serializer would never write.
//
// What a user sees when this breaks: a saved document opens with its zenith marker off, or in the
// wrong colour, or at a radius they never chose.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>

#include "gui/file_io.hpp"
#include "gui/gui_state.hpp"

namespace lumice::gui {
namespace {

// A document in the OLD shape: the four legacy keys, and none of the twenty-one the current
// serializer writes. Built as a JSON object rather than as a captured file so the case states
// exactly which keys it is claiming about — a captured .lmc would also carry a hundred keys this
// test has no opinion on, and one of them changing would be indistinguishable from a real break.
std::string LegacyDocument(bool line, float r, float g, float b, float alpha, float radius) {
  nlohmann::json root;
  root["overlay_zenith_nadir_line"] = line;
  root["overlay_zenith_nadir_color"] = { r, g, b };
  root["overlay_zenith_nadir_alpha"] = alpha;
  root["overlay_zenith_nadir_radius_px"] = radius;
  return root.dump();
}

TEST(MarkerDocumentCompat, LegacyKeysStillDriveTheZenithAndNadirRows) {
  // Values chosen to be nothing's default: the factory zenith is red-ish at 0.6 alpha and 8 px, so a
  // loader that silently ignored the legacy keys would leave those behind and this case would read
  // them rather than the document's.
  GuiState state;
  ASSERT_TRUE(DeserializeGuiStateJson(LegacyDocument(true, 0.1f, 0.7f, 0.4f, 0.35f, 13.0f), state));

  // ONE switch and ONE colour covered both directions in the old schema, which is why the same two
  // values land in both rows. That is the migration, stated: a user who had the pair on gets both
  // of its members on, not one.
  for (int id : { LUMICE_ANNOTATION_MARKER_ZENITH, LUMICE_ANNOTATION_MARKER_NADIR }) {
    EXPECT_TRUE(state.markers[id].show) << "marker id " << id;
    EXPECT_FLOAT_EQ(state.markers[id].color[0], 0.1f) << "marker id " << id;
    EXPECT_FLOAT_EQ(state.markers[id].color[1], 0.7f) << "marker id " << id;
    EXPECT_FLOAT_EQ(state.markers[id].color[2], 0.4f) << "marker id " << id;
    // The old marker drew no text at all, so there is no legacy source for this and it must be the
    // factory value. A loader that defaulted it to the line switch would turn on six names nobody
    // asked for the first time an old document was opened.
    EXPECT_FALSE(state.markers[id].label) << "marker id " << id;
  }
  EXPECT_FLOAT_EQ(state.markers_alpha, 0.35f);
  EXPECT_FLOAT_EQ(state.markers_radius_px, 13.0f);
}

TEST(MarkerDocumentCompat, TheLegacyPairSpeaksForNobodyElse) {
  // The four sun-relative markers had no representation in the old schema. A loader that applied the
  // legacy switch family-wide would turn on four rings a document from before they existed cannot
  // have asked for — the most likely shape of a "just map the old key onto the new field" mistake.
  GuiState state;
  ASSERT_TRUE(DeserializeGuiStateJson(LegacyDocument(true, 0.1f, 0.7f, 0.4f, 0.35f, 13.0f), state));

  const GuiState factory;
  for (int id : { LUMICE_ANNOTATION_MARKER_SUN, LUMICE_ANNOTATION_MARKER_SUBSUN, LUMICE_ANNOTATION_MARKER_ANTHELION,
                  LUMICE_ANNOTATION_MARKER_ANTISOLAR }) {
    EXPECT_FALSE(state.markers[id].show) << "marker id " << id;
    EXPECT_FALSE(state.markers[id].label) << "marker id " << id;
    for (int c = 0; c < 3; ++c) {
      EXPECT_FLOAT_EQ(state.markers[id].color[c], factory.markers[id].color[c]) << "marker id " << id << " ch " << c;
    }
  }
}

TEST(MarkerDocumentCompat, ANewKeyWinsOverTheLegacyOneForTheMarkerItNames) {
  // A document carrying both — which a hand edit produces, and which a half-migrated file would.
  // Per marker: the zenith names its own key and takes it; the nadir does not and falls back. The
  // two rows therefore end up DIFFERENT, from a schema in which they could not be.
  nlohmann::json root = nlohmann::json::parse(LegacyDocument(true, 0.1f, 0.7f, 0.4f, 0.35f, 13.0f));
  root[MarkerFieldKey(LUMICE_ANNOTATION_MARKER_ZENITH, MarkerKeyPart::kLine)] = false;
  root[MarkerFieldKey(LUMICE_ANNOTATION_MARKER_ZENITH, MarkerKeyPart::kColor)] = { 0.9f, 0.2f, 0.05f };

  GuiState state;
  ASSERT_TRUE(DeserializeGuiStateJson(root.dump(), state));

  EXPECT_FALSE(state.markers[LUMICE_ANNOTATION_MARKER_ZENITH].show);
  EXPECT_FLOAT_EQ(state.markers[LUMICE_ANNOTATION_MARKER_ZENITH].color[0], 0.9f);
  EXPECT_FLOAT_EQ(state.markers[LUMICE_ANNOTATION_MARKER_ZENITH].color[1], 0.2f);
  EXPECT_FLOAT_EQ(state.markers[LUMICE_ANNOTATION_MARKER_ZENITH].color[2], 0.05f);

  EXPECT_TRUE(state.markers[LUMICE_ANNOTATION_MARKER_NADIR].show);
  EXPECT_FLOAT_EQ(state.markers[LUMICE_ANNOTATION_MARKER_NADIR].color[1], 0.7f);

  // The family-wide pair follows the same rule at its own grain: the new key is absent here, so both
  // still come from the legacy document.
  EXPECT_FLOAT_EQ(state.markers_alpha, 0.35f);
  EXPECT_FLOAT_EQ(state.markers_radius_px, 13.0f);
}

TEST(MarkerDocumentCompat, ADocumentWithNeitherSchemaOpensAtTheFactoryValues) {
  // The control the three cases above need: without it, "the legacy keys were read" is satisfied by
  // a loader that ignores the document entirely and hands back whatever the factory says, since the
  // factory values would then be what every case measured. Here they are what the case DEMANDS, and
  // the fixture constants above are chosen to differ from all of them.
  GuiState state;
  ASSERT_TRUE(DeserializeGuiStateJson(nlohmann::json::object().dump(), state));

  const GuiState factory;
  for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
    EXPECT_EQ(state.markers[i].show, factory.markers[i].show) << "marker id " << i;
    EXPECT_EQ(state.markers[i].label, factory.markers[i].label) << "marker id " << i;
    for (int c = 0; c < 3; ++c) {
      EXPECT_FLOAT_EQ(state.markers[i].color[c], factory.markers[i].color[c]) << "marker id " << i << " ch " << c;
    }
  }
  EXPECT_FLOAT_EQ(state.markers_alpha, factory.markers_alpha);
  EXPECT_FLOAT_EQ(state.markers_radius_px, factory.markers_radius_px);
  EXPECT_EQ(state.markers_section_open, factory.markers_section_open);
}

TEST(MarkerDocumentCompat, TheSixFactoryColoursAreDistinguishableFromEachOther) {
  // The palette's whole job, asserted where it costs nothing: colour is the ONLY thing that tells
  // the six apart (radius and opacity are family-wide by design), so two factory entries landing on
  // the same colour would ship a pair no user can distinguish until they edit one. gui_test's
  // marker_panel suite makes the same claim about the RENDERED frame; this one makes it about the
  // defaults themselves, on every platform, with no display.
  const GuiState factory;
  for (int i = 0; i < LUMICE_ANNOTATION_MARKER_COUNT; ++i) {
    for (int j = i + 1; j < LUMICE_ANNOTATION_MARKER_COUNT; ++j) {
      float max_delta = 0.0f;
      for (int c = 0; c < 3; ++c) {
        max_delta = std::max(max_delta, std::abs(factory.markers[i].color[c] - factory.markers[j].color[c]));
      }
      // 0.15 in the widest-differing channel is roughly 38/255, comfortably outside the ±24/255 the
      // rendered-frame case allows for the ring's anti-aliased edge.
      EXPECT_GT(max_delta, 0.15f) << "markers " << kMarkerDisplayNames[i] << " and " << kMarkerDisplayNames[j]
                                  << " are too close to tell apart";
    }
  }
}

}  // namespace
}  // namespace lumice::gui
