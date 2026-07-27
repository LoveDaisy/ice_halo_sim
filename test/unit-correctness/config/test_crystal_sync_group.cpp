#include <spdlog/sinks/ostream_sink.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <variant>

#include "config/config_compare.hpp"
#include "config/crystal_config.hpp"
#include "core/math.hpp"
#include "gtest/gtest.h"
#include "util/logger.hpp"

// Shape-scalar sync groups, config side: canonical form, leader normalization,
// JSON codec and the equality predicate that drives re-simulation.

namespace {

namespace ns = lumice;

// Captures everything the global logger emits for the lifetime of the object.
// RAII rather than a manual remove_sink at the end of each test, because
// GetSharedSink() is a process-wide singleton: an ASSERT_* returning early with
// the sink still attached would leave later tests in this binary writing into a
// destroyed ostringstream.
class LogCapture {
 public:
  LogCapture() : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(oss_)) { ns::GetSharedSink()->add_sink(sink_); }

  ~LogCapture() { ns::GetSharedSink()->remove_sink(sink_); }

  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  std::string Text() const { return oss_.str(); }

 private:
  std::ostringstream oss_;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;
};

ns::Distribution Gauss(float mean, float std) {
  return ns::Distribution{ ns::DistributionType::kGaussian, mean, std };
}

ns::Distribution Uniform(float center, float spread) {
  return ns::Distribution{ ns::DistributionType::kUniform, center, spread };
}

// A shape JSON with randomized height and faces, matching the wire form
// Distribution::to_json emits for a non-kNoRandom distribution.
nlohmann::json PrismShapeJson() {
  return nlohmann::json{
    { "height", { { "type", "gauss" }, { "mean", 1.0 }, { "std", 0.2 } } },
    { "face_distance", nlohmann::json::array({
                           { { "type", "uniform" }, { "mean", 1.0 }, { "std", 0.4 } },
                           { { "type", "uniform" }, { "mean", 1.0 }, { "std", 0.4 } },
                           { { "type", "uniform" }, { "mean", 1.0 }, { "std", 0.4 } },
                           { { "type", "uniform" }, { "mean", 1.0 }, { "std", 0.4 } },
                           { { "type", "uniform" }, { "mean", 1.0 }, { "std", 0.4 } },
                           { { "type", "uniform" }, { "mean", 1.0 }, { "std", 0.4 } },
                       }) },
  };
}

nlohmann::json PyramidShapeJson() {
  nlohmann::json j = PrismShapeJson();
  j.erase("height");
  j["prism_h"] = { { "type", "uniform" }, { "mean", 1.0 }, { "std", 0.3 } };
  j["upper_h"] = { { "type", "gauss" }, { "mean", 0.3 }, { "std", 0.05 } };
  j["lower_h"] = { { "type", "gauss" }, { "mean", 0.3 }, { "std", 0.05 } };
  return j;
}

// ==================================================================================================
// AC3 — canonical form, one test per rule.

TEST(ShapeScalarSyncGroupCanonical, InapplicableSlotIsZeroedWithoutDissolvingTheGroup) {
  ns::PrismCrystalParam p;
  // upper_h does not exist on a prism, but faces 0 and 2 form a real group. Only
  // the inapplicable slot may be dropped — a rule 1 that also took the surviving
  // group down with it would pass a weaker single-assertion test.
  p.sync_group_[ns::kShapeScalarUpperH] = 5;
  p.sync_group_[ns::kShapeScalarFace0] = 5;
  p.sync_group_[ns::kShapeScalarFace2] = 5;
  ns::CanonicalizeSyncGroups(p);

  EXPECT_EQ(p.sync_group_[ns::kShapeScalarUpperH], 0) << "a prism has no upper_h; that declaration is not a membership";
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace0], 1);
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace2], 1);

  // Mirror case on a pyramid: it is `height` that does not exist there.
  ns::PyramidCrystalParam q;
  q.sync_group_[ns::kShapeScalarHeight] = 3;
  q.sync_group_[ns::kShapeScalarUpperH] = 3;
  q.sync_group_[ns::kShapeScalarLowerH] = 3;
  ns::CanonicalizeSyncGroups(q);

  EXPECT_EQ(q.sync_group_[ns::kShapeScalarHeight], 0);
  EXPECT_EQ(q.sync_group_[ns::kShapeScalarUpperH], 1);
  EXPECT_EQ(q.sync_group_[ns::kShapeScalarLowerH], 1);
}

TEST(ShapeScalarSyncGroupCanonical, SingletonGroupCollapsesToIndependent) {
  ns::PrismCrystalParam p;
  p.sync_group_[ns::kShapeScalarHeight] = 1;
  ns::CanonicalizeSyncGroups(p);

  for (int i = 0; i < ns::kShapeScalarCount; i++) {
    EXPECT_EQ(p.sync_group_[i], 0) << "slot " << i << ": a group of one is independence";
  }
  // And therefore indistinguishable from a default-constructed param.
  EXPECT_TRUE(p == ns::PrismCrystalParam{});
}

TEST(ShapeScalarSyncGroupCanonical, SamePartitionDifferentNumberingIsOneCanonicalForm) {
  ns::PrismCrystalParam a;
  ns::PrismCrystalParam b;
  const int a_faces[6] = { 2, 1, 2, 1, 2, 1 };
  const int b_faces[6] = { 1, 2, 1, 2, 1, 2 };
  for (int i = 0; i < 6; i++) {
    a.sync_group_[ns::kShapeScalarFace0 + i] = a_faces[i];
    b.sync_group_[ns::kShapeScalarFace0 + i] = b_faces[i];
  }

  // Equality must hold WITHOUT the caller canonicalizing first — that is the
  // whole point, since operator== is the re-simulation trigger.
  EXPECT_TRUE(a == b) << "two spellings of the same partition must not trigger a re-run";

  ns::PrismCrystalParam ca = a;
  ns::PrismCrystalParam cb = b;
  ns::CanonicalizeSyncGroups(ca);
  ns::CanonicalizeSyncGroups(cb);
  for (int i = 0; i < 6; i++) {
    EXPECT_EQ(ca.sync_group_[ns::kShapeScalarFace0 + i], b_faces[i])
        << "canonical numbering follows first appearance in ShapeScalar order, slot " << i;
    EXPECT_EQ(cb.sync_group_[ns::kShapeScalarFace0 + i], b_faces[i]);
  }
}

TEST(ShapeScalarSyncGroupCanonical, DifferentPartitionsStillCompareUnequal) {
  // Guards against writing the canonicalize-then-compare as a tautology: if
  // Canonicalize zeroed everything, or operator== ignored sync_group_ entirely,
  // the test above would still pass and this one would not.
  ns::PrismCrystalParam grouped;
  for (int i = 0; i < 6; i++) {
    grouped.sync_group_[ns::kShapeScalarFace0 + i] = (i % 2 == 0) ? 1 : 2;
  }
  EXPECT_FALSE(grouped == ns::PrismCrystalParam{}) << "an alternating C3 partition is not the all-independent param";

  // Same member count, different partition: {0,1,2}+{3,4,5} vs {0,2,4}+{1,3,5}.
  ns::PrismCrystalParam blocked;
  for (int i = 0; i < 6; i++) {
    blocked.sync_group_[ns::kShapeScalarFace0 + i] = (i < 3) ? 1 : 2;
  }
  EXPECT_FALSE(grouped == blocked);

  ns::PyramidCrystalParam pa;
  ns::PyramidCrystalParam pb;
  pa.sync_group_[ns::kShapeScalarUpperH] = 1;
  pa.sync_group_[ns::kShapeScalarLowerH] = 1;
  EXPECT_FALSE(pa == pb);
}

// ==================================================================================================
// AC5 — leader normalization is visible, in both directions.

TEST(ShapeScalarSyncGroupNormalize, DivergentMemberIsOverriddenWithAWarning) {
  ns::PrismCrystalParam p;
  p.d_[0] = Uniform(1.0f, 1.0f);
  p.d_[2] = Gauss(1.0f, 1.0f);  // Same group, different distribution.
  p.sync_group_[ns::kShapeScalarFace0] = 1;
  p.sync_group_[ns::kShapeScalarFace2] = 1;

  std::string log_text;
  {
    LogCapture capture;
    ns::PrepareSyncGroups(p);
    log_text = capture.Text();
  }

  EXPECT_NE(log_text.find("sync group"), std::string::npos)
      << "overriding a member's distribution must not be silent; captured log was: " << log_text;
  // Overridden, not rejected: the group ends up with one distribution, the
  // leader's (face 0, the lower ShapeScalar index and thus the one that draws).
  EXPECT_TRUE(p.d_[0] == p.d_[2]);
  EXPECT_TRUE(p.d_[0] == Uniform(1.0f, 1.0f));
}

TEST(ShapeScalarSyncGroupNormalize, AgreeingMembersProduceNoWarning) {
  ns::PrismCrystalParam p;
  p.d_[0] = Uniform(1.0f, 1.0f);
  p.d_[2] = Uniform(1.0f, 1.0f);
  p.sync_group_[ns::kShapeScalarFace0] = 1;
  p.sync_group_[ns::kShapeScalarFace2] = 1;

  std::string log_text;
  {
    LogCapture capture;
    ns::PrepareSyncGroups(p);
    log_text = capture.Text();
  }

  EXPECT_EQ(log_text.find("sync group"), std::string::npos)
      << "a consistent config must stay quiet; captured log was: " << log_text;
  EXPECT_TRUE(p.d_[0] == p.d_[2]);
}

TEST(ShapeScalarSyncGroupNormalize, LeaderIsTheLowestApplicableSlotNotTheLowestDeclared) {
  // On a pyramid, `height` is inapplicable, so upper_h — not the numerically
  // lower height slot — leads the group. The exclusion is structural: leader
  // election scans the slot map, and a slot this type does not have is a nullptr
  // there. It is NOT a consequence of canonicalization having zeroed the slot
  // first; see NormalizeAloneExcludesInapplicableSlotWithoutCanonicalizeFirst,
  // which reaches the same leader with only the normalize pass run.
  ns::PyramidCrystalParam p;
  p.h_pyr_u_ = Gauss(0.3f, 0.05f);
  p.h_pyr_l_ = Uniform(9.0f, 9.0f);
  p.sync_group_[ns::kShapeScalarUpperH] = 4;
  p.sync_group_[ns::kShapeScalarLowerH] = 4;

  ns::PrepareSyncGroups(p);
  EXPECT_TRUE(p.h_pyr_l_ == Gauss(0.3f, 0.05f)) << "upper_h leads the group; lower_h must adopt its distribution";
}

TEST(ShapeScalarSyncGroupNormalize, NormalizeAloneExcludesInapplicableSlotWithoutCanonicalizeFirst) {
  // NormalizeSyncGroups carries no ordering precondition — this runs it with
  // CanonicalizeSyncGroups deliberately NOT called, on the very configuration
  // the old doc comment claimed would misfire: a pyramid whose group also names
  // `height`, a slot pyramids do not have.
  ns::PyramidCrystalParam p;
  p.h_pyr_u_ = Gauss(0.3f, 0.05f);
  p.h_pyr_l_ = Uniform(9.0f, 9.0f);
  p.sync_group_[ns::kShapeScalarHeight] = 4;  // inapplicable to a pyramid, and still declared
  p.sync_group_[ns::kShapeScalarUpperH] = 4;
  p.sync_group_[ns::kShapeScalarLowerH] = 4;

  ns::NormalizeSyncGroups(p);

  EXPECT_TRUE(p.h_pyr_l_ == Gauss(0.3f, 0.05f))
      << "upper_h must still lead: the height slot names no field on a pyramid, so it can donate nothing";
  // Guards the premise of this test rather than its conclusion: had anything
  // canonicalized `p` along the way, this slot would read 0 and the case above
  // would no longer be the un-canonicalized one it claims to cover.
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarHeight], 4) << "NormalizeSyncGroups must not rewrite group numbers at all";
}

// ==================================================================================================
// AC4 — JSON codec.

TEST(ShapeScalarSyncGroupJson, PrismRoundTripIsStable) {
  nlohmann::json j = PrismShapeJson();
  j["sync_group"] = { { "face_distance", nlohmann::json::array({ 1, 2, 1, 2, 1, 2 }) } };

  const auto first = j.get<ns::PrismCrystalParam>();
  nlohmann::json round_tripped;
  ns::to_json(round_tripped, first);
  const auto second = round_tripped.get<ns::PrismCrystalParam>();

  for (int i = 0; i < ns::kShapeScalarCount; i++) {
    EXPECT_EQ(first.sync_group_[i], second.sync_group_[i]) << "slot " << i;
  }
  EXPECT_EQ(first.sync_group_[ns::kShapeScalarFace0], 1);
  EXPECT_EQ(first.sync_group_[ns::kShapeScalarFace1], 2);
  EXPECT_TRUE(first == second);
}

TEST(ShapeScalarSyncGroupJson, PyramidRoundTripCoversTheNamedHeightKeys) {
  nlohmann::json j = PyramidShapeJson();
  j["sync_group"] = {
    { "upper_h", 3 },
    { "lower_h", 3 },
    { "face_distance", nlohmann::json::array({ 1, 2, 1, 2, 1, 2 }) },
  };

  const auto first = j.get<ns::PyramidCrystalParam>();
  // Canonical renumbering follows ShapeScalar order, where the pyramidal heights
  // precede the faces: the height pair becomes group 1.
  EXPECT_EQ(first.sync_group_[ns::kShapeScalarUpperH], 1);
  EXPECT_EQ(first.sync_group_[ns::kShapeScalarLowerH], 1);
  EXPECT_EQ(first.sync_group_[ns::kShapeScalarFace0], 2);
  EXPECT_EQ(first.sync_group_[ns::kShapeScalarFace1], 3);
  EXPECT_EQ(first.sync_group_[ns::kShapeScalarPrismH], 0);

  nlohmann::json round_tripped;
  ns::to_json(round_tripped, first);
  const auto second = round_tripped.get<ns::PyramidCrystalParam>();
  for (int i = 0; i < ns::kShapeScalarCount; i++) {
    EXPECT_EQ(first.sync_group_[i], second.sync_group_[i]) << "slot " << i;
  }
  EXPECT_TRUE(first == second);
}

TEST(ShapeScalarSyncGroupJson, ConfigWithoutTheKeyParsesAsFullyIndependent) {
  const auto prism = PrismShapeJson().get<ns::PrismCrystalParam>();
  const auto pyramid = PyramidShapeJson().get<ns::PyramidCrystalParam>();
  for (int i = 0; i < ns::kShapeScalarCount; i++) {
    EXPECT_EQ(prism.sync_group_[i], 0) << "slot " << i;
    EXPECT_EQ(pyramid.sync_group_[i], 0) << "slot " << i;
  }
}

TEST(ShapeScalarSyncGroupJson, FullyIndependentParamEmitsNoSyncGroupKey) {
  nlohmann::json prism_out;
  ns::to_json(prism_out, ns::PrismCrystalParam{});
  EXPECT_FALSE(prism_out.contains("sync_group")) << "existing configs must serialize exactly as before";

  nlohmann::json pyramid_out;
  ns::to_json(pyramid_out, ns::PyramidCrystalParam{});
  EXPECT_FALSE(pyramid_out.contains("sync_group"));

  // A param whose only declaration is a singleton (or an inapplicable slot) is
  // semantically independent, so it must not resurrect the key either.
  ns::PrismCrystalParam singleton;
  singleton.sync_group_[ns::kShapeScalarFace3] = 7;
  singleton.sync_group_[ns::kShapeScalarUpperH] = 8;
  nlohmann::json singleton_out;
  ns::to_json(singleton_out, singleton);
  EXPECT_FALSE(singleton_out.contains("sync_group"));
}

TEST(ShapeScalarSyncGroupJson, SerializationDoesNotMutateTheSourceParam) {
  ns::PrismCrystalParam p;
  for (int i = 0; i < 6; i++) {
    p.sync_group_[ns::kShapeScalarFace0 + i] = (i % 2 == 0) ? 9 : 4;
  }
  const ns::PrismCrystalParam before = p;

  nlohmann::json out;
  ns::to_json(out, p);

  for (int i = 0; i < ns::kShapeScalarCount; i++) {
    EXPECT_EQ(p.sync_group_[i], before.sync_group_[i])
        << "to_json canonicalizes a copy, never its argument, slot " << i;
  }
  // The emitted numbering is canonical even though the source's is not.
  const auto emitted = out.at("sync_group").at("face_distance").get<std::vector<int>>();
  const std::vector<int> expected = { 1, 2, 1, 2, 1, 2 };
  EXPECT_EQ(emitted, expected);
}

TEST(ShapeScalarSyncGroupJson, ReachableFromTheDocumentedConfigFilePosition) {
  // The schema position users write is `crystal[].shape.sync_group`, one level
  // above the shape object the tests above feed directly. This pins that the key
  // survives CrystalConfig's own from_json rather than only the param's.
  nlohmann::json shape = PrismShapeJson();
  shape["sync_group"] = { { "face_distance", nlohmann::json::array({ 1, 2, 1, 2, 1, 2 }) } };
  const nlohmann::json crystal = {
    { "id", 1 },
    { "type", "prism" },
    { "shape", shape },
    { "axis",
      { { "zenith", { { "type", "uniform" }, { "mean", 90 }, { "std", 360 } } },
        { "azimuth", { { "type", "uniform" }, { "mean", 0 }, { "std", 360 } } } } },
  };

  const auto config = crystal.get<ns::CrystalConfig>();
  ASSERT_TRUE(std::holds_alternative<ns::PrismCrystalParam>(config.param_));
  const auto& p = std::get<ns::PrismCrystalParam>(config.param_);
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace0], 1);
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace1], 2);
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace2], 1);
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace3], 2);
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace4], 1);
  EXPECT_EQ(p.sync_group_[ns::kShapeScalarFace5], 2);
}

TEST(ShapeScalarSyncGroupJson, ParsingIntoAReusedParamClearsStaleGroups) {
  ns::PrismCrystalParam p;
  p.sync_group_[ns::kShapeScalarFace0] = 1;
  p.sync_group_[ns::kShapeScalarFace2] = 1;

  // Same object, now fed a config that declares nothing — matching how `d_` is
  // reset before parsing rather than merged into.
  ns::from_json(PrismShapeJson(), p);
  for (int i = 0; i < ns::kShapeScalarCount; i++) {
    EXPECT_EQ(p.sync_group_[i], 0) << "slot " << i;
  }
}

}  // namespace
