// Unit tests for ConfigManager::from_json post-parse binding.
//
// task-query-filter-uplift-v2 added an automatic binding pass that populates
// renderer.ms_filter_ from scattering.entries[].filter when the renderer block
// has no explicit "filter" key. The binding is the load-bearing mechanism that
// makes the query filter reach the consumer-side RenderConsumer.filters_ after
// the simulator-side filter was demoted to a branch gate; without it the
// consumer would silently see an empty filter list (Path A). Test the three
// non-trivial branches that e2e otherwise has to backstop.

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>
#include <string>
#include <variant>

#include "config/config_manager.hpp"
#include "config/filter_config.hpp"
#include "core/def.hpp"

namespace lumice {
namespace {

// Minimal valid scene wrapper. Caller fills in `filter_block` (filter array body),
// `entry_filter` (entries[0].filter field or empty), `extra_scattering` (additional
// scattering levels), and `render_filter` (render[0].filter field or empty).
std::string MakeConfigJson(const std::string& filter_block, const std::string& entry_filter,
                           const std::string& extra_scattering, const std::string& render_filter) {
  return std::string(R"({
  "crystal": [{
    "id": 1,
    "type": "prism",
    "shape": {"height": 1.2},
    "axis": {
      "zenith": {"type": "gauss", "mean": 0, "std": 180},
      "azimuth": {"type": "uniform", "mean": 0, "std": 360},
      "roll": {"type": "gauss", "mean": 0, "std": 0}
    }
  }],
  "filter": [)") +
         filter_block + R"(],
  "scene": {
    "light_source": {"type": "sun", "altitude": 20.0, "spectrum": "D65"},
    "ray_num": 1000,
    "max_hits": 7,
    "scattering": [
      {
        "prob": 0.0,
        "entries": [{"crystal": 1, "proportion": 10)" +
         entry_filter + R"(}]
      })" +
         extra_scattering + R"(
    ]
  },
  "render": [{
    "id": 1,
    "lens": {"type": "fisheye_equal_area", "fov": 120},
    "resolution": [64, 64],
    "view": {"elevation": 20})" +
         render_filter + R"(
  }]
})";
}

constexpr const char* kRealFilter = R"({
  "id": 1, "type": "raypath", "action": "filter_in",
  "raypath": [4, 6], "symmetry": "none"
})";

}  // namespace

TEST(ConfigManagerBinding, BindsScatteringFilterToRenderer) {
  // scattering.entries[0].filter = 1; renderer has no explicit filter.
  // Expectation: renderer.ms_filter_ auto-populated with the real filter.
  std::string json_text =
      MakeConfigJson(kRealFilter, R"(, "filter": 1)", /*extra_scattering=*/"", /*render_filter=*/"");

  auto j = nlohmann::json::parse(json_text);
  auto m = j.get<ConfigManager>();

  ASSERT_EQ(m.renderers_.size(), 1u);
  const auto& render = m.renderers_.at(1);
  ASSERT_EQ(render.ms_filter_.size(), 1u);
  EXPECT_EQ(render.ms_filter_[0].id_, 1);
  // Must be the real raypath filter, not a NoneFilter sentinel.
  EXPECT_NE(render.ms_filter_[0].id_, kInvalidId);
  const auto& simple = std::get<SimpleFilterParam>(render.ms_filter_[0].param_);
  EXPECT_TRUE(std::holds_alternative<RaypathFilterParam>(simple));
}

TEST(ConfigManagerBinding, DoesNotPropagateMissingFilter) {
  // scattering entry has no filter field; renderer has no explicit filter.
  // Expectation: renderer.ms_filter_ stays empty (no NoneFilter sentinel leaks
  // in — consumer's Path A is the correct branch when nothing is configured).
  std::string json_text =
      MakeConfigJson(/*filter_block=*/"", /*entry_filter=*/"", /*extra_scattering=*/"", /*render_filter=*/"");

  auto j = nlohmann::json::parse(json_text);
  auto m = j.get<ConfigManager>();

  ASSERT_EQ(m.renderers_.size(), 1u);
  EXPECT_TRUE(m.renderers_.at(1).ms_filter_.empty());
}

TEST(ConfigManagerBinding, ExplicitRendererFilterIsPreserved) {
  // Both scattering.entries[0].filter AND render.filter are set; the explicit
  // render.filter wins (binding short-circuits on non-empty ms_filter_).
  // Use renderer.filter=[-1] (NoneFilter sentinel) so we can detect override —
  // if the binding ran, ms_filter_[0].id_ would be 1, not kInvalidId.
  std::string json_text = MakeConfigJson(kRealFilter, R"(, "filter": 1)", /*extra_scattering=*/"",
                                         /*render_filter=*/R"(, "filter": [-1])");

  auto j = nlohmann::json::parse(json_text);
  auto m = j.get<ConfigManager>();

  ASSERT_EQ(m.renderers_.size(), 1u);
  const auto& render = m.renderers_.at(1);
  ASSERT_EQ(render.ms_filter_.size(), 1u);
  EXPECT_EQ(render.ms_filter_[0].id_, kInvalidId) << "explicit renderer.filter=[-1] was overwritten by binding";
}

TEST(ConfigManagerBinding, MultiMsLevelBindsOnePerLevel) {
  // Two scattering levels, each entry has the same real filter. The binding
  // contributes one entry per ms level (current 1+1 scope, inner break).
  std::string extra_scattering = R"(,
      {
        "prob": 0.0,
        "entries": [{"crystal": 1, "proportion": 10, "filter": 1}]
      })";
  std::string json_text = MakeConfigJson(kRealFilter, R"(, "filter": 1)", extra_scattering, /*render_filter=*/"");

  auto j = nlohmann::json::parse(json_text);
  auto m = j.get<ConfigManager>();

  ASSERT_EQ(m.scene_.ms_.size(), 2u);
  const auto& render = m.renderers_.at(1);
  ASSERT_EQ(render.ms_filter_.size(), 2u) << "expected one bound filter per ms level";
  EXPECT_EQ(render.ms_filter_[0].id_, 1);
  EXPECT_EQ(render.ms_filter_[1].id_, 1);
}

}  // namespace lumice
