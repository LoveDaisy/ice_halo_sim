#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <vector>

#include "config/config_compare.hpp"
#include "config/config_manager.hpp"
#include "include/lumice.h"
#include "server/c_api_internal.hpp"  // ConfigScratch + ParseConfigString + ConfigToJson (internal)
#include "util/color_space.hpp"       // SrgbToLinear (the JSON boundary conversion under test)
#include "util/logger.hpp"            // GetSharedSink (dead-key warning capture)

// Differential tests: core's native config parser (config/*::from_json) vs the C API JSON parser
// behind ParseConfigString / LUMICE_SceneFromJson (server/c_api.cpp::JsonToConfig).
//
// Why this exists: once the handle path is the only way to feed a config into the library, it must
// accept everything the core parser accepts and reject everything the core parser rejects — a
// stricter or looser C API parser means "Lumice can no longer read its own configs" (or silently
// accepts a config core would refuse). Reading both parsers side by side is not evidence; the core
// parser is used here as a mechanical oracle instead.
//
// Three divergence classes, all detected mechanically:
//   A — core accepts, C API rejects  (C API too strict)
//   B — core rejects, C API accepts  (C API too loose)
//   C — both accept, but the resulting configs differ in value
//
// Class C is detected through the same funnel the real commit path uses: every commit entry point
// ends up handing ConfigToJson()'s re-encoding of the parsed struct to core's own parser. So a
// value is preserved end-to-end iff core(ConfigToJson(capi_parse(text))) == core(text). No server
// or simulation is involved.

extern std::string repo_root_dir;

namespace {

namespace fs = std::filesystem;

// =============== Corpus ===============

// Every JSON config tracked in the repo: the shipped examples, the e2e corpus (including its
// error/ negatives — a negative that only ONE parser rejects is itself a divergence), and the
// unit-test fixture. Enumerated from disk rather than listed, so new corpus files are covered
// without touching this file.
std::vector<fs::path> CollectCorpus() {
  std::vector<fs::path> files;
  if (repo_root_dir.empty()) {
    return files;
  }
  const fs::path root(repo_root_dir);
  const fs::path dirs[] = { root / "examples", root / "test" / "e2e" / "configs" };
  for (const auto& dir : dirs) {
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) {
      continue;
    }
    for (const auto& entry : fs::recursive_directory_iterator(dir, ec)) {
      if (entry.is_regular_file() && entry.path().extension() == ".json") {
        files.push_back(entry.path());
      }
    }
  }
  const fs::path fixture = root / "test" / "fixtures" / "v3_config.json";
  std::error_code ec;
  if (fs::is_regular_file(fixture, ec)) {
    files.push_back(fixture);
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::string ReadFile(const fs::path& p) {
  std::ifstream f(p, std::ios::binary);
  std::ostringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// Path relative to the repo root, for readable failure messages.
std::string RelPath(const fs::path& p) {
  std::error_code ec;
  auto rel = fs::relative(p, fs::path(repo_root_dir), ec);
  return ec ? p.string() : rel.generic_string();
}

// =============== Parsers ===============

struct CoreOutcome {
  bool ok = false;
  std::string error;
  lumice::ConfigManager config;
};

// The oracle: core's own from_json. Anything it throws is "core rejects".
CoreOutcome ParseWithCore(const std::string& text) {
  CoreOutcome out;
  nlohmann::json root;
  try {
    root = nlohmann::json::parse(text);
  } catch (const std::exception& e) {
    out.error = std::string("json syntax: ") + e.what();
    return out;
  }
  try {
    lumice::from_json(root, out.config);
    out.ok = true;
  } catch (const std::exception& e) {
    out.error = e.what();
  }
  return out;
}

// Re-encode a parsed ConfigScratch through the same ConfigToJson() the commit path uses, then
// hand the result back to the core parser. Equality with the core-direct parse is exactly the
// "value survived the C API round trip" property.
CoreOutcome ReparseThroughCapiEncoding(const ConfigScratch& cfg) {
  CoreOutcome out;
  nlohmann::json encoded;
  try {
    encoded = ConfigToJson(cfg);
  } catch (const std::exception& e) {
    out.error = std::string("ConfigToJson threw: ") + e.what();
    return out;
  }
  try {
    lumice::from_json(encoded, out.config);
    out.ok = true;
  } catch (const std::exception& e) {
    out.error = std::string("core rejected the C API re-encoding: ") + e.what();
  }
  return out;
}

// A kNoRandom distribution's `spread` slot is documented as unused (core math.hpp: "kNoRandom |
// the value itself | unused"), and core's own from_json leaves whatever the destination happened
// to hold there — a bare-number distribution never writes it. Two configs that differ only in
// that dead slot are the same configuration, so both sides are normalized before comparison
// rather than weakening the shared field-by-field operator==.
void NormalizeDeadSpread(lumice::Distribution& d) {
  if (d.type == lumice::DistributionType::kNoRandom) {
    d.spread = 0.0f;
  }
}

void NormalizeDeadSpread(lumice::CrystalConfig& crystal) {
  NormalizeDeadSpread(crystal.axis_.azimuth_dist);
  NormalizeDeadSpread(crystal.axis_.latitude_dist);
  NormalizeDeadSpread(crystal.axis_.roll_dist);
  std::visit(
      [](auto& param) {
        using T = std::decay_t<decltype(param)>;
        if constexpr (std::is_same_v<T, lumice::PrismCrystalParam>) {
          NormalizeDeadSpread(param.h_);
        } else {
          NormalizeDeadSpread(param.h_prs_);
          NormalizeDeadSpread(param.h_pyr_u_);
          NormalizeDeadSpread(param.h_pyr_l_);
        }
        for (auto& d : param.d_) {
          NormalizeDeadSpread(d);
        }
      },
      crystal.param_);
}

void NormalizeDeadSpread(lumice::ConfigManager& m) {
  for (auto& [id, crystal] : m.crystals_) {
    NormalizeDeadSpread(crystal);
  }
  // Each scattering entry carries its OWN copy of the crystal it names.
  for (auto& ms : m.scene_.ms_) {
    for (auto& setting : ms.setting_) {
      NormalizeDeadSpread(setting.crystal_);
    }
  }
}

// Which top-level sections differ between two ConfigManagers.
struct SectionDiff {
  bool crystals = false;
  bool filters = false;
  bool renderers = false;
  bool scene = false;
  bool raypath_color = false;

  // v4.11 closed the renderer expression gap, so `renderers` is no longer carved out: every
  // section is now held to the same "values must survive the round trip" standard.
  bool Any() const { return crystals || filters || renderers || scene || raypath_color; }

  // The offending items, re-serialized through core's own to_json so a failure names the field
  // instead of dumping raw struct bytes.
  std::string detail;

  std::string ToString() const {
    std::string s;
    auto add = [&s](bool flag, const char* name) {
      if (flag) {
        s += s.empty() ? name : std::string(", ") + name;
      }
    };
    add(crystals, "crystal");
    add(filters, "filter");
    add(renderers, "render");
    add(scene, "scene");
    add(raypath_color, "raypath_color");
    return s.empty() ? "<none>" : s;
  }
};

// Per-id diff of one id-keyed section, rendered as core JSON for the two sides.
template <typename MapT>
void AppendItemDiff(const char* section, const MapT& expected, const MapT& actual, std::string* detail) {
  for (const auto& [id, item] : expected) {
    auto it = actual.find(id);
    if (it == actual.end()) {
      *detail += std::string("\n  ") + section + " id " + std::to_string(id) + ": missing after round trip";
      continue;
    }
    if (!(it->second == item)) {
      nlohmann::json a;
      nlohmann::json b;
      lumice::to_json(a, item);
      lumice::to_json(b, it->second);
      *detail += std::string("\n  ") + section + " id " + std::to_string(id) + ":\n    core: " + a.dump() +
                 "\n    capi: " + b.dump();
    }
  }
}

SectionDiff DiffSections(const lumice::ConfigManager& a, const lumice::ConfigManager& b) {
  SectionDiff d;
  d.crystals = !(a.crystals_ == b.crystals_);
  d.filters = !(a.filters_ == b.filters_);
  d.renderers = !(a.renderers_ == b.renderers_);
  d.scene = !(a.scene_ == b.scene_);
  d.raypath_color = !(a.raypath_color_ == b.raypath_color_);
  if (d.crystals) {
    AppendItemDiff("crystal", a.crystals_, b.crystals_, &d.detail);
  }
  if (d.filters) {
    AppendItemDiff("filter", a.filters_, b.filters_, &d.detail);
  }
  if (d.renderers) {
    AppendItemDiff("render", a.renderers_, b.renderers_, &d.detail);
  }
  if (d.scene) {
    nlohmann::json sa;
    nlohmann::json sb;
    lumice::to_json(sa, a.scene_);
    lumice::to_json(sb, b.scene_);
    d.detail += "\n  scene:\n    core: " + sa.dump() + "\n    capi: " + sb.dump();
  }
  return d;
}

// =============== Corpus sweep ===============

struct CorpusCase {
  std::string path;  // repo-relative
  CoreOutcome core;  // oracle
  LUMICE_ErrorCode capi_status = LUMICE_OK;
  bool capi_ok = false;        // the C API parse stage accepted it
  bool round_trip_ok = false;  // ... and core accepted the C API's re-encoding of the result

  // The C API validates in two stages: the JSON parser does a lossless translation, then core
  // re-validates the re-encoded document at commit time (LUMICE_CommitScene hands the core a
  // document produced by the same ConfigToJson encoder this test drives). So
  // "the C API accepts this config" end-to-end means BOTH stages passed; a config the parser
  // waves through but core rejects on re-encode is still rejected by every real entry point.
  bool AcceptedEndToEnd() const { return capi_ok && round_trip_ok; }
  std::string round_trip_error;
  SectionDiff diff;
  // Per-renderer diff, kept alongside the whole-section SectionDiff::renderers flag because it
  // names the offending renderer id. Before v4.11 this compared only the subset
  // LUMICE_RenderParam could carry; the struct now carries every field, so the comparison is
  // core's own RenderConfig::operator== with no whitelist.
  bool renderer_diff = false;
  std::string renderer_diff_detail;
};

// Run both parsers over the whole corpus once. Shared by the divergence-class tests so the sweep
// (and its file IO) happens a single time per binary run.
const std::vector<CorpusCase>& Corpus() {
  static const std::vector<CorpusCase> cases = [] {
    std::vector<CorpusCase> result;
    for (const auto& file : CollectCorpus()) {
      CorpusCase c;
      c.path = RelPath(file);
      const std::string text = ReadFile(file);
      c.core = ParseWithCore(text);

      ConfigScratch cfg{};
      ConfigScratchGuard guard(cfg);
      c.capi_status = ParseConfigString(text.c_str(), &cfg);
      c.capi_ok = (c.capi_status == LUMICE_OK);

      if (c.capi_ok) {
        CoreOutcome round_trip = ReparseThroughCapiEncoding(cfg);
        c.round_trip_ok = round_trip.ok;
        c.round_trip_error = round_trip.error;
        if (c.core.ok && round_trip.ok) {
          NormalizeDeadSpread(c.core.config);
          NormalizeDeadSpread(round_trip.config);
          c.diff = DiffSections(c.core.config, round_trip.config);
          for (const auto& [id, expected] : c.core.config.renderers_) {
            auto it = round_trip.config.renderers_.find(id);
            if (it == round_trip.config.renderers_.end()) {
              c.renderer_diff = true;
              c.renderer_diff_detail += " renderer id " + std::to_string(id) + " missing;";
              continue;
            }
            if (!(expected == it->second)) {
              nlohmann::json a;
              nlohmann::json b;
              lumice::to_json(a, expected);
              lumice::to_json(b, it->second);
              c.renderer_diff = true;
              c.renderer_diff_detail +=
                  "\n  renderer id " + std::to_string(id) + ":\n    core: " + a.dump() + "\n    capi: " + b.dump();
            }
          }
          if (c.core.config.renderers_.size() != round_trip.config.renderers_.size()) {
            c.renderer_diff = true;
            c.renderer_diff_detail += " renderer count differs;";
          }
        }
      }
      result.push_back(std::move(c));
    }
    return result;
  }();
  return cases;
}

bool CorpusAvailable() {
  if (repo_root_dir.empty()) {
    GTEST_LOG_(WARNING) << "repo root not passed as argv[3]; corpus parity tests skipped";
    return false;
  }
  return true;
}

// =============== Divergence class A: C API rejects what core accepts ===============

TEST(JsonParserParity, CorpusCapiAcceptsEverythingCoreAccepts) {
  if (!CorpusAvailable()) {
    GTEST_SKIP();
  }
  ASSERT_FALSE(Corpus().empty()) << "corpus enumeration found no config files";
  for (const auto& c : Corpus()) {
    if (c.core.ok && !c.capi_ok) {
      ADD_FAILURE() << "divergence A (C API too strict): " << c.path << " — core accepted it, C API returned "
                    << c.capi_status;
    }
  }
}

// =============== Divergence class B: C API accepts what core rejects ===============

TEST(JsonParserParity, CorpusCapiRejectsEverythingCoreRejects) {
  if (!CorpusAvailable()) {
    GTEST_SKIP();
  }
  ASSERT_FALSE(Corpus().empty()) << "corpus enumeration found no config files";
  for (const auto& c : Corpus()) {
    if (!c.core.ok && c.AcceptedEndToEnd()) {
      ADD_FAILURE() << "divergence B (C API too loose): " << c.path
                    << " — C API accepted it, core rejected it with: " << c.core.error;
    }
  }
}

// =============== Divergence class C: values must survive the C API round trip ===============
//
// Covers every section. The renderer section used to be carved out — LUMICE_RenderParam had no
// fields for lens / view / visible / background / ray_color / lens_shift / grid, so those values
// were dropped by construction — but v4.11 widened the struct to the full renderer description,
// so there is no longer a section held to a weaker standard than the rest.
// (Formerly JsonParserParity.CorpusValuesSurviveCapiRoundTripOutsideRendererGap.)
TEST(JsonParserParity, CorpusValuesSurviveCapiRoundTrip) {
  if (!CorpusAvailable()) {
    GTEST_SKIP();
  }
  ASSERT_FALSE(Corpus().empty()) << "corpus enumeration found no config files";
  for (const auto& c : Corpus()) {
    if (!c.core.ok || !c.capi_ok) {
      continue;
    }
    if (!c.round_trip_ok) {
      ADD_FAILURE() << "divergence C: " << c.path << " — " << c.round_trip_error;
      continue;
    }
    if (c.diff.Any()) {
      ADD_FAILURE() << "divergence C (value not preserved): " << c.path
                    << " — differing sections: " << c.diff.ToString() << c.diff.detail;
    }
  }
}

// Every renderer field must survive the round trip — no whitelist. Before v4.11 this asserted the
// weaker "the expression gap stays confined to the fields the C struct cannot carry" (only
// id / resolution / intensity_factor / overlap were compared, because lens / lens_shift
// / view / visible / background / ray_color / grid / horizon had no home in
// LUMICE_RenderParam and were silently replaced with a hardcoded renderer). The struct now carries
// all of them, so the comparison is core's own RenderConfig::operator==.
// (Formerly JsonParserParity.CorpusRendererGapConfinedToUnrepresentableFields.)
TEST(JsonParserParity, CorpusRendererFieldsSurviveCapiRoundTrip) {
  if (!CorpusAvailable()) {
    GTEST_SKIP();
  }
  for (const auto& c : Corpus()) {
    if (!c.round_trip_ok) {
      continue;
    }
    EXPECT_FALSE(c.renderer_diff) << "renderer field did not survive the C API round trip: " << c.path << " —"
                                  << c.renderer_diff_detail;
  }
}

// =============== Named alignment pins ===============
//
// Corpus coverage proves "the configs we ship still work"; it cannot prove a specific field's
// default is right, because the corpus does not necessarily contain that field combination.
// These pin the individual required-vs-optional decisions and the exact default value, and each
// was observed RED against the pre-alignment parser.

// Parse `text` with both parsers and require both to accept. Returns the two configs.
struct BothParsed {
  lumice::ConfigManager core;
  lumice::ConfigManager via_capi;
};

testing::AssertionResult ParseWithBoth(const std::string& text, BothParsed* out) {
  CoreOutcome core = ParseWithCore(text);
  if (!core.ok) {
    return testing::AssertionFailure() << "core rejected the input: " << core.error;
  }
  ConfigScratch cfg{};
  ConfigScratchGuard guard(cfg);
  LUMICE_ErrorCode status = ParseConfigString(text.c_str(), &cfg);
  if (status != LUMICE_OK) {
    return testing::AssertionFailure() << "C API rejected the input: status " << status;
  }
  CoreOutcome round_trip = ReparseThroughCapiEncoding(cfg);
  if (!round_trip.ok) {
    return testing::AssertionFailure() << round_trip.error;
  }
  NormalizeDeadSpread(core.config);
  NormalizeDeadSpread(round_trip.config);
  out->core = std::move(core.config);
  out->via_capi = std::move(round_trip.config);
  return testing::AssertionSuccess();
}

// End-to-end C API verdict on one document: the parse stage AND core's re-validation of the
// re-encoding (see CorpusCase::AcceptedEndToEnd for why both stages count).
bool CapiAcceptsEndToEnd(const std::string& text) {
  ConfigScratch cfg{};
  ConfigScratchGuard guard(cfg);
  if (ParseConfigString(text.c_str(), &cfg) != LUMICE_OK) {
    return false;
  }
  return ReparseThroughCapiEncoding(cfg).ok;
}

// Minimal well-formed blocks shared by the pins below.
const char* kMinimalSceneBlock = R"("scene": {
      "ray_num": 100, "max_hits": 4,
      "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
      "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ]
    })";
const char* kMinimalRenderBlock = R"("render": [ { "id": 1, "resolution": [64, 32] } ])";
const char* kMinimalCrystal = R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 } })";

std::string WrapCrystal(const std::string& crystal_json) {
  return std::string("{ \"crystal\": [") + crystal_json + "], \"filter\": [], " + kMinimalSceneBlock + ", " +
         kMinimalRenderBlock + " }";
}

std::string WrapFilter(const std::string& filter_json) {
  return std::string("{ \"crystal\": [") + kMinimalCrystal + "], \"filter\": [" + filter_json + "], " +
         kMinimalSceneBlock + ", " + kMinimalRenderBlock + " }";
}

const lumice::CrystalConfig& OnlyCrystal(const lumice::ConfigManager& m) {
  return m.crystals_.begin()->second;
}

// --- crystal.axis ---
// Core's two axis cases are NOT symmetric, and conflating them silently changes the sampled
// orientation: an absent `axis` object takes AxisDistribution's default constructor (zenith 0 /
// azimuth 0 / roll 0, all no-random), whereas a PRESENT `axis` object requires `zenith` and
// defaults the two others to a full 360° uniform sweep.

TEST(JsonParserParity, AxisOmittedEntirely) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapCrystal(R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 } })"), &p));
  const auto& axis = OnlyCrystal(p.via_capi).axis_;
  EXPECT_EQ(axis, OnlyCrystal(p.core).axis_);
  // Spelled out so the pin fails loudly if the core default itself ever moves.
  EXPECT_EQ(axis.latitude_dist.type, lumice::DistributionType::kNoRandom);
  EXPECT_FLOAT_EQ(axis.latitude_dist.center, 90.0f);  // zenith 0 == latitude 90
  EXPECT_EQ(axis.azimuth_dist.type, lumice::DistributionType::kNoRandom);
  EXPECT_EQ(axis.roll_dist.type, lumice::DistributionType::kNoRandom);
}

TEST(JsonParserParity, AxisPresentZenithOnly) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(
      WrapCrystal(R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 }, "axis": { "zenith": 90 } })"), &p));
  const auto& axis = OnlyCrystal(p.via_capi).axis_;
  EXPECT_EQ(axis, OnlyCrystal(p.core).axis_);
  EXPECT_EQ(axis.latitude_dist.type, lumice::DistributionType::kNoRandom);
  EXPECT_FLOAT_EQ(axis.latitude_dist.center, 0.0f);  // zenith 90 == latitude 0
  // The asymmetric part: present-but-partial `axis` defaults to a uniform full sweep, NOT to 0.
  EXPECT_EQ(axis.azimuth_dist.type, lumice::DistributionType::kUniform);
  EXPECT_FLOAT_EQ(axis.azimuth_dist.spread, 360.0f);
  EXPECT_EQ(axis.roll_dist.type, lumice::DistributionType::kUniform);
  EXPECT_FLOAT_EQ(axis.roll_dist.spread, 360.0f);
}

TEST(JsonParserParity, AxisPresentWithoutZenithIsRejected) {
  const std::string text =
      WrapCrystal(R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 }, "axis": { "roll": 0 } })");
  const auto core = ParseWithCore(text);
  EXPECT_FALSE(core.ok) << "core is expected to require axis.zenith";
  // The rejection is old; the message is not. It used to be nlohmann's raw
  // "[json.exception.out_of_range.403] key 'zenith' not found", which names neither the crystal
  // nor a way forward.
  EXPECT_EQ(core.error.find("out_of_range.403"), std::string::npos)
      << "expected this repo's own diagnosis, got nlohmann's: " << core.error;
  EXPECT_NE(core.error.find("crystal[id=1]"), std::string::npos) << core.error;
  EXPECT_NE(core.error.find("zenith"), std::string::npos) << core.error;
  EXPECT_FALSE(CapiAcceptsEndToEnd(text));
}

// An axis slot written as an object must name its `type`, in BOTH parsers. Both are required: the
// CLI reaches core only through the C API's parser, which re-serializes what it parsed via
// ConfigToJson — and that writer always emits `type`, so a check living only in core would never
// see the missing key. Landing the core half alone leaves core's own test green while the CLI goes
// on accepting the document, which is precisely how the `prob` narrowing nearly shipped half-done.
//
// One case per slot, not one case: before this narrowing the three slots silently produced three
// DIFFERENT wrong answers (zenith kept kNoRandom, azimuth/roll kept uniform-360), so the slots are
// not interchangeable evidence for one another.
TEST(JsonParserParity, AxisSlotObjectWithoutTypeIsRejected) {
  for (const char* slot : { "zenith", "azimuth", "roll" }) {
    SCOPED_TRACE(slot);
    // `zenith` is always present so the failure under test is the missing `type`, never a missing
    // `zenith` — for that slot the two collapse onto the same object.
    std::string axis = R"("zenith": { "type": "gauss", "mean": 90, "std": 1 })";
    const std::string typeless = std::string("\"") + slot + R"(": { "mean": 20, "std": 5 })";
    if (std::string(slot) == "zenith") {
      axis = typeless;
    } else {
      axis += ", " + typeless;
    }
    const std::string text =
        WrapCrystal(R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 }, "axis": { )" + axis + " } }");

    const auto core = ParseWithCore(text);
    EXPECT_FALSE(core.ok) << "core must reject an axis." << slot << " object with no \"type\"";
    EXPECT_NE(core.error.find(std::string("axis.") + slot), std::string::npos) << core.error;
    EXPECT_FALSE(CapiAcceptsEndToEnd(text)) << "the C API parser must reject it too, or the CLI still accepts it";
  }
}

// The other half of the same edit: deleting the typeless-object fallback must not disturb the
// deliberate seed for an ABSENT azimuth / roll key, which 47 slots in this repo's corpus rely on.
// AxisPresentZenithOnly above already pins core's values; this pins that both parsers still agree
// on them, which is the property that would break if only one of the two had been narrowed.
TEST(JsonParserParity, AxisKeysAbsentStillAgreeOnUniform360) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(
      WrapCrystal(
          R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 }, "axis": { "zenith": { "type": "gauss", "mean": 90, "std": 1 } } })"),
      &p));
  const auto& axis = OnlyCrystal(p.via_capi).axis_;
  EXPECT_EQ(axis, OnlyCrystal(p.core).axis_);
  EXPECT_EQ(axis.azimuth_dist.type, lumice::DistributionType::kUniform);
  EXPECT_FLOAT_EQ(axis.azimuth_dist.spread, 360.0f);
  EXPECT_EQ(axis.roll_dist.type, lumice::DistributionType::kUniform);
  EXPECT_FLOAT_EQ(axis.roll_dist.spread, 360.0f);
}

// --- crystal.shape ---

// Both parsers share one distribution entry point with the axis slots, so the missing-`type`
// narrowing reaches the shape scalars as well. Pinned on both sides, because "the check is
// shared" is a claim about two separate implementations here, not one.
TEST(JsonParserParity, ShapeScalarObjectWithoutTypeIsRejected) {
  const std::string text =
      WrapCrystal(R"({ "id": 1, "type": "prism", "shape": { "height": { "mean": 1.0, "std": 0.2 } } })");
  EXPECT_FALSE(ParseWithCore(text).ok) << "core must reject a shape.height object with no \"type\"";
  EXPECT_FALSE(CapiAcceptsEndToEnd(text));
}

TEST(JsonParserParity, PrismHeightOmitted) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapCrystal(R"({ "id": 1, "type": "prism", "shape": {} })"), &p));
  const auto& param = std::get<lumice::PrismCrystalParam>(OnlyCrystal(p.via_capi).param_);
  EXPECT_EQ(param, std::get<lumice::PrismCrystalParam>(OnlyCrystal(p.core).param_));
  EXPECT_EQ(param.h_.type, lumice::DistributionType::kNoRandom);
  EXPECT_FLOAT_EQ(param.h_.center, 1.0f);
}

TEST(JsonParserParity, PyramidHeightsOmitted) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapCrystal(R"({ "id": 1, "type": "pyramid", "shape": { "prism_h": 0.5 } })"), &p));
  const auto& param = std::get<lumice::PyramidCrystalParam>(OnlyCrystal(p.via_capi).param_);
  EXPECT_EQ(param, std::get<lumice::PyramidCrystalParam>(OnlyCrystal(p.core).param_));
  EXPECT_FLOAT_EQ(param.h_prs_.center, 0.5f);
  EXPECT_EQ(param.h_pyr_u_.type, lumice::DistributionType::kNoRandom);
  EXPECT_FLOAT_EQ(param.h_pyr_u_.center, 0.0f);
  EXPECT_EQ(param.h_pyr_l_.type, lumice::DistributionType::kNoRandom);
  EXPECT_FLOAT_EQ(param.h_pyr_l_.center, 0.0f);
}

TEST(JsonParserParity, FaceDistanceShorterThanSixKeepsDefaultsForTheRest) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(
      WrapCrystal(R"({ "id": 1, "type": "prism", "shape": { "height": 1.0, "face_distance": [0.8, 0.9] } })"), &p));
  const auto& param = std::get<lumice::PrismCrystalParam>(OnlyCrystal(p.via_capi).param_);
  EXPECT_EQ(param, std::get<lumice::PrismCrystalParam>(OnlyCrystal(p.core).param_));
  EXPECT_FLOAT_EQ(param.d_[0].center, 0.8f);
  EXPECT_FLOAT_EQ(param.d_[1].center, 0.9f);
  for (int i = 2; i < 6; i++) {
    EXPECT_EQ(param.d_[i].type, lumice::DistributionType::kNoRandom);
    EXPECT_FLOAT_EQ(param.d_[i].center, 1.0f) << "index " << i;
  }
}

TEST(JsonParserParity, DistributionObjectWithoutStdKeepsCoreDefault) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(
      WrapCrystal(
          R"({ "id": 1, "type": "prism", "shape": { "height": 1.0 }, "axis": { "zenith": { "type": "gauss", "mean": 90 } } })"),
      &p));
  const auto& axis = OnlyCrystal(p.via_capi).axis_;
  EXPECT_EQ(axis, OnlyCrystal(p.core).axis_);
  EXPECT_EQ(axis.latitude_dist.type, lumice::DistributionType::kGaussian);
  EXPECT_FLOAT_EQ(axis.latitude_dist.center, 0.0f);
}

// --- filter.action ---

TEST(JsonParserParity, FilterActionOmittedDefaultsToFilterIn) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapFilter(R"({ "id": 1, "type": "raypath", "raypath": [3, 5] })"), &p));
  const auto& filter = p.via_capi.filters_.begin()->second;
  EXPECT_EQ(filter, p.core.filters_.begin()->second);
  EXPECT_EQ(filter.action_, lumice::FilterConfig::kFilterIn);
}

// --- fields core requires: the C API must not accept them missing ---

TEST(JsonParserParity, FilterRaypathMissingIsRejected) {
  const std::string text = WrapFilter(R"({ "id": 1, "type": "raypath", "action": "filter_in" })");
  EXPECT_FALSE(ParseWithCore(text).ok) << "core is expected to require filter.raypath";
  EXPECT_FALSE(CapiAcceptsEndToEnd(text));
}

// A document with every required block present; the caller swaps in the one under test.
std::string Document(const std::string& crystal_block, const std::string& filter_block, const std::string& scene_block,
                     const std::string& render_block) {
  return "{ " + crystal_block + ", " + filter_block + ", " + scene_block + ", " + render_block + " }";
}

const std::string kCrystalBlock = std::string("\"crystal\": [") + kMinimalCrystal + "]";
const std::string kFilterBlock = "\"filter\": []";

TEST(JsonParserParity, RendererIdMissingIsRejected) {
  const std::string text =
      Document(kCrystalBlock, kFilterBlock, kMinimalSceneBlock, R"("render": [ { "resolution": [64, 32] } ])");
  EXPECT_FALSE(ParseWithCore(text).ok) << "core is expected to require render.id";
  EXPECT_FALSE(CapiAcceptsEndToEnd(text));
}

TEST(JsonParserParity, RendererResolutionMissingIsRejected) {
  const std::string text = Document(kCrystalBlock, kFilterBlock, kMinimalSceneBlock, R"("render": [ { "id": 1 } ])");
  EXPECT_FALSE(ParseWithCore(text).ok) << "core is expected to require render.resolution";
  EXPECT_FALSE(CapiAcceptsEndToEnd(text));
}

// Core iterates j.at("filter") / j.at("render") directly, so both keys must exist even when the
// array is empty — "no renderers at all" is not a config core will read.
TEST(JsonParserParity, TopLevelFilterAndRenderKeysAreRequired) {
  const std::string no_filter = "{ " + kCrystalBlock + ", " + kMinimalSceneBlock + ", " + kMinimalRenderBlock + " }";
  const std::string no_render = "{ " + kCrystalBlock + ", " + kFilterBlock + ", " + kMinimalSceneBlock + " }";
  for (const std::string& text : { no_filter, no_render }) {
    EXPECT_FALSE(ParseWithCore(text).ok) << "core is expected to reject: " << text;
    EXPECT_FALSE(CapiAcceptsEndToEnd(text)) << "C API accepted: " << text;
  }
}

TEST(JsonParserParity, LightSourceIsRequiredWithTypeAltitudeAndSpectrum) {
  const char* kBadScenes[] = {
    // no light_source at all
    R"("scene": { "ray_num": 100, "max_hits": 4,
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ] })",
    // light_source without "type"
    R"("scene": { "ray_num": 100, "max_hits": 4, "light_source": { "altitude": 20, "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ] })",
    // light_source without "altitude"
    R"("scene": { "ray_num": 100, "max_hits": 4, "light_source": { "type": "sun", "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ] })",
    // light_source without "spectrum"
    R"("scene": { "ray_num": 100, "max_hits": 4, "light_source": { "type": "sun", "altitude": 20 },
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ] })",
  };
  for (const char* scene : kBadScenes) {
    const std::string text = Document(kCrystalBlock, kFilterBlock, scene, kMinimalRenderBlock);
    EXPECT_FALSE(ParseWithCore(text).ok) << "core is expected to reject: " << text;
    EXPECT_FALSE(CapiAcceptsEndToEnd(text)) << "C API accepted: " << text;
  }
}

TEST(JsonParserParity, SceneRayNumAndMaxHitsAndScatteringAreRequired) {
  const char* kBadScenes[] = {
    R"("scene": { "max_hits": 4, "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ] })",
    R"("scene": { "ray_num": 100, "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1 } ] } ] })",
    R"("scene": { "ray_num": 100, "max_hits": 4,
                  "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" } })",
  };
  for (const char* scene : kBadScenes) {
    const std::string text = Document(kCrystalBlock, kFilterBlock, scene, kMinimalRenderBlock);
    EXPECT_FALSE(ParseWithCore(text).ok) << "core is expected to reject: " << text;
    EXPECT_FALSE(CapiAcceptsEndToEnd(text)) << "C API accepted: " << text;
  }
}

TEST(JsonParserParity, ScatteringEntriesAndIdReferencesAreValidated) {
  const char* kBadScenes[] = {
    // layer without "entries"
    R"("scene": { "ray_num": 100, "max_hits": 4,
                  "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0 } ] })",
    // entry without "crystal"
    R"("scene": { "ray_num": 100, "max_hits": 4,
                  "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0, "entries": [ { "proportion": 100 } ] } ] })",
    // entry naming a crystal id that does not exist
    R"("scene": { "ray_num": 100, "max_hits": 4,
                  "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 7 } ] } ] })",
    // entry naming a filter id that does not exist
    R"("scene": { "ray_num": 100, "max_hits": 4,
                  "light_source": { "type": "sun", "altitude": 20, "spectrum": "D65" },
                  "scattering": [ { "prob": 1.0, "entries": [ { "crystal": 1, "filter": 9 } ] } ] })",
  };
  for (const char* scene : kBadScenes) {
    const std::string text = Document(kCrystalBlock, kFilterBlock, scene, kMinimalRenderBlock);
    EXPECT_FALSE(ParseWithCore(text).ok) << "core is expected to reject: " << text;
    EXPECT_FALSE(CapiAcceptsEndToEnd(text)) << "C API accepted: " << text;
  }
}

// --- defaults that the zeroed C struct would otherwise silently change ---

TEST(JsonParserParity, ScatteringProportionOmittedDefaultsTo100) {
  const std::string text = Document(kCrystalBlock, kFilterBlock, kMinimalSceneBlock, kMinimalRenderBlock);
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(text, &p));
  ASSERT_EQ(p.via_capi.scene_.ms_.size(), 1u);
  ASSERT_EQ(p.via_capi.scene_.ms_[0].setting_.size(), 1u);
  EXPECT_EQ(p.via_capi.scene_, p.core.scene_);
  EXPECT_FLOAT_EQ(p.via_capi.scene_.ms_[0].setting_[0].crystal_proportion_, 100.0f);
}

TEST(JsonParserParity, RendererIntensityFactorOmittedDefaultsToOne) {
  const std::string text = Document(kCrystalBlock, kFilterBlock, kMinimalSceneBlock, kMinimalRenderBlock);
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(text, &p));
  ASSERT_EQ(p.via_capi.renderers_.size(), 1u);
  const auto& renderer = p.via_capi.renderers_.begin()->second;
  EXPECT_FLOAT_EQ(renderer.intensity_factor_, 1.0f);
  EXPECT_FLOAT_EQ(renderer.intensity_factor_, p.core.renderers_.begin()->second.intensity_factor_);
}

// --- render.grid.longitude: the third grid family (v4.18) ---
//
// The corpus is structurally blind to a new key: no shipped config carries "grid.longitude", so
// a decoder that never reads it round-trips every corpus document perfectly. Only a document that
// actually sets the key can tell the two parsers apart, and the failure this pins is the one the
// two-decoder shape produces — one of them growing the branch and the other not.

std::string WrapRenderWithGrid(const std::string& grid_json) {
  return "{ " + kCrystalBlock + ", " + kFilterBlock + ", " + kMinimalSceneBlock +
         R"(, "render": [ { "id": 1, "resolution": [64, 32], "grid": )" + grid_json + " } ] }";
}

TEST(JsonParserParity, GridLongitudeSurvivesBothParsers) {
  const std::string text = WrapRenderWithGrid(
      R"({ "longitude": [ { "value": -90.0, "width": 1.5, "opacity": 0.3, "color": [1.0, 0.5, 0.25] },
                          { "value": 90.0, "opacity": 0.7 } ],
           "elevation": [ { "value": 30.0, "opacity": 0.5 } ] })");
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(text, &p));
  ASSERT_EQ(p.via_capi.renderers_.size(), 1u);
  const auto& renderer = p.via_capi.renderers_.begin()->second;

  ASSERT_EQ(renderer.longitude_grid_.size(), 2u);
  EXPECT_FLOAT_EQ(renderer.longitude_grid_[0].value_, -90.0f);
  EXPECT_FLOAT_EQ(renderer.longitude_grid_[0].opacity_, 0.3f);
  EXPECT_FLOAT_EQ(renderer.longitude_grid_[0].color_[0], 1.0f);
  EXPECT_FLOAT_EQ(renderer.longitude_grid_[1].value_, 90.0f);
  // Not the parallels' list, and not merged with it — the two keys are decoded independently.
  ASSERT_EQ(renderer.elevation_grid_.size(), 1u);
  EXPECT_FLOAT_EQ(renderer.elevation_grid_[0].value_, 30.0f);

  // The whole renderer, which is what a missed branch on either side actually breaks.
  EXPECT_TRUE(renderer == p.core.renderers_.begin()->second);
}

TEST(JsonParserParity, GridLongitudeOmittedLeavesBothParsersEmpty) {
  const std::string text = Document(kCrystalBlock, kFilterBlock, kMinimalSceneBlock, kMinimalRenderBlock);
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(text, &p));
  EXPECT_TRUE(p.via_capi.renderers_.begin()->second.longitude_grid_.empty());
  EXPECT_TRUE(p.core.renderers_.begin()->second.longitude_grid_.empty());
}

// --- render.grid.zenith_nadir: the marker block (v4.19) ---
//
// Structurally blind in the corpus for the same reason grid.longitude is — no shipped config
// carries the key. What makes this block need MORE than the "present" / "absent" pair those get is
// that its three appearance fields default to NON-ZERO values, so there are three states to
// separate rather than two, and only one of them (the object written out in full) is covered by a
// decoder that merely remembers to read the key.

TEST(JsonParserParity, GridZenithNadirSurvivesBothParsers) {
  const std::string text = WrapRenderWithGrid(
      R"({ "zenith_nadir": { "enabled": true, "radius_px": 14.0, "opacity": 0.25,
                             "color": [0.1, 0.7, 0.9] } })");
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(text, &p));
  ASSERT_EQ(p.via_capi.renderers_.size(), 1u);
  const auto& renderer = p.via_capi.renderers_.begin()->second;

  EXPECT_TRUE(renderer.zenith_nadir_.enabled_);
  EXPECT_FLOAT_EQ(renderer.zenith_nadir_.radius_px_, 14.0f);
  EXPECT_FLOAT_EQ(renderer.zenith_nadir_.opacity_, 0.25f);
  EXPECT_FLOAT_EQ(renderer.zenith_nadir_.color_[1], 0.7f);

  // The whole renderer, which is what a missed branch on either side actually breaks.
  EXPECT_TRUE(renderer == p.core.renderers_.begin()->second);
}

TEST(JsonParserParity, GridZenithNadirPartialObjectAgreesOnTheOmittedDefaults) {
  // The state the "present" / "absent" pair cannot reach: the key exists, switches the marker on,
  // and says nothing about the appearance. Core's from_json leaves its member defaults (8 px, 0.6,
  // {0.8, 0.2, 0.2}); a C API decoder that reset the struct first would answer 0 / 0 / black, and
  // a config would then render differently depending on which parser read it.
  const std::string text = WrapRenderWithGrid(R"({ "zenith_nadir": { "enabled": true } })");
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(text, &p));
  ASSERT_EQ(p.via_capi.renderers_.size(), 1u);
  const auto& renderer = p.via_capi.renderers_.begin()->second;

  EXPECT_TRUE(renderer.zenith_nadir_.enabled_);
  EXPECT_FLOAT_EQ(renderer.zenith_nadir_.radius_px_, 8.0f);
  EXPECT_FLOAT_EQ(renderer.zenith_nadir_.opacity_, 0.6f);
  EXPECT_FLOAT_EQ(renderer.zenith_nadir_.color_[0], 0.8f);
  EXPECT_TRUE(renderer == p.core.renderers_.begin()->second);
}

TEST(JsonParserParity, GridZenithNadirOmittedLeavesBothParsersOnTheDefaults) {
  const std::string text = Document(kCrystalBlock, kFilterBlock, kMinimalSceneBlock, kMinimalRenderBlock);
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(text, &p));
  const lumice::ZenithNadirParam kDefaults;
  EXPECT_TRUE(p.via_capi.renderers_.begin()->second.zenith_nadir_ == kDefaults);
  EXPECT_TRUE(p.core.renderers_.begin()->second.zenith_nadir_ == kDefaults);
}

// --- render.front: the second clip dimension (v4.20) ---
//
// Its own top-level key, deliberately not a fourth "visible" enumerator. The negative half of that
// decision is pinned by RendererVisibleUnknownStringRejected in test_c_api.cpp (the C API refuses
// an unregistered visible string rather than letting nlohmann map it to "upper"); what is pinned
// here is the positive half — both parsers read and write the real key, and agree.

std::string WrapRenderWithFront(const std::string& front_json) {
  return "{ " + kCrystalBlock + ", " + kFilterBlock + ", " + kMinimalSceneBlock +
         R"(, "render": [ { "id": 1, "resolution": [64, 32], "visible": "full", "front": )" + front_json + " } ] }";
}

TEST(JsonParserParity, FrontSurvivesBothParsers) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapRenderWithFront("true"), &p));
  ASSERT_EQ(p.via_capi.renderers_.size(), 1u);
  const auto& renderer = p.via_capi.renderers_.begin()->second;

  EXPECT_TRUE(renderer.front_);
  // ...and it did not arrive by collapsing into `visible`, which is the failure mode this key
  // exists to avoid.
  EXPECT_EQ(renderer.visible_, lumice::RenderConfig::kFull);
  EXPECT_TRUE(renderer == p.core.renderers_.begin()->second);
}

TEST(JsonParserParity, FrontOmittedLeavesBothParsersUnclipped) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(Document(kCrystalBlock, kFilterBlock, kMinimalSceneBlock, kMinimalRenderBlock), &p));
  EXPECT_FALSE(p.via_capi.renderers_.begin()->second.front_);
  EXPECT_FALSE(p.core.renderers_.begin()->second.front_);
}

TEST(JsonParserParity, FrontFalseIsNotConfusedWithFrontMissing) {
  // Both mean "no clip", so this cannot be caught by comparing the parsed value — what it pins is
  // that an explicit false is ACCEPTED by both rather than rejected as a type error by one.
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapRenderWithFront("false"), &p));
  EXPECT_FALSE(p.via_capi.renderers_.begin()->second.front_);
  EXPECT_TRUE(p.via_capi.renderers_.begin()->second == p.core.renderers_.begin()->second);
}

// --- render.background: sRGB on the wire, linear in the struct ---
//
// The JSON key is authored in sRGB; both parsers convert to linear on decode and back on encode.
// Every corpus config writes [0, 0, 0], which is a FIXED POINT of both directions, so the corpus
// sweep above is structurally blind to this conversion — a non-zero value is the only input that
// can tell a converting parser from a passthrough one. The round-trip funnel the other pins use
// (ParseWithBoth) is blind to a one-sided miss for a second reason: it re-encodes the C API's
// result and hands it back to CORE, so a C API that neither decodes nor encodes cancels itself
// out and lands on the same value core-direct would. Hence the two direct-read pins below, one
// per parser, before the round-trip pin.

// A render block carrying an explicit background, everything else minimal.
std::string WrapRenderWithBackground(const std::string& background_json) {
  return "{ " + kCrystalBlock + ", " + kFilterBlock + ", " + kMinimalSceneBlock +
         R"(, "render": [ { "id": 1, "resolution": [64, 32], "background": )" + background_json + " } ] }";
}

// Well clear of both gamma segments' endpoints, so a passthrough parser and a converting one are
// far apart in every channel.
constexpr float kBackgroundSrgb[3] = { 0.2f, 0.35f, 0.6f };
const std::string kBackgroundSrgbJson = "[0.2, 0.35, 0.6]";

TEST(JsonParserParity, BackgroundNonZeroCoreConvertsSrgbToLinear) {
  CoreOutcome core = ParseWithCore(WrapRenderWithBackground(kBackgroundSrgbJson));
  ASSERT_TRUE(core.ok) << core.error;
  ASSERT_EQ(core.config.renderers_.size(), 1u);
  const auto& renderer = core.config.renderers_.begin()->second;
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(renderer.background_[j], lumice::SrgbToLinear(kBackgroundSrgb[j]), 1e-6f)
        << "channel " << j << ": core parser stored the sRGB value verbatim instead of converting";
  }
}

TEST(JsonParserParity, BackgroundNonZeroCapiConvertsSrgbToLinear) {
  ConfigScratch cfg{};
  ConfigScratchGuard guard(cfg);
  ASSERT_EQ(ParseConfigString(WrapRenderWithBackground(kBackgroundSrgbJson).c_str(), &cfg), LUMICE_OK);
  ASSERT_EQ(cfg.renderer_count, 1);
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(cfg.renderers[0].background[j], lumice::SrgbToLinear(kBackgroundSrgb[j]), 1e-6f)
        << "channel " << j << ": C API parser stored the sRGB value verbatim instead of converting";
  }
}

TEST(JsonParserParity, BackgroundNonZeroRoundTripsIdenticallyBothParsers) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapRenderWithBackground(kBackgroundSrgbJson), &p));
  ASSERT_EQ(p.via_capi.renderers_.size(), 1u);
  const auto& via_capi = p.via_capi.renderers_.begin()->second;
  const auto& core = p.core.renderers_.begin()->second;
  for (int j = 0; j < 3; j++) {
    EXPECT_NEAR(via_capi.background_[j], core.background_[j], 1e-6f) << "channel " << j;
  }
}

// --- render.background_color: a dead key, warned about rather than dropped in silence ---
//
// Orthogonal to the sRGB/linear parity above and deliberately grouped apart from it: what is
// asserted here is that neither parser stays silent about a key it cannot use.

// Captures everything the global logger emits for the lifetime of the object. RAII rather than a
// manual remove_sink, because GetSharedSink() is a process-wide singleton: an ASSERT_* returning
// early with the sink still attached would leave later tests in this binary writing into a
// destroyed ostringstream.
class LogCapture {
 public:
  LogCapture() : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(oss_)) {
    lumice::GetSharedSink()->add_sink(sink_);
  }

  ~LogCapture() { lumice::GetSharedSink()->remove_sink(sink_); }

  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  std::string Text() const { return oss_.str(); }

 private:
  std::ostringstream oss_;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;
};

// A render block carrying only the misspelling, so the warning is the sole observable effect.
std::string RenderWithDeadBackgroundColorKey() {
  return "{ " + kCrystalBlock + ", " + kFilterBlock + ", " + kMinimalSceneBlock +
         R"(, "render": [ { "id": 1, "resolution": [64, 32], "background_color": [0.2, 0.35, 0.6] } ] })";
}

TEST(JsonParserParity, BackgroundColorDeadKeyWarnsViaCoreParser) {
  std::string log;
  CoreOutcome core;
  {
    LogCapture capture;
    core = ParseWithCore(RenderWithDeadBackgroundColorKey());
    log = capture.Text();
  }
  ASSERT_TRUE(core.ok) << "the dead key must be ignored, not rejected: " << core.error;
  ASSERT_EQ(core.config.renderers_.size(), 1u);
  // Ignored means ignored: the misspelled value does not reach the field.
  for (int j = 0; j < 3; j++) {
    EXPECT_FLOAT_EQ(core.config.renderers_.begin()->second.background_[j], 0.0f);
  }
  EXPECT_NE(log.find("background_color"), std::string::npos) << "core parser log: " << log;
  EXPECT_NE(log.find("\"background\""), std::string::npos)
      << "the warning must name the key that actually works; core parser log: " << log;
}

TEST(JsonParserParity, BackgroundColorDeadKeyWarnsViaCapiParser) {
  ConfigScratch cfg{};
  ConfigScratchGuard guard(cfg);
  std::string log;
  LUMICE_ErrorCode status = LUMICE_OK;
  {
    LogCapture capture;
    status = ParseConfigString(RenderWithDeadBackgroundColorKey().c_str(), &cfg);
    log = capture.Text();
  }
  ASSERT_EQ(status, LUMICE_OK) << "the dead key must be ignored, not rejected";
  ASSERT_EQ(cfg.renderer_count, 1);
  for (int j = 0; j < 3; j++) {
    EXPECT_FLOAT_EQ(cfg.renderers[0].background[j], 0.0f);
  }
  EXPECT_NE(log.find("background_color"), std::string::npos) << "C API parser log: " << log;
  EXPECT_NE(log.find("\"background\""), std::string::npos)
      << "the warning must name the key that actually works; C API parser log: " << log;
}

TEST(JsonParserParity, PyramidWedgeAngleOmittedDefaultsTo28) {
  BothParsed p;
  ASSERT_TRUE(ParseWithBoth(WrapCrystal(R"({ "id": 1, "type": "pyramid", "shape": { "prism_h": 0.5 } })"), &p));
  const auto& param = std::get<lumice::PyramidCrystalParam>(OnlyCrystal(p.via_capi).param_);
  EXPECT_FLOAT_EQ(param.wedge_angle_u_, 28.0f);
  EXPECT_FLOAT_EQ(param.wedge_angle_l_, 28.0f);
  EXPECT_EQ(param, std::get<lumice::PyramidCrystalParam>(OnlyCrystal(p.core).param_));
}

// =============== CLI entry-point spot-check ===============
//
// The end of the chain the CLI will use: file -> LUMICE_SceneFromJsonFile -> LUMICE_CommitScene.
// ray_num is overridden to a token value before committing — this asserts the shipped examples are
// READABLE and COMMITTABLE through the handle path, not that a 450M-ray render completes.
TEST(JsonParserParity, ShippedExamplesCommitThroughTheSceneHandlePath) {
  if (!CorpusAvailable()) {
    GTEST_SKIP();
  }
  const fs::path examples_dir = fs::path(repo_root_dir) / "examples";
  std::vector<fs::path> examples;
  std::error_code ec;
  for (const auto& entry : fs::directory_iterator(examples_dir, ec)) {
    if (entry.is_regular_file() && entry.path().extension() == ".json") {
      examples.push_back(entry.path());
    }
  }
  std::sort(examples.begin(), examples.end());
  ASSERT_FALSE(examples.empty()) << "no example configs found under " << examples_dir;

  LUMICE_ServerConfig server_config{};
  server_config.num_workers = 1;
  LUMICE_Server* server = LUMICE_CreateServerEx(&server_config);
  ASSERT_NE(server, nullptr);

  for (const auto& example : examples) {
    LUMICE_Scene* scene = nullptr;
    EXPECT_EQ(LUMICE_SceneFromJsonFile(example.string().c_str(), &scene), LUMICE_OK)
        << "SceneFromJsonFile rejected " << RelPath(example);
    if (scene == nullptr) {
      continue;
    }
    EXPECT_EQ(LUMICE_SceneSetSimParams(scene, 0, 1000, 4, 0), LUMICE_OK);
    EXPECT_EQ(LUMICE_CommitScene(server, scene, nullptr), LUMICE_OK) << "CommitScene rejected " << RelPath(example);
    LUMICE_SceneDestroy(scene);
  }

  LUMICE_StopServer(server);
  LUMICE_DestroyServer(server);
}

}  // namespace
