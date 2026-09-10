// "a ground with no headroom left says so", asserted on the server/CLI side.
//
// doc/print-mode-subtractive-ink.md §8 (owner decision D6) gives the two tone operators ONE
// degenerate direction each and requires ONE predicate over both. util/contrast_headroom.hpp is that
// predicate and test/unit-correctness/util/test_contrast_headroom.cpp asserts what it computes; what
// this file owns is the half no assertion on the function can see:
//
//   - CommitConfig actually ASKS. The whole point of the CLI arm is that `Lumice -f black-paper.json`
//     has no panel to surface anything in, so an unasked predicate is indistinguishable from no
//     predicate at all.
//   - it asks about the ground the LIVE operator reads, and only that one. A screen document with
//     pitch-black paper and a print document with a blinding sky are both perfectly fine, and both
//     are what a predicate wired to one field would warn about.
//   - the notice does not fail the commit. `Error::IsSuccess()` is the exact signal the CLI turns
//     into its exit code — src/main.cpp's `if (LUMICE_CommitScene(...) != LUMICE_OK) return 1;` —
//     so asserting it here is asserting the exit code, not a proxy for it.
//   - the conversion to the predicate's domain happens. The grounds are LINEAR in core and the
//     predicate is defined on the sRGB encoding; a case placed where the two domains disagree is the
//     only way a missing conversion can go red (see TheThresholdIsReadInTheEncodedDomain below).
//
// Cases drive `lumice::Server::CommitConfig` with a raw document rather than the C API, for the
// reason test_print_mode_colour_exclusions.cpp beside it gives: the judgement lives in CommitConfig,
// and the C API's JSON -> struct -> JSON round trip would put a second decoder in between.

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "server/server.hpp"
#include "util/color_space.hpp"
#include "util/contrast_headroom.hpp"
#include "util/logger.hpp"

namespace lumice {
namespace {

// Captures everything the global sink receives for the object's lifetime. RAII because
// GetSharedSink() is process-wide: an early return with the sink still attached would leave later
// cases in this binary writing into a destroyed ostringstream.
class LogCapture {
 public:
  LogCapture() : sink_(std::make_shared<spdlog::sinks::ostream_sink_mt>(oss_)) { GetSharedSink()->add_sink(sink_); }
  ~LogCapture() { GetSharedSink()->remove_sink(sink_); }
  LogCapture(const LogCapture&) = delete;
  LogCapture& operator=(const LogCapture&) = delete;

  std::string Text() const { return oss_.str(); }

 private:
  std::ostringstream oss_;
  std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink_;
};

// Substrings chosen to be the part that names the FIELD and its law, not the part that explains the
// symptom: the explanation is prose that may be reworded, "its paper is within" is what a user greps
// for and what distinguishes the two wordings from each other.
constexpr const char* kPrintNotice = "its paper is within";
constexpr const char* kScreenNotice = "its background is within";

bool Mentions(const std::string& haystack, const char* needle) {
  return haystack.find(needle) != std::string::npos;
}

// A minimal committable document, zero simulation: every case here is about what CommitConfig decides
// while reading the config, so running rays would only add seconds and a source of flake.
nlohmann::json MakeBaseConfig() {
  nlohmann::json root;
  root["crystal"] =
      nlohmann::json::array({ { { "id", 1 }, { "type", "prism" }, { "shape", { { "height", 1.0f } } } } });
  root["filter"] = nlohmann::json::array();

  nlohmann::json scene;
  scene["light_source"] = {
    { "type", "sun" }, { "altitude", 20.0f }, { "azimuth", 0.0f }, { "diameter", 0.5f }, { "spectrum", "D65" }
  };
  scene["ray_num"] = 1000;
  scene["max_hits"] = 4;
  scene["scattering"] = nlohmann::json::array(
      { { { "prob", 0.0f }, { "entries", nlohmann::json::array({ { { "crystal", 1 }, { "proportion", 1.0f } } }) } } });
  root["scene"] = scene;

  nlohmann::json rn;
  rn["id"] = 1;
  rn["lens"] = { { "type", "fisheye_equal_area" }, { "fov", 180.0f } };
  rn["resolution"] = { 32, 32 };
  rn["view"] = { { "elevation", 0.0f }, { "azimuth", 0.0f }, { "roll", 0.0f } };
  rn["visible"] = "full";
  rn["intensity_factor"] = 1.0f;
  root["render"] = nlohmann::json::array({ rn });
  return root;
}

// The document's colour keys are authored in sRGB (ParseRenderConfig converts to the linear struct
// fields), so a case states its ground in the same units the warning's threshold is stated in.
nlohmann::json WithGrounds(nlohmann::json root, const char* tone, float background_srgb, float paper_srgb) {
  root["render"][0]["tone"] = tone;
  root["render"][0]["background"] = { background_srgb, background_srgb, background_srgb };
  root["render"][0]["paper"] = { paper_srgb, paper_srgb, paper_srgb };
  return root;
}

constexpr float kLevel = 1.0f / 255.0f;
// Comfortably inside / outside the threshold — four 8-bit levels either side of it, so no case turns
// on a rounding question. Those belong to the predicate's own test.
const float kNoHeadroomBright = 1.0f - (kContrastHeadroomWarnLevels - 4) * kLevel;
const float kNoHeadroomDark = (kContrastHeadroomWarnLevels - 4) * kLevel;
const float kAmpleBright = 1.0f - (kContrastHeadroomWarnLevels + 4) * kLevel;
const float kAmpleDark = (kContrastHeadroomWarnLevels + 4) * kLevel;

// Commit `config` into a throwaway server and return everything the log saw while doing it. The
// IsSuccess() expectation is not incidental: it is this file's AC4 assertion, made on every arm
// rather than in one case of its own, because "the notice never fails a commit" is a property of
// every notice and not of one configuration.
std::string CommitAndCaptureLog(const nlohmann::json& config) {
  LogCapture capture;
  Server server(1);
  EXPECT_TRUE(server.CommitConfig(config).IsSuccess())
      << "a ground with no headroom is a legal configuration; the notice must not fail the commit "
         "(src/main.cpp maps exactly this signal to exit code 1)";
  server.Terminate();
  return capture.Text();
}

}  // namespace

// The screen half, and the state the user feedback behind this scrum came from: the sky is nearly
// white, the halo has nowhere to clamp to, and the overlay lines stay perfectly visible.
TEST(ContrastHeadroomWarning, ANearWhiteSkyUnderScreenSaysTheHaloWillBeInvisible) {
  EXPECT_TRUE(
      Mentions(CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "screen", kNoHeadroomBright, 1.0f)), kScreenNotice));

  EXPECT_FALSE(
      Mentions(CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "screen", kAmpleBright, 1.0f)), kScreenNotice))
      << "four 8-bit levels outside the threshold is still headroom; warning there makes the notice noise";
  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "screen", 0.0f, 1.0f)), kScreenNotice))
      << "the shipped default sky must be quiet, or the warning is a permanent fixture";
}

// The print half. Same three arms on the other law, including that the shipped default paper (white)
// is quiet — the field defaults were chosen so that neither degenerate state is one tick-box away
// (doc decision D5), and this is the assertion that the warning agrees with that choice.
TEST(ContrastHeadroomWarning, ANearBlackPaperUnderPrintSaysTheInkWillBeInvisible) {
  EXPECT_TRUE(
      Mentions(CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "print", 0.0f, kNoHeadroomDark)), kPrintNotice));

  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "print", 0.0f, kAmpleDark)), kPrintNotice))
      << "four 8-bit levels outside the threshold is still headroom";
  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "print", 0.0f, 1.0f)), kPrintNotice))
      << "the shipped default paper must be quiet";
}

// The cross arms, which are the reason `tone_` selects the FIELD and not only the direction. Each
// document below is degenerate in the ground the live operator does not read, and is therefore a
// perfectly good configuration: the defaults of the two fields are each other's degenerate value, so
// a predicate wired to one field would warn on every document that merely switched tone.
TEST(ContrastHeadroomWarning, OnlyTheGroundTheLiveOperatorReadsIsJudged) {
  const std::string screen_with_black_paper =
      CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "screen", 0.0f, kNoHeadroomDark));
  EXPECT_FALSE(Mentions(screen_with_black_paper, kPrintNotice))
      << "under screen the paper is not the ground; black paper is not a degenerate state";
  EXPECT_FALSE(Mentions(screen_with_black_paper, kScreenNotice)) << "and the sky it does read is black, i.e. fine";

  const std::string print_with_white_sky =
      CommitAndCaptureLog(WithGrounds(MakeBaseConfig(), "print", kNoHeadroomBright, 1.0f));
  EXPECT_FALSE(Mentions(print_with_white_sky, kScreenNotice))
      << "under print the background is not the ground; a white sky is not a degenerate state";
  EXPECT_FALSE(Mentions(print_with_white_sky, kPrintNotice)) << "and the paper it does read is white, i.e. fine";
}

// The conversion, which is the one thing this side owns that the predicate's own test cannot see:
// core stores both grounds as LINEAR RGB and the predicate is defined on the sRGB encoding. A ground
// chosen where the two domains give opposite verdicts is the only input that can tell a missing
// LinearToSrgb from a present one — and the direction of the failure is silence, so it cannot be
// left to inspection.
//
// Under print, take a paper whose LINEAR value is above the threshold while its sRGB encoding is
// below it. `LinearToSrgb` is expansive at the dark end (slope 12.92 on the straight segment), so
// the encoded value is the LARGER one: the case therefore has to run the other way round, picking a
// paper that is degenerate in linear terms but has ample headroom once encoded. Skipping the
// conversion there produces a warning on a document that is fine.
TEST(ContrastHeadroomWarning, TheThresholdIsReadInTheEncodedDomain) {
  // A paper at 2 linear levels: linear margin 2/255 (under the threshold) but
  // LinearToSrgb(2/255) ~= 0.085, i.e. ~21.7 8-bit levels (well over it).
  const float kPaperLinear = 2.0f * kLevel;
  ASSERT_LT(kPaperLinear, kContrastHeadroomWarnMargin) << "case setup: must be degenerate in LINEAR terms";
  ASSERT_GT(LinearToSrgb(kPaperLinear), kContrastHeadroomWarnMargin)
      << "case setup: must have ample headroom once encoded";

  // The document's "paper" key is sRGB, so to place a chosen LINEAR value the key carries its
  // encoding — which is the same conversion, used here to construct the input rather than to judge
  // it.
  auto root = WithGrounds(MakeBaseConfig(), "print", 0.0f, LinearToSrgb(kPaperLinear));
  EXPECT_FALSE(Mentions(CommitAndCaptureLog(root), kPrintNotice))
      << "this paper only looks degenerate in the linear domain the struct happens to store; judged "
         "where the pixels actually land it has ~21.7 8-bit levels of headroom";
}

}  // namespace lumice
