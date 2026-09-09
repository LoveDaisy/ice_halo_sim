// "print takes over the colour channel", asserted on the server/CLI side.
//
// doc/print-mode-subtractive-ink.md §7 states ONE rule — the print operator lays a single neutral
// ink whose only degree of freedom is density, so every field that carries information in a HUE is
// mutually exclusive with it — and lists four instances of it. Two of the four are settled
// elsewhere and are deliberately NOT restated here: that print produces the same bytes whatever
// `ray_color_` and the annotation colours say is test_render_consumer_print_mode.cpp's
// PrintIgnoresRayColor / AnnotationColourDoesNotReachThePaper, which own the pixel half of the
// claim. What this file owns is the half those cannot see:
//
//   - the raypath-colour COMPOSITE is not produced at all under print. That is a structural
//     exclusion rather than a pixel property: there is no frame to compare, because DoSnapshot
//     skips the renderer.
//   - a config whose colour half print will drop SAYS SO, once per commit, on the shared log. A
//     silently-dropped field is the failure this rule exists to prevent, and no pixel assertion
//     anywhere can see a missing log line.
//   - the exclusion and the notice are the SAME judgement. Asserted as a cross-check inside one
//     fixture rather than as two independent cases: two separately-passing assertions would still
//     permit "warned but did not exclude" and "excluded but did not warn", which are exactly the
//     two ways a rule with two implementation points comes apart.
//   - tone=screen is untouched, asserted as the negative arm of every case above rather than
//     assumed from reading the `if`.
//
// The warning cases drive `lumice::Server::CommitConfig` with a raw config document, not the C
// API: CommitConfig is where the judgement lives, and the C API's JSON -> struct -> JSON round trip
// would put a second decoder between the document under test and the code under test.

#include <gtest/gtest.h>
#include <spdlog/sinks/ostream_sink.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "server/server.hpp"
#include "support/live_server.hpp"
#include "support/scoped_result_frame.hpp"
#include "util/logger.hpp"

namespace lumice {
namespace {

// Captures everything the global sink receives for the object's lifetime. RAII for the reason the
// copies in test_render_config.cpp / test_crystal_sync_group.cpp are: GetSharedSink() is a
// process-wide singleton, so an early return with the sink still attached would leave later cases
// in this binary writing into a destroyed ostringstream.
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

// Substrings of the three notices, chosen to be the part that names the FIELD rather than the part
// that explains the operator: the explanation is prose that may be reworded, the field name is the
// thing the user greps for.
constexpr const char* kCompositeNotice = "raypath-colour composite is not produced";
constexpr const char* kRayColorNotice = "sets ray_color";
constexpr const char* kAnnotationNotice = "sets an annotation colour";

bool Mentions(const std::string& haystack, const char* needle) {
  return haystack.find(needle) != std::string::npos;
}

// A minimal committable document. Zero rays: every case here is about what CommitConfig decides
// while reading the config, so a simulation would only add seconds and a source of flake.
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
  rn["background"] = { 0.0f, 0.0f, 0.0f };
  rn["intensity_factor"] = 1.0f;
  root["render"] = nlohmann::json::array({ rn });
  return root;
}

// The three colour-carrying fields, each added to the base document one at a time so a case can
// attribute the notice it sees to the field it set.
nlohmann::json WithColorClass(nlohmann::json root) {
  root["raypath_color"] = { { "mode", "dominant" },
                            { "classes", nlohmann::json::array({ { { "color", { 1.0f, 0.0f, 0.0f } },
                                                                   { "match", nlohmann::json::array({
                                                                                  { { "layer", 0 }, { "crystal", 1 } },
                                                                              }) } } }) } };
  return root;
}

nlohmann::json WithRayColor(nlohmann::json root) {
  root["render"][0]["ray_color"] = { 1.0f, 0.0f, 0.0f };
  return root;
}

nlohmann::json WithAnnotationColour(nlohmann::json root) {
  // A single elevation line whose colour is not GridLineParam's {1,1,1} default. `value` is the
  // one required key of that struct's decoder.
  root["render"][0]["grid"]["elevation"] =
      nlohmann::json::array({ { { "value", 22.0f }, { "color", { 1.0f, 0.0f, 0.0f } } } });
  return root;
}

nlohmann::json WithTone(nlohmann::json root, const char* tone) {
  root["render"][0]["tone"] = tone;
  return root;
}

// Commit `config` into a throwaway server and return everything the log saw while doing it.
std::string CommitAndCaptureLog(const nlohmann::json& config) {
  LogCapture capture;
  Server server(1);
  EXPECT_TRUE(server.CommitConfig(config).IsSuccess());
  server.Terminate();
  return capture.Text();
}

}  // namespace

// Instance 2's notice, and its screen control. The class table is scene-level, so the notice keys
// on "this document configures colour classes AND this renderer is print" — a document with either
// half alone must stay silent, which is the second and third arm below.
TEST(PrintModeColourExclusions, AColourClassConfigUnderPrintSaysTheCompositeIsDropped) {
  EXPECT_TRUE(Mentions(CommitAndCaptureLog(WithTone(WithColorClass(MakeBaseConfig()), "print")), kCompositeNotice));

  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithTone(WithColorClass(MakeBaseConfig()), "screen")), kCompositeNotice))
      << "tone=screen composites normally; there is nothing to warn about";
  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithTone(MakeBaseConfig(), "print")), kCompositeNotice))
      << "print alone drops nothing: no colour class was configured";
}

// Instance 3. The sentinel {-1,-1,-1} means "use the natural spectral colour", so it is the value
// that must NOT warn — the notice is about a user-chosen tint being dropped, not about the field
// being present.
TEST(PrintModeColourExclusions, ARayColorUnderPrintSaysItIsNotRead) {
  EXPECT_TRUE(Mentions(CommitAndCaptureLog(WithTone(WithRayColor(MakeBaseConfig()), "print")), kRayColorNotice));

  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithTone(WithRayColor(MakeBaseConfig()), "screen")), kRayColorNotice))
      << "tone=screen reads ray_color; there is nothing to warn about";
  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithTone(MakeBaseConfig(), "print")), kRayColorNotice))
      << "the {-1,-1,-1} sentinel is 'no tint', not a dropped tint";
}

// Instance 4. Same three arms, on the annotation colour family.
TEST(PrintModeColourExclusions, AnAnnotationColourUnderPrintSaysItIsNotRead) {
  EXPECT_TRUE(
      Mentions(CommitAndCaptureLog(WithTone(WithAnnotationColour(MakeBaseConfig()), "print")), kAnnotationNotice));

  EXPECT_FALSE(
      Mentions(CommitAndCaptureLog(WithTone(WithAnnotationColour(MakeBaseConfig()), "screen")), kAnnotationNotice))
      << "tone=screen draws the line in the colour asked for";
  EXPECT_FALSE(Mentions(CommitAndCaptureLog(WithTone(MakeBaseConfig(), "print")), kAnnotationNotice))
      << "a grid line at the default colour is not a dropped colour";
}

// A print document carrying all three at once must name all three. One merged line would make the
// two instances that happened not to apply read as if they had — which is the reason the
// implementation emits three lines and not one, so it is worth an assertion of its own.
TEST(PrintModeColourExclusions, EachDroppedFieldIsNamedSeparately) {
  const std::string log =
      CommitAndCaptureLog(WithTone(WithAnnotationColour(WithRayColor(WithColorClass(MakeBaseConfig()))), "print"));
  EXPECT_TRUE(Mentions(log, kCompositeNotice));
  EXPECT_TRUE(Mentions(log, kRayColorNotice));
  EXPECT_TRUE(Mentions(log, kAnnotationNotice));
}

namespace {

// kColorSceneJson with the tone switched. That scene is the one built for composite cases
// (scattering.prob 0.0, so a `layer: 0` class actually matches) — see live_server.hpp.
std::string ColorSceneWithTone(const char* tone) {
  auto root = nlohmann::json::parse(test::kColorSceneJson);
  root["render"][0]["tone"] = tone;
  return root.dump();
}

// Runs the scene and reports whether the published frame carries a composite payload.
bool RunAndHasComposite(const char* tone) {
  test::LiveServer server;
  EXPECT_TRUE(server.Run(ColorSceneWithTone(tone).c_str(), 20000));
  test::ScopedResultFrame frame(server.get());
  EXPECT_EQ(frame.err(), LUMICE_OK);
  LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
  EXPECT_EQ(LUMICE_FrameGetComposite(frame.get(), comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  return comp[0].img_buffer != nullptr;
}

}  // namespace

// The exclusion itself, on a real run, together with the cross-check that ties it to the notice.
//
// Both halves are read off ONE arm each rather than from two separate cases, because the property
// that matters is that they agree: `RenderConsumer::ColoredMask()` (what DoSnapshot's loop tests)
// returns the very field `ColorClassTable::referenced_mask_` (what CommitConfig's notice tests),
// broadcast to every consumer at construction — so if these two ever disagree, that identity has
// been broken and this is the case that says so.
TEST(PrintModeColourExclusions, TheCompositeIsExcludedAndTheNoticeAgreesWithIt) {
  bool screen_has_composite = false;
  bool print_has_composite = true;
  std::string screen_log;
  std::string print_log;
  {
    LogCapture capture;
    screen_has_composite = RunAndHasComposite("screen");
    screen_log = capture.Text();
  }
  {
    LogCapture capture;
    print_has_composite = RunAndHasComposite("print");
    print_log = capture.Text();
  }

  EXPECT_TRUE(screen_has_composite) << "tone=screen must still composite — this is the AC4 control";
  EXPECT_FALSE(print_has_composite) << "tone=print must produce no composite for a colour-configured renderer";

  // The cross-check. Each arm's two verdicts are read off the SAME commit, so this says "warned
  // and excluded are one judgement", not "two facts that happen to line up today".
  EXPECT_TRUE(Mentions(print_log, kCompositeNotice)) << "the print arm dropped the composite; it must also say so";
  EXPECT_FALSE(Mentions(screen_log, kCompositeNotice)) << "the screen arm composited; it must not claim otherwise";
}

}  // namespace lumice
