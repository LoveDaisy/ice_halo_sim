// The composite (raypath-colour) display channel, driven against a real server without a window.
//
// Two things are being pinned here and they are easy to confuse. One is the PAYLOAD contract: when a
// scene carries colour classes the poller must publish a composite payload whose pixels are the ones
// the server just produced, and when it does not it must stay on the xyz path with no cost. The
// other is the DISPLAY-TIME contract: a colour edit, an exposure push or a visibility toggle
// re-renders the picture WITHOUT restarting the simulation — the run is over, and the user is
// changing how its result is shown, not asking for a new one. A regression in the second one looks
// like a regression in nothing at all: the picture updates, and the ray count quietly starts over.
//
// The pure decisions these feed (ShouldUseCompositeUpload, ShouldFireCompositeUpload,
// ShouldDefaultEnableColorsOnOpen, ToggleCompositePreview) are asserted over their whole domain in
// test/composition-correctness/gui/test_run_lifecycle_chain.cpp.
//
// Six siblings stay in test/gui/functional/test_gui_composite_preview.cpp: the four
// *_fences_stale_composite cases (one family pinning the same fence across four document-entry
// paths — three of them need a GL texture or the harness reset, and splitting a family across two
// targets leaves the seam unowned), plus zorder_priority_persists_across_rerun and
// revert_repushes_server_display_state, which pump frames.

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <thread>
#include <vector>

#include "gui/app.hpp"
#include "gui/color_window.hpp"
#include "gui/gui_state.hpp"
#include "gui/gui_state_reconcile.hpp"
#include "gui/server_poller.hpp"
#include "lumice.h"
#include "support/live_server.hpp"

namespace gui = lumice::gui;

using lumice::test::CommitSceneJson;
using lumice::test::kColorSceneJson;
using lumice::test::kMonoSceneJson;
using lumice::test::kTwoColorSceneJson;
using lumice::test::LiveServer;

namespace {

// A composite read, detached from the frame that produced it so it can outlive the acquire and be
// compared against a later one.
struct Composite {
  std::vector<uint8_t> rgb;
  int width = 0;
  int height = 0;
  float p99 = 0.0f;

  // Mean of one channel over a fixed pixel population. Averaging over a population rather than
  // reading one pixel is what makes the brightness assertions survive Monte-Carlo noise.
  double MeanChannel(const std::vector<size_t>& offsets, int channel) const {
    unsigned long long sum = 0;
    for (size_t off : offsets) {
      sum += rgb[off + channel];
    }
    return offsets.empty() ? 0.0 : static_cast<double>(sum) / offsets.size();
  }
};

Composite ReadComposite(LUMICE_Server* server) {
  Composite out;
  LUMICE_RenderResult r[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame(server);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame.get(), r, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(r[0].img_buffer != nullptr);
  if (r[0].img_buffer == nullptr) {
    return out;
  }
  out.width = r[0].img_width;
  out.height = r[0].img_height;
  out.p99 = r[0].composite_p99_y;
  out.rgb.assign(r[0].img_buffer, r[0].img_buffer + static_cast<size_t>(out.width) * out.height * 3);
  return out;
}

bool SameBytes(const Composite& a, const Composite& b) {
  return a.rgb.size() == b.rgb.size() && std::memcmp(a.rgb.data(), b.rgb.data(), a.rgb.size()) == 0;
}

// One display-time class setting. The colour is what the class paints with; visible/solo are the two
// ways a user removes a class from the picture without touching the simulation.
struct ClassDisplay {
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  bool visible = true;
  bool solo = false;
};

void SetClasses(LUMICE_Server* server, std::initializer_list<ClassDisplay> classes, int mode) {
  std::vector<LUMICE_ColorClassDisplay> disp(classes.size());
  size_t i = 0;
  for (const ClassDisplay& c : classes) {
    disp[i].color[0] = c.r;
    disp[i].color[1] = c.g;
    disp[i].color[2] = c.b;
    disp[i].visible = c.visible ? 1 : 0;
    disp[i].solo = c.solo ? 1 : 0;
    ++i;
  }
  EXPECT_EQ(LUMICE_SetRaypathColors(server, disp.data(), static_cast<int>(disp.size()), nullptr, mode), LUMICE_OK);
}

// The two numbers a display-time edit must leave alone. Both are needed and neither implies the
// other: a stealth re-commit bumps the epoch, while a stealth accumulator reset leaves the epoch
// alone and starts the ray count over. Capturing them also asserts the run really did finish, so a
// case that silently ran against a still-running sim cannot pass vacuously.
struct RunAnchor {
  unsigned long long epoch = 0;
  LUMICE_RayCount rays = 0;
};

RunAnchor CaptureCompletedRun(LUMICE_Server* server) {
  RunAnchor a;
  LUMICE_SimLifecycleResult lc{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  a.epoch = lc.epoch;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &a.rays), LUMICE_OK);
  return a;
}

void ExpectNoRestart(LUMICE_Server* server, const RunAnchor& before) {
  LUMICE_SimLifecycleResult lc{};
  EXPECT_EQ(LUMICE_GetSimLifecycle(server, &lc), LUMICE_OK);
  EXPECT_EQ(lc.lifecycle, static_cast<int>(LUMICE_LIFECYCLE_COMPLETED));
  EXPECT_EQ(lc.epoch, before.epoch);
  LUMICE_RayCount rays = 0;
  EXPECT_EQ(LUMICE_GetSimRayCount(server, &rays), LUMICE_OK);
  EXPECT_GE(rays, before.rays);
}

// A private poller that stops itself before the server it polls can die.
class ScopedPoller {
 public:
  ScopedPoller() = default;
  ~ScopedPoller() { p_.Stop(); }
  ScopedPoller(const ScopedPoller&) = delete;
  ScopedPoller& operator=(const ScopedPoller&) = delete;

  gui::ServerPoller* operator->() { return &p_; }

  std::shared_ptr<const gui::PreviewSnapshot> Repoll(LUMICE_Server* server) {
    p_.ResetGenerationForTest();
    p_.PollOnceForTest(server);
    return p_.LoadSnapshot();
  }

 private:
  gui::ServerPoller p_;
};

}  // namespace

// ---- The payload contract, in both directions ----
//
// The two halves are one proposition read forwards and backwards: colour classes present means the
// poller points rgb_buffer at the server's composite and says so; absent means it stays on the xyz
// path and the composite surface reports its null sentinel. Byte-identity against a direct C-API
// read on both sides is what makes this a wiring assertion rather than a plausibility check.
TEST(CompositePreview, ThePayloadCarriesTheCompositeExactlyWhenTheSceneHasColourClasses) {
  struct SceneCase {
    const char* name;
    const char* json;
    bool expect_composite;
  };
  const SceneCase kCases[] = {
    { "a scene with a match-all colour class", kColorSceneJson, true },
    { "the same scene with no colour classes", kMonoSceneJson, false },
  };

  for (const SceneCase& c : kCases) {
    LiveServer srv;
    if (!srv.Run(c.json)) {
      ADD_FAILURE() << c.name << ": server failed to run the scene";
      continue;  // no server to poll for this row; the rest still get checked
    }

    ScopedPoller local;
    auto snap = local.Repoll(srv);
    if (snap == nullptr) {
      ADD_FAILURE() << c.name << ": poll produced no snapshot";
      continue;
    }
    EXPECT_TRUE(snap->valid) << c.name;
    if (snap->payload == nullptr) {
      ADD_FAILURE() << c.name << ": snapshot has no payload";
      continue;
    }
    EXPECT_EQ(snap->payload->is_composite, c.expect_composite) << c.name;
    EXPECT_EQ(snap->payload->rgb_buffer != nullptr, c.expect_composite) << c.name;
    // The xyz lane is populated either way — the auto-EV path reads it regardless of colour.
    EXPECT_TRUE(snap->payload->xyz_buffer != nullptr) << c.name;
    // The payload views the frame's pixels rather than owning a copy, so the frame share is what
    // makes every read below defined at all.
    if (snap->payload->frame == nullptr) {
      ADD_FAILURE() << c.name << ": payload has no backing frame";
      continue;
    }

    const size_t pixels = static_cast<size_t>(snap->payload->width) * snap->payload->height;

    LUMICE_RawXyzResult xyz[2]{};
    lumice::test::ScopedResultFrame frame_xyz(srv);
    EXPECT_EQ(LUMICE_FrameGetRawXyz(frame_xyz.get(), xyz, 1), LUMICE_OK) << c.name;
    if (xyz[0].xyz_buffer == nullptr) {
      ADD_FAILURE() << c.name << ": raw xyz frame has no buffer";
      continue;
    }
    EXPECT_EQ(xyz[0].img_width, snap->payload->width) << c.name;
    EXPECT_EQ(xyz[0].img_height, snap->payload->height) << c.name;
    EXPECT_EQ(std::memcmp(snap->payload->xyz_buffer, xyz[0].xyz_buffer, pixels * 3 * sizeof(float)), 0) << c.name;

    LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
    lumice::test::ScopedResultFrame frame_comp(srv);
    EXPECT_EQ(LUMICE_FrameGetComposite(frame_comp.get(), comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK) << c.name;
    EXPECT_EQ(comp[0].img_buffer != nullptr, c.expect_composite) << c.name;
    if (!c.expect_composite) {
      continue;
    }
    EXPECT_EQ(comp[0].img_width, snap->payload->width) << c.name;
    EXPECT_EQ(comp[0].img_height, snap->payload->height) << c.name;
    EXPECT_EQ(std::memcmp(snap->payload->rgb_buffer, comp[0].img_buffer, pixels * 3), 0) << c.name;
    // ... and the composite is a picture, not a black frame: the class is match-all red.
    unsigned long long sum = 0;
    for (size_t i = 0; i < pixels * 3; ++i) {
      sum += snap->payload->rgb_buffer[i];
    }
    EXPECT_GT(sum, 0u) << c.name;
  }
}

// ---- The composite and the xyz it was rendered from are never mixed across generations ----
//
// The pre-frame code pulled the two through independent C-API calls whose cross-call window was
// near-guaranteed to be crossed by batch churn under an active sim. The result frame closes that for
// every reader at once: both are read off ONE immutable frame, so a cross-generation mix has nowhere
// left to come from. Each churn round arms a fresh dirty — functionally identical to a background
// batch commit landing in the poll window — and must see the generation strictly advance, so a round
// that armed nothing cannot be mistaken for a round that stayed coherent.
TEST(CompositePreview, ACompositeAlwaysPairsWithTheXyzOfItsOwnGeneration) {
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kColorSceneJson));

  ScopedPoller local;
  unsigned long long prev_gen = 0;

  for (int round = 0; round < 4; ++round) {
    SetClasses(srv, { { 0.0f, 0.0f, static_cast<float>(round % 3) / 2.0f } }, LUMICE_COLOR_MODE_DOMINANT);

    LUMICE_RawXyzResult xyz[LUMICE_MAX_RENDER_RESULTS + 1]{};
    LUMICE_RenderResult comp[LUMICE_MAX_RENDER_RESULTS + 1]{};
    lumice::test::ScopedResultFrame frame(srv);
    EXPECT_EQ(LUMICE_FrameGetRawXyz(frame.get(), xyz, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_EQ(LUMICE_FrameGetComposite(frame.get(), comp, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    EXPECT_TRUE(xyz[0].xyz_buffer != nullptr);
    if (comp[0].img_buffer == nullptr) {
      ADD_FAILURE() << "round " << round << ": composite frame has no buffer";
      continue;  // no pixels to compare for this round; the rest still get checked
    }
    EXPECT_GT(xyz[0].snapshot_generation, prev_gen);  // the churn armed a real snapshot
    prev_gen = xyz[0].snapshot_generation;

    // A same-tick second frame must land on the same frozen snapshot, since nothing armed dirty
    // between the two acquires. No defensive copy is taken first: `comp` reads into the frame this
    // scope holds, and holding it is what keeps those bytes valid — which is the property the frame
    // handle exists to provide.
    const size_t nbytes = static_cast<size_t>(comp[0].img_width) * comp[0].img_height * 3;
    LUMICE_RenderResult direct[LUMICE_MAX_RENDER_RESULTS + 1]{};
    lumice::test::ScopedResultFrame frame_direct(srv);
    EXPECT_EQ(LUMICE_FrameGetComposite(frame_direct.get(), direct, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
    if (direct[0].img_buffer == nullptr) {
      ADD_FAILURE() << "round " << round << ": direct re-read has no buffer";
      continue;
    }
    EXPECT_EQ(std::memcmp(comp[0].img_buffer, direct[0].img_buffer, nbytes), 0);

    // And one real poll builds this round's payload out of that same frozen snapshot. Driving
    // PollOnce rather than a populate helper is the stronger assertion, and since the copy-free
    // payload there is no helper left to call anyway.
    local->PollOnceForTest(srv);
    auto snap = local->LoadSnapshot();
    if (snap == nullptr) {
      ADD_FAILURE() << "round " << round << ": poll produced no snapshot";
      continue;
    }
    if (snap->payload == nullptr) {
      ADD_FAILURE() << "round " << round << ": snapshot has no payload";
      continue;
    }
    EXPECT_TRUE(snap->payload->is_composite);
    if (snap->payload->rgb_buffer == nullptr) {
      ADD_FAILURE() << "round " << round << ": payload has no rgb buffer";
      continue;
    }
    EXPECT_EQ(std::memcmp(snap->payload->rgb_buffer, direct[0].img_buffer, nbytes), 0);
  }
}

// ---- A display-time colour edit repaints without restarting the run ----
//
// The mechanism half: the poll after the edit yields a payload whose pixels reflect the new colour
// and match a same-tick direct read, proving the display-time dirty flag was consumed rather than
// lost. The non-restart half: the lifecycle epoch and the accumulated ray count are untouched, which
// is what fails if the wake path ever regresses to a commit.
TEST(CompositePreview, ADisplayTimeColourEditRepaintsWithoutRestartingTheRun) {
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kColorSceneJson));
  const RunAnchor anchor = CaptureCompletedRun(srv);

  ScopedPoller local;
  auto snap_before = local.Repoll(srv);
  ASSERT_TRUE(snap_before != nullptr);
  ASSERT_TRUE(snap_before->payload != nullptr);
  ASSERT_TRUE(snap_before->payload->is_composite);
  ASSERT_TRUE(snap_before->payload->rgb_buffer != nullptr);
  const size_t nbytes = static_cast<size_t>(snap_before->payload->width) * snap_before->payload->height * 3;
  // A real copy, deliberately: this is the one place the case NEEDS pixels detached from the frame,
  // because the whole point is to diff them against a later poll's.
  const std::vector<uint8_t> composite_before(snap_before->payload->rgb_buffer,
                                              snap_before->payload->rgb_buffer + nbytes);
  const unsigned long long serial_before = snap_before->texture_serial;

  SetClasses(srv, { { 0.0f, 0.0f, 1.0f } }, LUMICE_COLOR_MODE_DOMINANT);  // red -> blue
  // The very wake call PushDisplayState invokes in production. It starts a REAL background thread
  // polling this same server, so from here to the read below there would be two consumers driving
  // DoSnapshot concurrently — and that getter family is not safe for two. Its three segments are not
  // jointly atomic: phase 1 consumes snapshot_dirty_ and bumps the generation under one lock, phase
  // 1.5 counts pixels under none, and only phase 2 rebuilds the composite cache under another. A
  // second caller arriving during phase 1.5 finds dirty already false, returns early without waiting
  // for phase 2, and reports the already-bumped generation beside the PREVIOUS generation's pixels
  // — and img_buffer is a borrowed pointer that phase 2's cache swap frees, so the loser may also be
  // reading released memory. That is exactly the failure this case hit on CI.
  //
  // Stop() drains the worker synchronously, collapsing the two consumers back to one: it removes the
  // window structurally rather than narrowing it — no sleep, no retry, no tolerance.
  //
  // The yield is the nail rather than an incidental detail. Without it the woken worker usually
  // never gets scheduled before Stop() lands, so the interleaving this case is supposed to survive
  // is barely exercised and deleting the Stop() would mostly go unnoticed — which is how the flake
  // survived here. A repeat sweep measured the yield to raise detection of a missing Stop() by
  // roughly two orders of magnitude (one machine's one-off sample, not an invariant; it will drift
  // with hardware, pixel count and scheduler). What does NOT drift, and is why the yield stays: it
  // cannot produce a false red — it only hands the CPU to a worker a correct sequence has already
  // serialized away, so a green run never depends on that sample.
  gui::g_server_poller.WakeForRefresh(srv);
  std::this_thread::yield();
  gui::g_server_poller.Stop();

  local->PollOnceForTest(srv);
  auto snap_after = local->LoadSnapshot();
  ASSERT_TRUE(snap_after != nullptr);
  EXPECT_TRUE(snap_after->valid);
  ASSERT_TRUE(snap_after->payload != nullptr);
  EXPECT_TRUE(snap_after->payload->is_composite);
  ASSERT_TRUE(snap_after->payload->rgb_buffer != nullptr);
  // This poll must have MATERIALIZED a payload, not carried the previous one forward. The two ways
  // this case can go stale look identical at the byte level but differ here: a torn read publishes a
  // new payload holding old pixels, whereas a gate rejection republishes the old payload pointer.
  // Asserting the serial separately keeps a future failure self-diagnosing.
  EXPECT_NE(snap_after->texture_serial, serial_before);
  ASSERT_EQ(static_cast<size_t>(snap_after->payload->width) * snap_after->payload->height * 3, nbytes);
  EXPECT_NE(std::memcmp(snap_after->payload->rgb_buffer, composite_before.data(), nbytes), 0);

  LUMICE_RenderResult direct[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_direct(srv);
  EXPECT_EQ(LUMICE_FrameGetComposite(frame_direct.get(), direct, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  ASSERT_TRUE(direct[0].img_buffer != nullptr);
  EXPECT_EQ(std::memcmp(snap_after->payload->rgb_buffer, direct[0].img_buffer, nbytes), 0);

  EXPECT_EQ(snap_after->payload->payload_epoch, anchor.epoch);  // no accumulator reset
  ExpectNoRestart(srv, anchor);
  gui::g_server_poller.Stop();
}

// ---- Exposure moves the picture and nothing else ----
//
// Exposure is a display-time scalar re-baked into the composite on the next read. Two invariants
// travel with it and both are the kind that fail silently: the P99 anchor is computed over UNEXPOSED
// lane values, so it must not move with EV (an anchor that drifted with exposure would make the
// auto-exposure loop chase its own tail), and the mono getter must leave that composite-only field
// alone rather than reporting a plausible number nobody computed.
TEST(CompositePreview, ExposureRebakesTheCompositeWithoutMovingTheAnchorOrTheRun) {
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kColorSceneJson));
  const RunAnchor anchor = CaptureCompletedRun(srv);

  const Composite baseline = ReadComposite(srv);  // EV 0 -> scale 1.0
  ASSERT_FALSE(baseline.rgb.empty());
  EXPECT_GT(baseline.p99, 0.0f);

  LUMICE_RenderResult mono[LUMICE_MAX_RENDER_RESULTS + 1]{};
  lumice::test::ScopedResultFrame frame_mono(srv);
  EXPECT_EQ(LUMICE_FrameGetRender(frame_mono.get(), mono, LUMICE_MAX_RENDER_RESULTS), LUMICE_OK);
  EXPECT_TRUE(mono[0].img_buffer != nullptr);
  EXPECT_EQ(mono[0].composite_p99_y, 0.0f);  // composite-only field, not populated by the mono path

  EXPECT_EQ(LUMICE_SetCompositeExposure(srv, 2.0f), LUMICE_OK);
  const Composite brighter = ReadComposite(srv);
  EXPECT_EQ(brighter.width, baseline.width);
  EXPECT_EQ(brighter.height, baseline.height);
  EXPECT_FALSE(SameBytes(brighter, baseline));
  EXPECT_EQ(brighter.p99, baseline.p99);

  EXPECT_EQ(LUMICE_SetCompositeExposure(srv, -2.0f), LUMICE_OK);
  const Composite dimmer = ReadComposite(srv);
  EXPECT_FALSE(SameBytes(dimmer, baseline));
  EXPECT_FALSE(SameBytes(dimmer, brighter));

  // Re-applying a value already in force is a no-op, not a second re-bake, and a null server is
  // rejected rather than crashed on.
  EXPECT_EQ(LUMICE_SetCompositeExposure(srv, -2.0f), LUMICE_OK);
  EXPECT_TRUE(SameBytes(ReadComposite(srv), dimmer));
  EXPECT_EQ(LUMICE_SetCompositeExposure(nullptr, 0.0f), LUMICE_ERR_NULL_ARG);

  ExpectNoRestart(srv, anchor);  // three re-bakes, still the same run
}

// ---- Hiding a class re-anchors the exposure over what is left ----
//
// The bug this closes: the exposure scalar was sourced from the mono path and never recomputed when
// the participating set changed, so hiding the bright class left the dim one exactly as invisible as
// it had been — the toggle appeared to do nothing. The fix re-derives the anchor over the
// participating classes only, so removing the bright one makes the remaining one brighter.
//
// The two modes fail differently and so are asserted differently. Additive mixes every visible
// class's lane, so the claim is about a whole population's average brightness. Dominant takes an
// argmax, so the claim is about WHICH class owns a pixel — and a mode that merely won by one unit
// would still be a broken picture, hence the second assertion that the winner is clearly lit.

TEST(CompositePreview, HidingAClassReAnchorsTheExposureOverWhatIsLeftInBothCombineModes) {
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kTwoColorSceneJson));
  const RunAnchor anchor = CaptureCompletedRun(srv);

  const ClassDisplay kRed{ 1.0f, 0.0f, 0.0f };
  const ClassDisplay kBlue{ 0.0f, 0.0f, 1.0f };
  const ClassDisplay kRedHidden{ 1.0f, 0.0f, 0.0f, /*visible=*/false };

  SetClasses(srv, { kRed, kBlue }, LUMICE_COLOR_MODE_ADDITIVE);
  const Composite both = ReadComposite(srv);
  ASSERT_FALSE(both.rgb.empty());
  EXPECT_GT(both.p99, 0.0f);

  // The population: every pixel where class 1 has ANY blue signal. Additive only mixes class 1 into
  // blue, so blue_linear = lane[p] * s and the sRGB byte is monotonic in it up to saturation.
  // Bounding to already-blue pixels excludes the class-1-never-hit background, which would otherwise
  // dilute the ratio with a huge 0-vs-0 term.
  std::vector<size_t> probe;
  for (size_t off = 0; off + 2 < both.rgb.size(); off += 3) {
    if (both.rgb[off + 2] > 0) {
      probe.push_back(off);
    }
  }
  ASSERT_GE(probe.size(), 32u);  // class 1 has a non-trivial blue population
  const double blue_before = both.MeanChannel(probe, 2);

  SetClasses(srv, { kRedHidden, kBlue }, LUMICE_COLOR_MODE_ADDITIVE);
  const Composite dim_only = ReadComposite(srv);
  EXPECT_GT(dim_only.p99, 0.0f);
  EXPECT_LT(dim_only.p99, both.p99);  // the participating set shrank, so the anchor dropped
  const double blue_after = dim_only.MeanChannel(probe, 2);
  // The theoretical growth in linear space is the anchor ratio, ~2.7x on this fixture; sRGB gamma
  // compresses that to ~1.6x in byte space (measured 1.72x at 400k rays), with further attenuation
  // as top-decile pixels saturate first. 1.3x is a conservative floor that survives seed-to-seed
  // noise while still ruling out the pre-fix behaviour, where the average stayed roughly constant.
  EXPECT_GT(blue_after, blue_before);
  EXPECT_GE(blue_after, blue_before * 1.3);

  // Restoring must return the picture byte-exactly, not merely approximately: the underlying lanes
  // were never touched, and the scalar is a pure function of the participating set. Asserted per
  // pixel rather than on the mean, which a compensating pair of errors could satisfy.
  SetClasses(srv, { kRed, kBlue }, LUMICE_COLOR_MODE_ADDITIVE);
  const Composite restored = ReadComposite(srv);
  EXPECT_EQ(restored.p99, both.p99);
  for (size_t off : probe) {
    EXPECT_EQ(restored.rgb[off + 2], both.rgb[off + 2]) << "probe offset " << off;
  }

  // ---- The same edit in dominant mode, on the staging this case already built ----
  //
  // Solo the dim class to find a pixel it definitely owns: clearly blue, and with no red in it.
  SetClasses(srv, { kRed, ClassDisplay{ 0.0f, 0.0f, 1.0f, /*visible=*/true, /*solo=*/true } },
             LUMICE_COLOR_MODE_DOMINANT);
  const Composite solo_dim = ReadComposite(srv);
  ASSERT_FALSE(solo_dim.rgb.empty());
  size_t probe_off = solo_dim.rgb.size();
  for (size_t off = 0; off + 2 < solo_dim.rgb.size(); off += 3) {
    if (solo_dim.rgb[off + 2] >= 16 && solo_dim.rgb[off] < 8) {
      probe_off = off;
      break;
    }
  }
  ASSERT_LT(probe_off, solo_dim.rgb.size()) << "the dim class's solo picture has no clearly blue pixel";

  struct DominantCase {
    const char* name;
    bool bright_visible;
    int winner_channel;  // the channel that must dominate at the probe pixel
    int loser_channel;
  };
  const DominantCase kCases[] = {
    // The match-all class wins the argmax at every pixel carrying any signal.
    { "both visible: the bright class owns the pixel", true, 0, 2 },
    // Hidden, so the argmax passes to the dim class — and the re-anchor makes it clearly lit rather
    // than winning by a single unit over a black pixel.
    { "the bright class hidden: the dim one owns it", false, 2, 0 },
  };

  for (const DominantCase& c : kCases) {
    SetClasses(srv, { ClassDisplay{ 1.0f, 0.0f, 0.0f, c.bright_visible }, kBlue }, LUMICE_COLOR_MODE_DOMINANT);
    const Composite comp = ReadComposite(srv);
    if (comp.rgb.empty()) {
      ADD_FAILURE() << c.name << ": composite read produced no pixels";
      continue;  // no pixel to compare for this row; the rest still get checked
    }
    const uint8_t winner = comp.rgb[probe_off + c.winner_channel];
    const uint8_t loser = comp.rgb[probe_off + c.loser_channel];
    EXPECT_GT(winner, loser) << c.name;
    EXPECT_GE(winner, 16) << c.name;
  }

  ExpectNoRestart(srv, anchor);  // visibility toggles are display-time, whatever they repaint
}

// ---- Re-running the same scene at the same exposure produces the same picture ----
//
// The bug: the shared exposure scalar was amplified by 2^EV on top of the display-time push, so a
// second run at EV 2 came back 16x scaled instead of 4x — a four-fold brightness jump on a scene the
// user did not change. Invisible at EV 0, which is why this case pins a non-zero one.
TEST(CompositePreview, RerunningAtTheSameExposureReproducesTheSamePicture) {
  LiveServer srv;
  constexpr float kUserEv = 2.0f;

  ASSERT_TRUE(srv.Run(kColorSceneJson));
  EXPECT_EQ(LUMICE_SetCompositeExposure(srv, kUserEv), LUMICE_OK);
  const Composite first = ReadComposite(srv);
  ASSERT_FALSE(first.rgb.empty());

  // The re-run commits the scene again, which stops and rebuilds the accumulator — the exact path a
  // second press of Run takes. If any layer had baked EV into the committed config, this composite
  // would come back amplified.
  ASSERT_TRUE(srv.Run(kColorSceneJson));
  EXPECT_EQ(LUMICE_SetCompositeExposure(srv, kUserEv), LUMICE_OK);
  const Composite rerun = ReadComposite(srv);
  ASSERT_EQ(rerun.rgb.size(), first.rgb.size());

  // Ratios rather than byte-equality: two independently seeded accumulations reach IDLE at slightly
  // different batch boundaries, so ~10-15% run-to-run noise is expected. [0.8, 1.25] tolerates that
  // while ruling out the ~4x the doubling bug produced. The unexposed anchor is the tighter of the
  // two signals, being independent of EV altogether.
  const auto MeanByte = [](const Composite& c) {
    unsigned long long sum = 0;
    for (uint8_t v : c.rgb) {
      sum += v;
    }
    return static_cast<double>(sum) / c.rgb.size();
  };
  const double mean_first = MeanByte(first);
  const double mean_rerun = MeanByte(rerun);
  ASSERT_GT(mean_first, 0.0);
  EXPECT_GT(mean_rerun / mean_first, 0.8);
  EXPECT_LT(mean_rerun / mean_first, 1.25);
  ASSERT_GT(first.p99, 0.0f);
  EXPECT_GT(rerun.p99 / first.p99, 0.8);
  EXPECT_LT(rerun.p99 / first.p99, 1.25);
}

// ---- Adding a class re-converges instead of stalling ----
//
// Adding a class is a structural edit: the GUI re-commits, the consumer rebuilds its lanes, and the
// preview must reach a composite of the NEW generation. The bounded loop is the assertion — anything
// past the ceiling is a stall rather than a slow convergence, which would mean the carry-forward is
// covering for a payload that never arrives. The signal flags are the paired proof at the truth
// level that both classes really do have rays, so a green picture cannot come from the GUI-side
// cache defaulting to "has signal".
TEST(CompositePreview, AddingAClassReconvergesToANewGenerationsComposite) {
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kColorSceneJson));
  const unsigned long long epoch_before = CaptureCompletedRun(srv).epoch;

  ScopedPoller local;
  auto baseline = local.Repoll(srv);
  ASSERT_TRUE(baseline != nullptr);
  ASSERT_TRUE(baseline->payload != nullptr);
  ASSERT_TRUE(baseline->payload->is_composite);

  ASSERT_EQ(CommitSceneJson(srv, kTwoColorSceneJson), LUMICE_OK);

  // 300 polls x 10ms is a 3s ceiling; this fixture converges in well under 500ms on a release build.
  bool converged = false;
  for (int poll = 0; poll < 300 && !converged; ++poll) {
    local->PollOnceForTest(srv);
    auto snap = local->LoadSnapshot();
    converged = snap != nullptr && snap->valid && snap->payload != nullptr && snap->payload->is_composite &&
                snap->payload->rgb_buffer != nullptr && snap->payload->payload_epoch > epoch_before;
    if (!converged) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
  EXPECT_TRUE(converged);

  int flags[2] = { -1, -1 };
  EXPECT_EQ(LUMICE_GetColorClassSignal(srv, flags, 2), LUMICE_OK);
  EXPECT_EQ(flags[0], 1);
  EXPECT_EQ(flags[1], 1);
}

// ---- Opening a document fences the previous one's composite out of the upload gate ----
//
// The defect: after a New/Open reset the bookkeeping is back at zero (no serial uploaded, no floor,
// last upload not composite), and against a poller still holding the previous document's staged
// snapshot that combination reads as "a fresh composite nobody has shown yet" — so the new document
// opens showing the old one's picture. Dropping the staged payload is what closes it, and the gate
// then collapses because the upload term requires a payload at all.
TEST(CompositePreview, InvalidatingTheStagedTextureClosesTheFireGateAfterADocumentReset) {
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kColorSceneJson));

  ScopedPoller local;
  auto snap = local.Repoll(srv);
  ASSERT_TRUE(snap != nullptr);
  ASSERT_TRUE(snap->payload != nullptr);
  ASSERT_TRUE(snap->payload->is_composite);
  ASSERT_GT(snap->texture_serial, 0u);
  ASSERT_GT(snap->payload->payload_epoch, 0u);

  // The post-reset bookkeeping, verbatim. This is the bug shape: the gate is open.
  EXPECT_TRUE(gui::ShouldFireCompositeUpload(*snap, /*last_uploaded_texture_serial=*/0,
                                             /*display_epoch_floor=*/0, /*show_composite_preview=*/true,
                                             /*last_uploaded_as_composite=*/false));

  local->InvalidateStagedTexture();
  auto after = local->LoadSnapshot();
  ASSERT_TRUE(after != nullptr);
  EXPECT_TRUE(after->payload == nullptr);                  // the fence took effect: payload dropped...
  EXPECT_EQ(after->texture_serial, snap->texture_serial);  // ... and the serial deliberately did not move
  EXPECT_FALSE(gui::ShouldFireCompositeUpload(*after, /*last_uploaded_texture_serial=*/0,
                                              /*display_epoch_floor=*/0, /*show_composite_preview=*/true,
                                              /*last_uploaded_as_composite=*/false));
}

// ---- The reconciler's display push is the direct push ----
//
// Two paths reach the server with a display-time edit: the per-frame reconciler diffing GuiState, and
// a direct PushDisplayState call. They must produce byte-identical pictures, or the same user action
// means different things depending on which one ran. Both funnel through PushDisplayState with the
// same GuiState, so any drift would mean the reconciler mutates state between diff and push, or
// routes the edit somewhere other than the display-push branch.
//
// The sanity gate is what keeps the equivalence from being vacuous: each mutation must first be shown
// to change the picture at all. An invisible mutation would make "both paths agree" true of two
// identical no-ops.
TEST(CompositePreview, TheReconcilerDrivenDisplayPushMatchesADirectPushByteForByte) {
  LiveServer srv;
  ASSERT_TRUE(srv.Run(kTwoColorSceneJson));

  // A GuiState mirror of the committed scene's display defaults. PushDisplayState reads only
  // {color, visible, solo, z_order, raypath_color_mode} — match/combine are structural-tier and not
  // on this channel — so the structural fields need not be JSON-perfect here.
  auto MakeBaseline = [] {
    gui::GuiState s;
    s.raypath_color.resize(2);
    s.raypath_color[0].color[0] = 1.0f;  // red, match-all
    s.raypath_color[0].visible = true;
    s.raypath_color[0].z_order = 0;
    s.raypath_color[1].color[2] = 1.0f;  // blue, entry_exit len 3
    s.raypath_color[1].visible = true;
    s.raypath_color[1].z_order = 1;
    s.raypath_color_mode = LUMICE_COLOR_MODE_DOMINANT;
    return s;
  };

  // Seed last_pushed_display_state so the reconcile below sees a DIFF edge rather than the
  // first-push-after-reset edge; the mutation is what must create the diff.
  auto SeedDisplayBaseline = [](gui::GuiState& s) {
    gui::GuiState::DisplayStateBaseline dsb;
    for (const auto& cls : s.raypath_color) {
      dsb.color_display.push_back(static_cast<const gui::ColorClassDisplayState&>(cls));
    }
    dsb.raypath_color_mode = s.raypath_color_mode;
    s.last_pushed_display_state = std::move(dsb);
  };

  // Every PushDisplayState wakes the global poller's worker, which would then race the synchronous
  // composite read below over DoSnapshot — the same two-consumer hazard the colour-edit case above
  // documents at length. Stopping first collapses it to one consumer; the next push re-wakes the
  // worker, so no display-time behaviour is lost.
  auto Read = [&] {
    gui::g_server_poller.Stop();
    return ReadComposite(srv);
  };

  const gui::GuiState baseline = MakeBaseline();
  EXPECT_TRUE(gui::PushDisplayState(baseline, srv));
  const Composite composite_baseline = Read();
  ASSERT_FALSE(composite_baseline.rgb.empty());

  struct Mutation {
    const char* name;
    void (*apply)(gui::GuiState&);
  };
  const Mutation kMutations[] = {
    { "dim the bright class", [](gui::GuiState& s) { s.raypath_color[0].color[0] = 0.25f; } },
    { "hide the bright class", [](gui::GuiState& s) { s.raypath_color[0].visible = false; } },
    { "solo the dim class", [](gui::GuiState& s) { s.raypath_color[1].solo = true; } },
    { "reorder the layers",
      [](gui::GuiState& s) {
        // Dominant mode is invariant to z_order (the argmax wins regardless of stacking) but painter
        // mode is not, so the two move together to make this lane observable at all.
        s.raypath_color[0].z_order = 1;
        s.raypath_color[1].z_order = 0;
        s.raypath_color_mode = LUMICE_COLOR_MODE_PAINTER;
      } },
    { "switch the combine mode", [](gui::GuiState& s) { s.raypath_color_mode = LUMICE_COLOR_MODE_ADDITIVE; } },
  };

  for (const Mutation& m : kMutations) {
    // Path A: through the reconciler.
    EXPECT_TRUE(gui::PushDisplayState(baseline, srv));
    EXPECT_TRUE(SameBytes(Read(), composite_baseline)) << m.name << ": the reset did not restore the baseline";

    gui::GuiState state_a = MakeBaseline();
    SeedDisplayBaseline(state_a);
    m.apply(state_a);
    gui::GuiEffects effects = gui::ReconcileGuiEffects(state_a);
    // Every mutation touches a display-tier field, so it must route to the display push and to
    // nothing else — a structural reset or a re-sim here would throw the run away over a colour edit.
    EXPECT_TRUE(effects.need_display_push) << m.name;
    EXPECT_FALSE(effects.need_hard_reset) << m.name;
    EXPECT_FALSE(effects.need_resim) << m.name;
    gui::ApplyGuiEffects(state_a, srv, effects);
    const Composite composite_a = Read();
    EXPECT_FALSE(SameBytes(composite_a, composite_baseline)) << m.name << ": the mutation rendered nothing";

    // Having pushed, a second reconcile with no further edit must be quiet — the channel is
    // edge-triggered, so a reconciler that forgot to record what it pushed would push every frame.
    EXPECT_TRUE(state_a.last_pushed_display_state.has_value()) << m.name;
    EXPECT_FALSE(gui::ReconcileGuiEffects(state_a).need_display_push) << m.name;

    // Path B: straight to PushDisplayState.
    EXPECT_TRUE(gui::PushDisplayState(baseline, srv));
    gui::GuiState state_b = MakeBaseline();
    m.apply(state_b);
    EXPECT_TRUE(gui::PushDisplayState(state_b, srv));
    EXPECT_TRUE(SameBytes(Read(), composite_a)) << m.name;
  }

  gui::g_server_poller.Stop();
}
