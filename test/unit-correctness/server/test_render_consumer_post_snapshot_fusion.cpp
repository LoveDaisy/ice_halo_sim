// Bit-identical guard for RenderConsumer::PostSnapshot()'s XYZ→uint8 chain.
//
// PostSnapshot() used to run four separate full-buffer passes over a mutable
// scratch buffer (memcpy → scale → per-pixel XYZ→RGB+background → sRGB gamma →
// narrow to uint8). Those passes were fused into one per-pixel loop that keeps
// the intermediates in registers, which removed the scratch buffer entirely.
// Every pass was element-wise, so the fusion reorders no floating-point
// operation and the output must be byte-for-byte unchanged.
//
// This test states that claim as an executable contract: it independently
// recomputes the expected uint8 image from the consumer's own published inputs
// (GetRawXyzResult().xyz_buffer_ + ExposureScale()) using the same low-level
// color primitives PostSnapshot calls (GamutClipXyz / XyzToLinearRgb /
// LinearToSrgb, all in util/color_space.hpp and untouched by the fusion), and
// compares byte-for-byte — never with a tolerance. A tolerance here would
// silently accept exactly the floating-point reordering the fusion must not
// introduce.
//
// PostSnapshot also skips the background on pixels the lens does not image, so the
// re-derivation below carries its own domain/visibility predicate (PixelImagesSky). It is
// written out from the projection primitives rather than calling BuildVisibleMask, for the
// same reason the rest of this file does not call render.cpp: a contract test that invokes
// the code under test twice states nothing. This fixture's 16x16 canvas under a 180 deg
// equal-area fisheye puts all four corners outside the image circle (corner radius
// 1.33x the circle's), so the predicate is genuinely exercised, not decorative.
//
// Coverage: the three branch combinations PostSnapshot can take —
//   1. use_real_color (ray_color_[0] < 0) with a zero background;
//   2. the gray + ray_color tint fallback (no gamut clip) with a zero background;
//   3. use_real_color with a non-zero background, which lifts every empty pixel
//      off zero and drives the post-blend clamp on the lit pixel.
// Each case asserts its own coverage rather than assuming it: a non-black image
// (so the byte comparison is not vacuous) and, for case 3, that the post-blend
// clamp actually fired.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <string>
#include <vector>

#include "config/color_class_table.hpp"
#include "config/proj_config.hpp"
#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/geo3d.hpp"
#include "core/projection.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation
#include "server/render.hpp"
#include "util/color_data.hpp"
#include "util/color_space.hpp"

namespace lumice {
namespace {

constexpr float kWl = 550.0f;

RenderConfig MakeSnapshotRenderConfig() {
  RenderConfig cfg;
  cfg.id_ = 0;
  cfg.lens_.type_ = LensParam::kFisheyeEqualArea;
  cfg.lens_.fov_ = 180.0f;
  cfg.resolution_[0] = 16;
  cfg.resolution_[1] = 16;
  cfg.view_.az_ = 0.0f;
  cfg.view_.el_ = 90.0f;
  cfg.view_.ro_ = 0.0f;
  cfg.visible_ = RenderConfig::kUpper;
  // This suite is a byte-level property test over the per-pixel fusion ORDER, which is
  // orthogonal to which exposure formula produced the scale. Pin the absolute anchor so the
  // fixed input keeps producing the fixed bytes these cases were calibrated against.
  cfg.ev_mode_ = RenderConfig::kAbsolute;
  return cfg;
}

// A deterministic batch: every ray points straight up, so the whole weight
// lands on one pixel and the other 255 pixels stay exactly zero. That mix is
// deliberate — the zero pixels exercise the background/clamp path on an
// otherwise black image, the lit pixel exercises the color transform.
SimData MakeUpwardBatch(const std::vector<float>& weights) {
  SimData data;
  data.curr_wl_ = kWl;
  data.outgoing_d_.reserve(weights.size() * 3);
  for (size_t i = 0; i < weights.size(); ++i) {
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(0.0f);
    data.outgoing_d_.push_back(-1.0f);  // sky-up
  }
  data.outgoing_w_ = weights;
  // The normalization denominator. In this fixture nothing is filtered and
  // every ray lands, so the energy emitted equals the energy that arrived — the
  // batch has to declare it either way, because the renderer divides by what was
  // emitted and no longer infers it from what landed.
  data.emitted_energy_ = std::accumulate(weights.begin(), weights.end(), 0.0f);
  return data;
}

// Does pixel `i` image a visible piece of sky? Independent re-derivation of the predicate
// PostSnapshot gates the background on, for this fixture's lens family (single equal-area
// fisheye) only — asserting a general mask belongs in the mask's own tests, not here.
//
// Deliberately complete rather than fixture-shaped: this fixture's `view.el_ = 90` points the
// 180 deg field straight up so `visible = kUpper` excludes nothing inside the circle, but the
// visibility half is still computed. Written the narrow way, changing `el_` here would silently
// stop testing what this function claims to test.
bool PixelImagesSky(const RenderConfig& cfg, int i) {
  const int w = cfg.resolution_[0];
  const int h = cfg.resolution_[1];
  const float short_pix = static_cast<float>(std::min(w, h));
  const float fov_rad = cfg.lens_.fov_ * 3.14159265358979323846f / 180.0f;
  // Equal-area: r = 1 (the inverse's domain edge) sits at theta = 90 deg.
  const float scale = short_pix / 2.0f / std::sqrt(2.0f) / std::sin(fov_rad / 4.0f);

  const float u = (static_cast<float>(i % w) + 0.5f - static_cast<float>(w) / 2.0f) / scale;
  const float v = (static_cast<float>(i / w) + 0.5f - static_cast<float>(h) / 2.0f) / scale;
  const projection::Dir3 c = projection::FisheyeEqualAreaInverse(-u, v, 1.0f);
  if (!c.valid) {
    return false;
  }
  float d[3]{ c.x, c.y, c.z };
  MakeCameraRotation(cfg).Apply(d);
  const float wz = -d[2];
  if (cfg.visible_ == RenderConfig::kUpper && wz > 0.0f) {
    return false;
  }
  if (cfg.visible_ == RenderConfig::kLower && wz < 0.0f) {
    return false;
  }
  return true;
}

// Per-pixel scaled XYZ → linear RGB, i.e. everything PostSnapshot does BEFORE
// the background blend. Shared by the expected-image builder and the
// clamp-coverage probe so the two cannot drift apart.
void ScaledXyzToLinearRgb(const RenderConfig& cfg, const float* xyz_raw, int i, float scale, float rgb[3]) {
  float xyz[3];
  for (int j = 0; j < 3; j++) {
    xyz[j] = xyz_raw[i * 3 + j] * scale;
  }
  if (cfg.ray_color_[0] < 0) {
    float clipped[3];
    GamutClipXyz(xyz, clipped);
    XyzToLinearRgb(clipped, rgb);
  } else {
    float gray[3];
    for (int j = 0; j < 3; j++) {
      gray[j] = kWhitePointD65[j] * xyz[1];
    }
    for (int j = 0; j < 3; j++) {
      float v = 0;
      for (int k = 0; k < 3; k++) {
        v += gray[k] * kXyzToRgb[j * 3 + k];
      }
      rgb[j] = v * cfg.ray_color_[j];
    }
  }
}

// Independent re-derivation of PostSnapshot's per-pixel chain, written out step
// by step from the same primitives the production path calls. Deliberately
// literal (it does not call into render.cpp) so it is an independent statement
// of the formula rather than a second call into the code under test.
std::vector<uint8_t> ExpectedImage(const RenderConfig& cfg, const float* xyz_raw, int total_pix, float scale) {
  std::vector<uint8_t> out(static_cast<size_t>(total_pix) * 3);
  for (int i = 0; i < total_pix; i++) {
    float rgb[3];
    ScaledXyzToLinearRgb(cfg, xyz_raw, i, scale, rgb);
    const bool paint_bg = PixelImagesSky(cfg, i);
    for (int j = 0; j < 3; j++) {
      if (paint_bg) {
        rgb[j] += cfg.background_[j];
      }
      rgb[j] = std::clamp(rgb[j], 0.0f, 1.0f);
      rgb[j] = LinearToSrgb(rgb[j]);
      out[i * 3 + j] = static_cast<uint8_t>(rgb[j] * 255);
    }
  }
  return out;
}

// How many channels the post-blend clamp actually altered. Used to prove the
// clamp branch was exercised — checking the output bytes for 255 cannot do
// that, because LinearToSrgb(1.0f) is 1.055f - 0.055f == 0.99999994f in float,
// so a fully saturated channel narrows to 254 and 255 is unreachable.
size_t CountPostBlendClamps(const RenderConfig& cfg, const float* xyz_raw, int total_pix, float scale) {
  size_t clamped = 0;
  for (int i = 0; i < total_pix; i++) {
    float rgb[3];
    ScaledXyzToLinearRgb(cfg, xyz_raw, i, scale, rgb);
    const float bg_scale = PixelImagesSky(cfg, i) ? 1.0f : 0.0f;
    for (int j = 0; j < 3; j++) {
      const float blended = rgb[j] + cfg.background_[j] * bg_scale;
      if (blended != std::clamp(blended, 0.0f, 1.0f)) {
        ++clamped;
      }
    }
  }
  return clamped;
}

struct Coverage {
  size_t nonzero_bytes = 0;
  size_t clamped_channels = 0;
  size_t unimaged_pixels = 0;
};

// Drives one consumer through a snapshot and asserts the produced image is
// byte-identical to the independently computed expectation. Reports what the
// scene actually covered through `cov` so each caller can assert its own
// coverage. Returns void because it uses ASSERT_*.
void RunAndCompare(const RenderConfig& cfg, const std::vector<float>& weights, const std::string& label,
                   Coverage* cov) {
  *cov = Coverage{};
  RenderConsumer rc(cfg, ColorClassTable{});
  auto data = MakeUpwardBatch(weights);
  rc.Consume(data);
  rc.PrepareSnapshot();
  rc.PostSnapshot();

  const int total_pix = cfg.resolution_[0] * cfg.resolution_[1];
  const auto raw = rc.GetRawXyzResult();
  const float scale = rc.ExposureScale();
  ASSERT_GT(scale, 0.0f) << label << ": exposure scale must be positive or PostSnapshot takes the early-out path";

  const auto expected = ExpectedImage(cfg, raw.xyz_buffer_, total_pix, scale);

  auto result = rc.GetResult();
  const auto* rr = std::get_if<RenderResult>(&result);
  ASSERT_NE(rr, nullptr) << label << ": RenderConsumer must produce a RenderResult";

  size_t nonzero = 0;
  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_EQ(rr->img_buffer_[i], expected[i])
        << label << ": byte " << i << " diverged (pixel " << i / 3 << ", channel " << i % 3
        << ") — the fused loop must not reorder any "
           "floating-point operation";
    if (rr->img_buffer_[i] != 0) {
      ++nonzero;
    }
  }
  cov->nonzero_bytes = nonzero;
  cov->clamped_channels = CountPostBlendClamps(cfg, raw.xyz_buffer_, total_pix, scale);
  for (int i = 0; i < total_pix; ++i) {
    if (!PixelImagesSky(cfg, i)) {
      ++cov->unimaged_pixels;
    }
  }
}

// -----------------------------------------------------------------------------
// 1. use_real_color path (gamut clip → matrix), zero background.
// -----------------------------------------------------------------------------
TEST(RenderConsumerPostSnapshotFusion, RealColorZeroBackground) {
  RenderConfig cfg = MakeSnapshotRenderConfig();
  // ray_color_ keeps its {-1,-1,-1} default → use_real_color.
  Coverage cov;
  RunAndCompare(cfg, { 0.5f, 0.7f, 0.3f, 0.9f }, "RealColorZeroBackground", &cov);
  EXPECT_GT(cov.nonzero_bytes, 0u) << "an all-black image would make the byte comparison vacuous";
}

// -----------------------------------------------------------------------------
// 2. Gray + ray_color tint fallback (no gamut clip), zero background.
// -----------------------------------------------------------------------------
TEST(RenderConsumerPostSnapshotFusion, GrayTintZeroBackground) {
  RenderConfig cfg = MakeSnapshotRenderConfig();
  cfg.ray_color_[0] = 1.0f;  // >= 0 → gray + tint branch
  cfg.ray_color_[1] = 0.5f;
  cfg.ray_color_[2] = 0.2f;
  Coverage cov;
  RunAndCompare(cfg, { 0.5f, 0.7f, 0.3f, 0.9f }, "GrayTintZeroBackground", &cov);
  EXPECT_GT(cov.nonzero_bytes, 0u) << "an all-black image would make the byte comparison vacuous";
}

// -----------------------------------------------------------------------------
// 3. use_real_color + non-zero background: covers `rgb += background` and the
//    [0,1] clamp that follows it.
// -----------------------------------------------------------------------------
TEST(RenderConsumerPostSnapshotFusion, RealColorNonzeroBackground) {
  RenderConfig cfg = MakeSnapshotRenderConfig();
  cfg.background_[0] = 0.9f;
  cfg.background_[1] = 0.25f;
  cfg.background_[2] = 0.4f;
  Coverage cov;
  RunAndCompare(cfg, { 0.5f, 0.7f, 0.3f, 0.9f }, "RealColorNonzeroBackground", &cov);
  EXPECT_GT(cov.nonzero_bytes, 0u) << "an all-black image would make the byte comparison vacuous";
  EXPECT_GT(cov.clamped_channels, 0u) << "the post-blend clamp never fired — retune the background/weights so this "
                                         "scene actually covers the clamp";
  // With a zero background the mask changes nothing, so only this case can show it works.
  // 16x16 under a 180 deg equal-area fisheye leaves 48 corner pixels outside the image circle.
  EXPECT_EQ(cov.unimaged_pixels, 48u) << "no pixel fell outside the lens's domain — this case is what pins that the "
                                         "background is withheld there, and it just stopped covering it";
}

}  // namespace
}  // namespace lumice
