#include "server/render.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <memory>
#include <vector>

#include "config/render_config.hpp"
#include "config/sim_data.hpp"
#include "core/annotation_font.hpp"
#include "core/annotation_overlay.hpp"
#include "core/color_util.hpp"
#include "core/def.hpp"
#include "core/ev_anchor.hpp"
#include "core/lens_proj_build.hpp"
#include "core/math.hpp"
#include "core/raypath.hpp"
#include "core/scatter_accum.hpp"  // MakeCameraRotation (single source of the camera rotation chain)
#include "core/shared/accum_shared.h"
#include "core/shared/projection_shared.h"
#include "util/color_data.hpp"
#include "util/color_space.hpp"


namespace lumice {

// Color transforms (SpectrumToXyz, kNormScale) live in core/color_util.hpp.
// Projection is single-sourced by lm_proj::ProjectExitToPixel (see
// core/shared/projection_shared.h); the host builds its ProjParams via
// BuildProjParams (see core/lens_proj_build.hpp). The old per-type
// GetProjFunc + inline overlap_fwd lambda + Pass 2 hand-rewrite are gone —
// one per-ray call now returns 0/1/2 pixel hits with bump_landed to split
// main from overlap.


namespace {

// The half of an annotation::Request that is the same for every mask this consumer builds: the
// view geometry, straight off the config, and masks-only. The caller adds the one angle list (and,
// for angular_dist, the reference direction) that distinguishes its family. Single-sourced because
// a view field written into one family's request and forgotten in another's would put that
// family's lines on a different projection than the image they are drawn onto.
annotation::Request MakeMaskRequest(const RenderConfig& config) {
  annotation::Request req;
  req.view.width = config.resolution_[0];
  req.view.height = config.resolution_[1];
  req.view.lens_type = config.lens_.type_;
  req.view.fov_deg = config.lens_.fov_;
  req.view.lens_shift[0] = config.lens_shift_[0];
  req.view.lens_shift[1] = config.lens_shift_[1];
  req.view.overlap = config.overlap_;
  req.view.az_deg = config.view_.az_;
  req.view.el_deg = config.view_.el_;
  req.view.roll_deg = config.view_.ro_;
  req.view.visible = config.visible_;
  // The second clip dimension, ANDed with `visible`. It has to be forwarded here and not left at
  // false: BuildVisibleMask clips the background sky by front_, so an annotation that ignored it
  // would draw its line or marker over the half that was clipped away.
  req.view.front = config.front_;
  // Masks only BY DEFAULT. Whether a given call also wants the label anchors is a per-FAMILY
  // question — each family has its own *_label_ switch — so it is left to the caller, exactly as
  // the angle list is, and for the same reason: a request field that varies per family cannot have
  // a correct value here. The curve walk is several times cheaper than the mask sweep, but it is
  // not free, and a family whose labels are switched off must not pay for it.
  req.labels = false;
  return req;
}

// Move one ComputeOverlay call's label anchors into a per-family list, stamping each with the
// index of the config line it annotates.
//
// The rewrite is the point. Core sets Label::index to the position within the REQUEST's angle
// list, and every request built here carries exactly one angle, so what comes back is always 0.
// Merging several such answers into one list without restamping would leave every label claiming
// to belong to the family's first line — and the compositor reads that index to find the colour
// and the opacity to paint it in.
void AppendLabels(const std::vector<annotation::Label>& in, int line_index, std::vector<annotation::Label>& out) {
  out.reserve(out.size() + in.size());
  for (const annotation::Label& label : in) {
    annotation::Label copy = label;
    copy.index = line_index;
    out.push_back(std::move(copy));
  }
}

// The horizon annotation's appearance. Fixed constants, not config: core has no per-annotation
// appearance fields for it (unlike GridLineParam), and inventing config for one line is a wider
// decision than drawing it. The value is the GUI overlay's own horizon default, sRGB {0.8, 0.2,
// 0.2} at alpha 0.6 (gui_state.hpp horizon_color / horizon_alpha).
//
// File scope rather than local to PostSnapshot because the horizon's LINE and its LABELS have to
// be the same colour, and they are painted by two different functions. Two copies of 0.6f would be
// two things free to drift apart.
constexpr float kOutlineAlpha = 0.6f;
constexpr float kOutlineSrgb[3]{ 0.8f, 0.2f, 0.2f };

}  // namespace

// =============== Renderer ===============
RenderConsumer::RenderConsumer(RenderConfig config, ColorClassTable class_table, SunParam sun)
    : config_(std::move(config)),
      short_pix_(static_cast<float>(std::min(config_.resolution_[0], config_.resolution_[1]))),
      internal_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)),
      comp_xyz_(std::make_unique<float[]>(config_.resolution_[0] * config_.resolution_[1] * 3)), sun_(sun),
      class_table_(std::move(class_table)),
      lane_pixel_count_(static_cast<size_t>(config_.resolution_[0]) * static_cast<size_t>(config_.resolution_[1])) {
  // Borrow the first pair up front so the two snapshot getters never see a null
  // buffer before the first PrepareSnapshot/PostSnapshot — the pool zero-fills fresh
  // allocations, matching what the make_unique members these replaced guaranteed.
  const size_t buf_elems = lane_pixel_count_ * 3u;
  snapshot_xyz_ = xyz_pool_->Acquire(buf_elems);
  snapshot_image_buffer_ = image_pool_->Acquire(buf_elems);

  // The camera rotation is core's, not this file's: MakeCameraRotation (core/scatter_accum.hpp)
  // held an exact copy of the chain that used to be spelled out here, and the annotation overlay
  // needs the same one. Three call sites, one derivation.
  rot_ = MakeCameraRotation(config_);

  // Once per consumer, right after rot_ is final — see the member's declaration for why a
  // single build covers the whole lifetime.
  visible_mask_ = BuildVisibleMask(config_, rot_, short_pix_);
  RebuildAngularDistMasks();
  RebuildGridMasks();
  // Where every named reference direction lands, whether or not the config asks to draw it. Not
  // gated on which markers are enabled, and deliberately so — that is an appearance question, so a
  // config that turns one on mid-run arrives through ResetWith with no rebuild, and a table
  // computed only for the flags held here would be missing exactly the entry the user has just
  // asked for. Same argument the horizon mask's declaration makes, and the cost is far smaller:
  // this request carries no angle list and no labels, so ComputeOverlay does the drawable sweep
  // and six forward projections, not a level-set sweep per line.
  RebuildMarkerPoints();
  // Same lifetime argument as the three mask families: the label half depends on an appearance
  // flag that ResetWith can change, so it is (re)built here AND there rather than once. The mask
  // half rides along on the same call, which is what leaves the horizon with ONE computation.
  RebuildHorizonAnnotation();

  // task-339.3: allocate one W*H Y-lane per color class (compact by z-order).
  // Empty class table → no lane state, pre-336 zero heap allocations.
  if (HasColorClasses() && lane_pixel_count_ > 0) {
    size_t class_count = class_table_.classes_.size();
    lane_y_.resize(class_count);
    snapshot_lane_y_.resize(class_count);
    for (size_t i = 0; i < class_count; ++i) {
      lane_y_[i] = std::make_unique<float[]>(lane_pixel_count_);
      snapshot_lane_y_[i] = std::make_unique<float[]>(lane_pixel_count_);
    }
  }
}

const float* RenderConsumer::GetColorClassLaneY(size_t class_idx) const {
  if (class_idx >= snapshot_lane_y_.size()) {
    return nullptr;
  }
  return snapshot_lane_y_[class_idx].get();
}

bool RenderConsumer::HasColorClassSignal(size_t class_idx) const {
  if (class_idx >= snapshot_lane_y_.size()) {
    return false;
  }
  const float* lane = snapshot_lane_y_[class_idx].get();
  if (lane == nullptr) {
    return false;
  }
  for (size_t i = 0; i < lane_pixel_count_; i++) {
    if (lane[i] > 0.0f) {
      return true;
    }
  }
  return false;
}

float RenderConsumer::AxisSolidAngle() const {
  // Rebuilt rather than cached: it is a handful of trig, called once per snapshot, and taking it
  // from the same BuildProjParams the per-ray path uses is what stops it being computed against a
  // fov, a resolution or an overlap the projection did not actually use.
  return ComputeAxisSolidAngle(BuildProjParams(config_, rot_, short_pix_));
}

// task-336.3: single source of truth for the mono-image exposure scale (see
// plan §1.1). PostSnapshot() below calls this so the inline scale and the
// compositor's scale can never drift apart.
float RenderConsumer::ExposureScale() const {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  if (total_pix <= 0) {
    return 0.0f;
  }
  if (config_.ev_mode_ == RenderConfig::kAbsolute) {
    if (snapshot_emitted_energy_ <= 0.0f) {
      return 0.0f;
    }
    return config_.intensity_factor_ * kNormScale * total_pix / snapshot_emitted_energy_;
  }

  // kRelative: anchor to the SCENE's sky radiance, which is the same number the GUI anchors to.
  //
  // The denominator is `Omega_axis(this view) * L99_sky`, and the two factors answer two
  // different questions that used to be answered by one P99 over this renderer's own buffer:
  //   * L99_sky (anchor_l99_sky_) is HOW BRIGHT THE SKY IS — a P99 radiance per steradian over
  //     the session's fixed full-sky equal-area anchor plane, invariant to lens, fov, view,
  //     `visible` and output resolution. Measured once per snapshot by AnchorConsumer and pushed
  //     in by DoSnapshot; see core/anchor_buffer.hpp for why it is a radiance and not a raw P99.
  //   * Omega_axis converts that radiance into the units THIS buffer holds. Each pixel here
  //     accumulated the energy arriving over its own solid angle, so the buffer is `L * Omega_p`;
  //     dividing by the ON-AXIS solid angle leaves `L * m(pos) / L99_sky`, where
  //     m = Omega_p / Omega_axis is exactly the relative illumination the GUI preview shader
  //     applies per pixel (src/gui/preview_jacobian.hpp). That is why the two sides now agree
  //     even though the expressions do not look alike: the GUI's texture holds bare L and its
  //     shader carries m(pos), so its scalar needs no Omega_axis, and the two chains meet on the
  //     same per-pixel product.
  //
  // The GUI reaches its half in two hops -- ComputeEvAuto returns
  // ev = log2(target_linear * snapshot_intensity / anchor), the pipeline consumes it as
  // intensity_factor = 2^ev, and the shader then multiplies raw Y by
  // intensity_scale = intensity_factor / snapshot_intensity. The snapshot_intensity CANCELS
  // between those two hops, leaving an effective per-pixel multiplier of target_linear / anchor.
  // That is why the expression below carries no energy term at all, neither emitted nor landed:
  // an anchor that is itself a statistic over the accumulation cannot contain one. (Same
  // cancellation, same reason, as ParticipatingExposureScale below -- see its comment for the
  // failure mode a naive mirror of ComputeEvAuto's numerator produces.)
  //
  // What is deliberately NOT reproduced is ComputeEvAuto's clamp to [-6, 6] stops. That clamp
  // guards a GUI slider's usable range; applying it here would make the CLI's brightness depend
  // on a UI affordance the CLI does not have.
  if (anchor_l99_sky_ <= 0.0f) {
    return 0.0f;
  }
  const float omega_axis = AxisSolidAngle();
  if (omega_axis <= 0.0f) {
    return 0.0f;
  }
  const float target_linear = TargetWhiteToLinear(kAnchorTargetWhite);
  if (target_linear <= 0.0f) {
    return 0.0f;
  }
  return config_.intensity_factor_ * target_linear / (omega_axis * anchor_l99_sky_);
}

// task-347 (Fix B): composite-path self-anchored exposure scale. See
// declaration comment in render.hpp for the full contract.
//
// Formula derivation: `ComputeEvAuto` returns EV = log2(target_linear ·
// snapshot_intensity / p99_raw_y), and the mono pipeline consumes it as
// `intensity_factor = 2^ev`, then the shader multiplies raw pixel Y by
// `intensity_scale = intensity_factor / snapshot_intensity`. So the effective
// per-pixel multiplier the mono pipeline applies to the raw Y-lane at anchor
// is `target_linear / p99_raw_y` — the snapshot_intensity factor CANCELS.
// The composite pipeline applies our returned `s` directly to lane[p], so `s`
// must reproduce that same effective multiplier without the intermediate
// `pow(2, log2(...))` round-trip. Hence the formula below — no
// snapshot_intensity in the numerator (a naive mirror of ComputeEvAuto's
// numerator would give `s = target_linear · snapshot_intensity / p99`,
// blowing up bytes by snapshot_intensity, which is 10^7 on a 400k-ray fixture).
float RenderConsumer::ParticipatingExposureScale(float participating_p99_y) const {
  if (participating_p99_y <= 0.0f || snapshot_intensity_ <= 0.0f) {
    return 0.0f;
  }
  // target_white on the 0-255 sRGB scale, converted to linear by
  // `core/ev_anchor.hpp::TargetWhiteToLinear` — the same reverse transform
  // ComputeEvAuto uses, now with a single owner rather than a mirrored copy.
  // The 135 itself is `kAnchorTargetWhite`, shared with the kRelative branch of ExposureScale().
  const float target_linear = TargetWhiteToLinear(kAnchorTargetWhite);
  if (target_linear <= 0.0f) {
    return 0.0f;
  }
  return config_.intensity_factor_ * target_linear / participating_p99_y;
}

float RenderConsumer::CompositeAnchorScale(float participating_p99_y) const {
  if (config_.ev_mode_ == RenderConfig::kAbsolute) {
    return ExposureScale();
  }
  return ParticipatingExposureScale(participating_p99_y);
}


void RenderConsumer::ConsumeDeviceFused(const SimData& data) {
  // S1 device-fused: backend already accumulated XYZ on-device; skip
  // projection and fold the pixel buffer into internal_xyz_ via Neumaier.
  auto t0 = std::chrono::steady_clock::now();
  size_t total = static_cast<size_t>(config_.resolution_[0]) * static_cast<size_t>(config_.resolution_[1]) * 3u;
  assert(data.xyz_pixel_data_.size() == total);
  for (size_t i = 0u; i < total; ++i) {
    NeumaierAdd(internal_xyz_[i], comp_xyz_[i], data.xyz_pixel_data_[i]);
  }
  total_intensity_ += data.xyz_landed_weight_;
  total_emitted_energy_ += data.emitted_energy_;
  // task-358.1 Step 4 (AC3): fold the device per-color-class Y-lane accumulator
  // into lane_y_. Layout (matches Metal MSL write side):
  //     lane_pixel_data_[c * (W*H) + (py*W+px)]
  // Simple += (not Neumaier) mirrors CPU AccumulateColorClassLanes at
  // render.cpp:398 — the two are numerically comparable for the AC3 visual
  // parity target. Note: on-device order of atomic_fetch_add per pixel is
  // non-deterministic, so Y values are numerically-close-not-bitwise-identical
  // to CPU — this affects AC3 visual (which is exact-parity-not-required) but
  // NOT AC1 mask parity (mask bits are OR-accumulated → order-independent).
  const size_t pix_wh = static_cast<size_t>(config_.resolution_[0]) * static_cast<size_t>(config_.resolution_[1]);
  const size_t lane_slots = lane_y_.size();
  // Shape check gates the accumulation loop itself (not just an assert) —
  // release builds compile out assert (-DNDEBUG), so a backend/consumer
  // resolution or class_count disagreement must fall through to the warn
  // branch below instead of indexing lane_pixel_data_ with a stride that
  // doesn't match its actual size (code-review-01 Major: this used to be an
  // assert-only guard, i.e. a heap-buffer-overflow read in release).
  const bool lane_shape_ok = !data.lane_pixel_data_.empty() && lane_slots > 0 && data.lane_class_count_ > 0 &&
                             pix_wh > 0 && data.lane_pixel_data_.size() == data.lane_class_count_ * pix_wh;
  if (lane_shape_ok) {
    // Iterate min(server-side lane count, drained class count) — the server
    // sizes lane_y_ from RaypathColorConfig at consumer construction so both
    // paths should agree, but tolerate a smaller drain gracefully rather than
    // walking past the smaller allocation.
    const size_t n_classes = std::min(lane_slots, data.lane_class_count_);
    for (size_t c = 0; c < n_classes; ++c) {
      float* dst = lane_y_[c].get();
      const float* src = data.lane_pixel_data_.data() + c * pix_wh;
      for (size_t p = 0; p < pix_wh; ++p) {
        dst[p] += src[p];
      }
    }
  } else if (HasColorClasses() && !logged_missing_component_) {
    // Backend has raypath_color but delivered no usable lane data — either
    // no lane data at all (backend not extended for device-side lanes yet),
    // or a class_count/resolution mismatch that would have produced a
    // shape-mismatched drain. Log once so a config author sees the
    // regression rather than silently empty/skipped lanes.
    ILOG_WARN(logger_,
              "RenderConsumer: raypath_color configured but this device-fused batch delivered no usable "
              "per-class Y-lane data (lane_pixel_data_ empty or shape mismatch: size={} expected={}). Backend "
              "may not be extended for device-side lane accumulation yet, or class_count/resolution disagree "
              "between backend and consumer.",
              data.lane_pixel_data_.size(), data.lane_class_count_ * pix_wh);
    logged_missing_component_ = true;
  }
  // Count toward the consume profile (proj=0: device did the projection).
  // The batch-invariance positive control reads "Consume profile: N batches"
  // to confirm the commit grain took effect — the device-fused branch must
  // bump the counter or that witness reads 0 / the line never logs.
  consume_count_++;
  consume_accum_us_ += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
}


// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void RenderConsumer::Consume(const SimData& data) {
  if (!data.xyz_pixel_data_.empty()) {
    ConsumeDeviceFused(data);
    return;
  }

  // scrum-258.1: SimData carries a single payload form — outgoing_d_/w_
  // (plus the optional per-ray outgoing_wl_ added in scrum-268.8 (DR-3)) —
  // regardless of whether the simulator ran via the legacy CPU path or a
  // TraceBackend (exit seam). Both converge here and run through the
  // projection pipeline below.
  // Charged up front, before any early-out or ray-dependent branch below: the
  // emitted energy of a batch is a property of the batch, not of how many of its
  // rays survived. A batch whose rays are all filtered away still emitted them,
  // and under an absolute scale it must still enlarge the denominator — that is
  // precisely how a heavily filtered scene ends up correctly darker instead of
  // being re-brightened back to the unfiltered look.
  total_emitted_energy_ += data.emitted_energy_;

  auto t0 = std::chrono::steady_clock::now();
  // Resize pre-allocated buffers if needed (grow-only).
  // Use outgoing count for capacity — it's the upper bound for filtered rays.
  size_t outgoing_count = data.outgoing_w_.size();
  size_t needed = std::max(data.ray_seg_count_, outgoing_count);
  if (needed > buf_capacity_) {
    buf_capacity_ = needed;
    d_buf_ = std::make_unique<float[]>(buf_capacity_ * 3);
    w_buf_ = std::make_unique<float[]>(buf_capacity_);
    xy_buf_ = std::make_unique<int[]>(buf_capacity_ * 2);
    overlap_w_buf_ = std::make_unique<float[]>(buf_capacity_);
    // scrum-268.8 (DR-3): per-ray wavelength side-car, lock-step grow.
    wl_buf_ = std::make_unique<float[]>(buf_capacity_);
    overlap_wl_buf_ = std::make_unique<float[]>(buf_capacity_);
    // task-339.3: per-ray component mask side-cars. Gated on HasColorClasses()
    // so the pre-336 zero-config path stays at zero extra allocation (review
    // Minor #1). Once allocated, they follow the same grow-only capacity as
    // w_buf_ / overlap_w_buf_ / wl_buf_.
    if (HasColorClasses()) {
      comp_buf_ = std::make_unique<uint64_t[]>(buf_capacity_);
      overlap_comp_buf_ = std::make_unique<uint64_t[]>(buf_capacity_);
    }
  }

  // Design A: filter runs simulator-side (see doc/filter-architecture.md §2).
  // All rays in data.outgoing_* have already been filter-gated; the consumer
  // simply projects and accumulates. No per-Consume FilterSpec table needed.
  const auto proj_params = BuildProjParams(config_, rot_, short_pix_);
  // Verbose diagnostic for the dual-fisheye-orthographic no-overlap policy —
  // matches the legacy log message so downstream tooling (docs/tests grep-ing
  // the string) keeps functioning.
  if (config_.overlap_ > 0 && config_.lens_.type_ == LensParam::kDualFisheyeOrthographic) {
    ILOG_VERBOSE(logger_, "Dual Fisheye Orthographic: overlap parameter is ignored in this release");
  }

  // Copy pre-filtered outgoing rays into contiguous buffers (Design A:
  // simulator-side filter has already dropped non-matching rays).
  //
  // The original assertion here checked "batch traced ray segments =>
  // outgoing_w_ non-empty", but that property is NOT something Design A ever
  // promises: the simulator-side filter acts as an emit-gate, so an entire
  // Consume() batch having every candidate ray rejected (outgoing_* empty while
  // ray_seg_count_ is non-zero) is a legitimate tail event, not a malformed state — normal
  // per-batch pass rates are already well under 1% (see
  // doc/filter-architecture.md §4 "Empty-batch contract"). What Design A DOES
  // promise is the outgoing_d_/outgoing_w_ parallel-array sizing invariant
  // documented in sim_data.hpp: three direction floats per weight.
  assert(data.outgoing_d_.size() == 3 * data.outgoing_w_.size());
  size_t filtered_ray_num = outgoing_count;
  if (!data.outgoing_d_.empty()) {
    std::memcpy(d_buf_.get(), data.outgoing_d_.data(), filtered_ray_num * 3 * sizeof(float));
    std::memcpy(w_buf_.get(), data.outgoing_w_.data(), filtered_ray_num * sizeof(float));
  }
  // scrum-268.8 (DR-3): per-ray wavelength is supplied by Metal (and the CPU
  // path when it migrates). Empty vector → legacy per-batch curr_wl_ branch.
  bool per_ray_wl = !data.outgoing_wl_.empty();
  // scrum-268.8 (DR-3) anti-silent-fallback gate: an exit-seam batch carrying
  // outgoing rays but neither per-ray wavelength nor a valid per-batch curr_wl_
  // means the per-ray wl was dropped upstream (e.g. a commit-chunk / transport
  // path that forgot to carry outgoing_wl_, as in a101c53e). The fallback below
  // would silently render a flat, illuminant-independent spectrum — exactly the
  // bug that hid for an entire scrum. Fail loud instead. CPU/legacy and
  // discrete-wl paths set a real curr_wl_ (≥380) and never trip this.
  if (!per_ray_wl && !data.outgoing_d_.empty() && data.curr_wl_ < 1.0f) {
    ILOG_ERROR(logger_,
               "RenderConsumer: {} outgoing rays with no per-ray wavelength and curr_wl_={:.1f} (<1) — "
               "per-ray wavelength was dropped upstream (scrum-268.8)",
               data.outgoing_d_.size() / 3, data.curr_wl_);
    assert(false && "scrum-268.8: per-ray wavelength dropped before consumer");
  }
  if (per_ray_wl) {
    assert(data.outgoing_wl_.size() == filtered_ray_num && "outgoing_wl_ size must match outgoing_w_ when present");
    std::memcpy(wl_buf_.get(), data.outgoing_wl_.data(), filtered_ray_num * sizeof(float));
  }
  // task-339.3: gather per-ray component masks alongside wavelengths.
  bool has_lanes = HasColorClasses();
  bool has_component = has_lanes && !data.outgoing_component_.empty();
  if (has_component) {
    assert(data.outgoing_component_.size() == filtered_ray_num &&
           "outgoing_component_ size must match outgoing_w_ when present");
    std::memcpy(comp_buf_.get(), data.outgoing_component_.data(), filtered_ray_num * sizeof(uint64_t));
  } else if (has_lanes && filtered_ray_num > 0 && !logged_missing_component_) {
    // Non-fatal (issue.md non-goal: GPU device-side lane accumulation is out of
    // scope for 336.2). Log once so silent zero-lane data during a colored
    // config still leaves an actionable trace.
    ILOG_WARN(logger_,
              "RenderConsumer: raypath_color configured but this batch carries no "
              "outgoing_component_ (empty vector) — component lanes will not accumulate for "
              "this batch. Most likely a GPU backend that has not been extended for lane "
              "delivery yet (see scrum-3c).");
    logged_missing_component_ = true;
  }

  // Single-pass per-ray projection: dispatch to ProjectExitToPixel, split
  // hits into "main" (bump_landed=true → drives landed_weight) vs "overlap"
  // (bump_landed=false → dual-write for dual-fisheye seam ring). Each batch
  // then flows through the existing SpectrumToXyz batch interface — Pass 2
  // stays as a second SpectrumToXyz call to preserve the legacy
  // normalization contract (overlap contributions do NOT enter total_intensity_).
  const int w_res = config_.resolution_[0];
  const int h_res = config_.resolution_[1];
  size_t main_n = 0;
  size_t overlap_n = 0;
  float landed_weight = 0.0f;
  for (size_t i = 0; i < filtered_ray_num; ++i) {
    auto hit = lm_proj::ProjectExitToPixel(proj_params, d_buf_[i * 3 + 0], d_buf_[i * 3 + 1], d_buf_[i * 3 + 2]);
    for (int k = 0; k < hit.count; ++k) {
      int px = hit.hits[k].px;
      int py = hit.hits[k].py;
      if (px < 0 || px >= w_res || py < 0 || py >= h_res) {
        continue;
      }
      if (hit.hits[k].bump_landed) {
        xy_buf_[main_n] = py * w_res + px;
        w_buf_[main_n] = w_buf_[i];
        if (per_ray_wl) {
          wl_buf_[main_n] = wl_buf_[i];
        }
        // task-336.2: parallel compaction of the component mask. main_n <= i
        // (same in-place-compaction invariant that lets w_buf_[main_n] =
        // w_buf_[i] be safe here).
        if (has_component) {
          comp_buf_[main_n] = comp_buf_[i];
        }
        landed_weight += w_buf_[i];
        ++main_n;
      } else {
        // Overlap ring uses dedicated side-arrays so main-batch data is not
        // clobbered before it hits SpectrumToXyz.
        overlap_w_buf_[overlap_n] = w_buf_[i];
        if (per_ray_wl) {
          overlap_wl_buf_[overlap_n] = wl_buf_[i];
        }
        if (has_component) {
          overlap_comp_buf_[overlap_n] = comp_buf_[i];
        }
        // Reuse the tail of xy_buf_ for overlap pixels (main uses the head).
        // Safe because filtered_ray_num is the shared upper bound and both
        // main_n and overlap_n are bounded by hit.count * filtered_ray_num.
        xy_buf_[filtered_ray_num + overlap_n] = py * w_res + px;
        ++overlap_n;
      }
    }
  }
  auto t2 = std::chrono::steady_clock::now();

  if (per_ray_wl) {
    SpectrumToXyzPerRay(wl_buf_.get(), w_buf_.get(), xy_buf_.get(), internal_xyz_.get(), main_n);
  } else {
    SpectrumToXyz(data.curr_wl_, w_buf_.get(), xy_buf_.get(), internal_xyz_.get(), main_n);
  }
  total_intensity_ += landed_weight;
  // task-339.3: fan the same batch into per-color-class Y-lanes. Shares the
  // wl / w / xy inputs of the SpectrumToXyz(PerRay) call above → same
  // exposure, same projection, same rounding (issue.md shared-exposure hard
  // invariant).
  if (has_component) {
    AccumulateColorClassLanes(per_ray_wl, wl_buf_.get(), data.curr_wl_, w_buf_.get(), comp_buf_.get(), xy_buf_.get(),
                              main_n);
  }

  if (overlap_n > 0) {
    // Pass 2 does NOT update total_intensity_ — preserves normalization.
    if (per_ray_wl) {
      SpectrumToXyzPerRay(overlap_wl_buf_.get(), overlap_w_buf_.get(), xy_buf_.get() + filtered_ray_num,
                          internal_xyz_.get(), overlap_n);
    } else {
      SpectrumToXyz(data.curr_wl_, overlap_w_buf_.get(), xy_buf_.get() + filtered_ray_num, internal_xyz_.get(),
                    overlap_n);
    }
    // task-339.3: overlap ring lane accumulation, symmetric with main pass.
    if (has_component) {
      AccumulateColorClassLanes(per_ray_wl, overlap_wl_buf_.get(), data.curr_wl_, overlap_w_buf_.get(),
                                overlap_comp_buf_.get(), xy_buf_.get() + filtered_ray_num, overlap_n);
    }
  }

  auto t3 = std::chrono::steady_clock::now();

  consume_count_++;
  consume_proj_us_ += std::chrono::duration<double, std::micro>(t2 - t0).count();
  consume_accum_us_ += std::chrono::duration<double, std::micro>(t3 - t2).count();
}

// task-339.3: per-ray → per-color-class Y scatter. For each ray we evaluate
// every class's predicate (any / all) against the ray's raw component mask and,
// on match, add the ray's Y into that class's lane. Overlap (a ray satisfies
// multiple classes) is intentional: each class independently accumulates,
// which is exactly what makes rule-lanes the source of AND (cross-layer via
// `all`) and overlap (compositor at 339.4).
//
// referenced_mask_ short-circuit: a ray whose masked bits intersect no class
// cannot satisfy any predicate. Safe for `any` (mask & bits == 0 → false) and
// safe for `all` because the empty-member class guard (bits == 0 → skip)
// prevents the vacuous-truth case (mask & 0 == 0 == 0 would otherwise fire).
void RenderConsumer::AccumulateColorClassLanes(bool per_ray_wl, const float* wl_buf, float curr_wl, const float* w_buf,
                                               const uint64_t* comp_buf, const int* xy_buf, size_t num) {
  if (num == 0 || !HasColorClasses()) {
    return;
  }
  uint64_t ref_mask = class_table_.referenced_mask_;
  // Bound by lane_y_.size() (the allocated-lanes source of truth) — NOT
  // class_table_.classes_.size() — to match PrepareSnapshot()/Reset() and stay
  // safe if the two ever decouple (e.g. lane_pixel_count_ == 0 leaves lane_y_
  // empty while HasColorClasses() is still true). lane_y_ is either empty or
  // sized to classes_.size(), so classes_[c] stays in range for every c here.
  size_t class_count = lane_y_.size();
  for (size_t i = 0; i < num; ++i) {
    uint64_t mask = comp_buf[i];
    if ((mask & ref_mask) == 0) {
      continue;
    }
    float wl_i = per_ray_wl ? wl_buf[i] : curr_wl;
    float y = SpectrumToYSingle(wl_i, w_buf[i]);
    if (y == 0.0f) {
      continue;
    }
    size_t pidx = static_cast<size_t>(xy_buf[i]);
    for (size_t c = 0; c < class_count; ++c) {
      const auto& cls = class_table_.classes_[c];
      uint64_t bits = cls.member_bits_;
      if (bits == 0) {
        // Defensive: an empty-member class never contributes energy. Guards
        // the `all` vacuous-truth trap (`mask & 0 == 0 == bits` would be
        // trivially satisfied by every ray). `any` is already safe here
        // (mask & 0 == 0), but treating both branches uniformly removes a
        // silent asymmetry a future reader could easily miss.
        continue;
      }
      bool satisfied = (cls.combine_ == ColorClassCombine::kAny) ? ((mask & bits) != 0) : ((mask & bits) == bits);
      if (satisfied) {
        lane_y_[c][pidx] += y;
      }
    }
  }
}

void RenderConsumer::LogConsumeProfile() const {
  if (consume_count_ == 0) {
    return;
  }
  double avg_proj = consume_proj_us_ / static_cast<double>(consume_count_);
  double avg_accum = consume_accum_us_ / static_cast<double>(consume_count_);
  double avg_total = avg_proj + avg_accum;
  ILOG_INFO(logger_, "Consume profile: {} batches, avg {:.1f}us (proj {:.1f}us {:.0f}% + accum {:.1f}us {:.0f}%)",
            consume_count_, avg_total, avg_proj, avg_proj / avg_total * 100, avg_accum, avg_accum / avg_total * 100);
}

// See doc/ev-pipeline-architecture.md §2.2
// See doc/accumulator-consumer-architecture.md §4.2 (two-phase snapshot protocol, Phase 1).
void RenderConsumer::PrepareSnapshot() {
  size_t total = static_cast<size_t>(config_.resolution_[0]) * config_.resolution_[1] * 3u;
  // Borrow a FRESH buffer for this snapshot instead of rewriting the previous one:
  // whoever holds the previous snapshot (a published ResultFrame, and through it any
  // C-API reader) co-owns that buffer, so writing into it here is precisely the
  // tearing this task removes. The loop below writes every element, so the borrowed
  // buffer's prior contents are irrelevant.
  snapshot_xyz_ = xyz_pool_->Acquire(total);
  // True running sum = internal_xyz_ + comp_xyz_ (Neumaier residual). The
  // device-fused path (ConsumeDeviceFused) accumulates the compensation in
  // comp_xyz_; folding it here is what realizes the precision gain — without
  // this add the compensation would be tracked but never applied (plain +=).
  // comp_xyz_ stays all-zero on the legacy projection path, so this is a no-op
  // there.
  for (size_t i = 0u; i < total; ++i) {
    snapshot_xyz_[i] = internal_xyz_[i] + comp_xyz_[i];
  }
  snapshot_intensity_ = total_intensity_;
  snapshot_emitted_energy_ = total_emitted_energy_;
  // task-339.3: shadow per-class lanes into snapshot_lane_y_ under the same
  // two-phase snapshot protocol. lane_y_ has no Neumaier compensation
  // counterpart (single-precision scatter Y is enough for display), so a plain
  // memcpy matches internal_xyz_'s treatment when comp_xyz_ is all-zero on the
  // legacy path. PostSnapshot is not touched: lanes carry raw Y, no EV/sRGB
  // conversion (compositor multiplies by ExposureScale()).
  for (size_t c = 0; c < lane_y_.size(); ++c) {
    std::memcpy(snapshot_lane_y_[c].get(), lane_y_[c].get(), lane_pixel_count_ * sizeof(float));
  }
}

// See doc/accumulator-consumer-architecture.md §4.2 (Phase 1.5 — runs outside consumer_mutex_).
void RenderConsumer::CountEffectivePixels() {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  int count = 0;
  const float* xyz = snapshot_xyz_.get();
  for (int i = 0; i < total_pix; i++) {
    if (xyz[i * 3] != 0 || xyz[i * 3 + 1] != 0 || xyz[i * 3 + 2] != 0) {
      count++;
    }
  }
  effective_pix_ = std::max(count, 1);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void RenderConsumer::PostSnapshot() {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  // Fresh borrow for the same reason PrepareSnapshot takes one (the previous mono
  // image belongs to the previously published frame). Taken before the early-out below
  // so BOTH exits hand the frame a buffer this snapshot owns.
  snapshot_image_buffer_ = image_pool_->Acquire(static_cast<size_t>(std::max(total_pix, 0)) * 3u);
  if (total_pix <= 0 || snapshot_intensity_ <= 0) {
    std::memset(snapshot_image_buffer_.get(), 0, total_pix * 3);
    return;
  }

  // Intensity scaling uses config_.intensity_factor_ (from CLI JSON / CommitConfig snapshot).
  // GUI rendering uses a separate path: exposure_offset → shader uniform (see app_panels.cpp).
  // task-336.3: the scale expression now lives in ExposureScale() (single source
  // shared with the compositor). Its guard is on the emitted-energy denominator,
  // a different quantity from the snapshot_intensity_ tested above, so ask it for
  // the value and test that — restating its guard here would be two conditions
  // free to drift apart.
  float scale = ExposureScale();
  if (scale <= 0.0f) {
    std::memset(snapshot_image_buffer_.get(), 0, static_cast<size_t>(total_pix) * 3u);
    return;
  }

  bool use_real_color = config_.ray_color_[0] < 0;
  // Defensive: a degenerate resolution leaves the mask empty (BuildVisibleMask's contract).
  // total_pix > 0 is already guaranteed above, so this only differs from `true` if the two
  // ever disagree about the pixel count.
  const bool masked_bg = visible_mask_.size() == static_cast<size_t>(total_pix);
  // The celestial-horizon annotation. Gated here rather than at build time — see the member's
  // declaration. Its colour comes from the file-scope kOutlineSrgb / kOutlineAlpha, converted here
  // because this blend happens in LINEAR RGB — same domain and same place in the chain as the
  // background term below it, which is the repo's standing rule for anything added to radiance
  // before the transfer curve.
  //
  // `config_.horizon_` alone, NOT ORed with horizon_label_: whether the LINE is painted is this
  // flag's own question. The label half is independent of it and is decided in PaintLabels().
  const bool paint_outline_layer = config_.horizon_ && horizon_mask_.size() == static_cast<size_t>(total_pix);
  float outline_rgb[3];
  SrgbToLinearRgb(kOutlineSrgb, outline_rgb);

  // The angular-distance circles and the coordinate grid (parallels + meridians). Unlike the
  // horizon these carry their own appearance (GridLineParam::opacity_ / color_), so the pre-pass
  // below converts each line's colour to linear once instead of per pixel. `width_` is read and
  // round-tripped but has no effect: the mask generator derives its own half-width from the local
  // gradient and takes no width input.
  //
  // The colour is per line, so the masks are per line too, and a pixel on two circles is blended
  // twice — which is what the preview shader's loop already does, in this same order.
  struct LineLayer {
    const uint8_t* mask;
    float rgb[3];
    float alpha;
  };
  // Turn one family's (mask, GridLineParam) pairs into blend-ready layers: drop the degenerate and
  // the fully transparent, convert each line's sRGB colour to linear once. Shared by all three
  // families — they differ only in which list they read, never in how a line becomes a layer.
  const auto collect_layers = [total_pix](const std::vector<std::vector<uint8_t>>& masks,
                                          const std::vector<GridLineParam>& lines, std::vector<LineLayer>& out) {
    for (size_t k = 0; k < masks.size() && k < lines.size(); ++k) {
      if (masks[k].size() != static_cast<size_t>(total_pix)) {
        continue;  // degenerate view: ComputeOverlay returned an empty mask
      }
      const float alpha = std::clamp(lines[k].opacity_, 0.0f, 1.0f);
      if (alpha <= 0.0f) {
        continue;
      }
      LineLayer layer{ masks[k].data(), { 0.0f, 0.0f, 0.0f }, alpha };
      SrgbToLinearRgb(lines[k].color_, layer.rgb);
      out.push_back(layer);
    }
  };

  // The coordinate grid — parallels then meridians, both under the angular-distance circles.
  // One list, not two: the blend loop treats every line identically, and the two families' order
  // relative to each other is unobservable (they share an appearance model and a blend operator).
  std::vector<LineLayer> grid_layers;
  grid_layers.reserve(elevation_masks_.size() + longitude_masks_.size());
  collect_layers(elevation_masks_, config_.elevation_grid_, grid_layers);
  collect_layers(longitude_masks_, config_.longitude_grid_, grid_layers);

  std::vector<LineLayer> angular_dist_layers;
  angular_dist_layers.reserve(angular_dist_masks_.size());
  collect_layers(angular_dist_masks_, config_.angular_dist_grid_, angular_dist_layers);

  // The ring markers, on top of everything else — the layer order the preview shader uses
  // (overlayAuxLines draws them last). No mask: the ring is a circle of a config-named radius
  // around a single point, so the per-pixel test is a distance comparison, which is cheaper than a
  // W*H buffer that would have to be rebuilt whenever the radius moved.
  //
  // ARBITRATION between the two ways a config can ask for markers, and the ONLY place in the tree
  // that decides it. A non-empty markers_ wins outright; zenith_nadir_ is consulted only when
  // markers_ is empty. It happens HERE, at the consumer, rather than in either JSON decoder,
  // because RenderConfig has three producers (this file's own callers, config_manager.cpp,
  // c_api.cpp) and a rule applied by producers is a rule each of them can forget.
  //
  // Not a merge: a config that lists markers is describing its whole marker set, and quietly
  // adding two more rings from a legacy field it also carries would draw something nobody asked
  // for. Emptiness is the only absence signal there is, so an explicit `"markers": []` beside a
  // `"zenith_nadir"` falls back to the legacy pair — a vector cannot tell "key absent" from "key
  // present and empty", and recording a "was the key seen" bit would push a JSON-parsing detail
  // into a struct that two of its three producers build with no JSON at all.
  struct MarkerLayer {
    annotation::CanvasPoint point;
    float rgb[3];
  };
  std::vector<MarkerLayer> marker_layers;
  float marker_alpha = 0.0f;
  float marker_radius_px = 0.0f;
  if (!config_.markers_.empty()) {
    marker_alpha = std::clamp(config_.markers_opacity_, 0.0f, 1.0f);
    marker_radius_px = config_.markers_radius_px_;
    marker_layers.reserve(config_.markers_.size());
    for (const auto& m : config_.markers_) {
      if (!m.enabled_) {
        continue;
      }
      MarkerLayer layer{ marker_points_[static_cast<size_t>(m.id_)], { 0.0f, 0.0f, 0.0f } };
      SrgbToLinearRgb(m.color_, layer.rgb);
      marker_layers.push_back(layer);
    }
  } else if (config_.zenith_nadir_.enabled_) {
    // The legacy pair, expressed in the same terms. Deliberately the SAME arithmetic in the same
    // order as the branch above and as the code this replaced: one clamp of the same opacity, one
    // sRGB conversion of the same colour, then zenith before nadir. That is what makes a
    // zenith_nadir-only config render the identical bytes rather than merely a similar picture —
    // the per-pixel work below is one loop over two entries where it used to be two ifs, and no
    // floating-point operation is added, removed or reordered.
    marker_alpha = std::clamp(config_.zenith_nadir_.opacity_, 0.0f, 1.0f);
    marker_radius_px = config_.zenith_nadir_.radius_px_;
    float rgb[3]{ 0.0f, 0.0f, 0.0f };
    SrgbToLinearRgb(config_.zenith_nadir_.color_, rgb);
    marker_layers.push_back({ marker_points_[annotation::kMarkerZenith], { rgb[0], rgb[1], rgb[2] } });
    marker_layers.push_back({ marker_points_[annotation::kMarkerNadir], { rgb[0], rgb[1], rgb[2] } });
  }
  // A coarse gate on entering the per-pixel work at all; each point's own `valid` is still tested
  // individually below, because opposite world directions cannot both be on a canvas that is not
  // full-sky.
  const bool paint_marker =
      std::any_of(marker_layers.begin(), marker_layers.end(), [](const MarkerLayer& l) { return l.point.valid; });
  // Half the ring's thickness. The same 1.5 px the level-set mask generator lands on for a field
  // that changes by one unit per pixel (LevelSetMaskFromField: clamp(|grad|, 1e-4, 2) * 1.5), and
  // the distance to a fixed point IS such a field — its gradient is a unit vector everywhere. Not
  // a separately chosen number: the markers are the same thickness as every other annotation.
  constexpr float kMarkerHalfWidthPx = 1.5f;
  const int width_px = config_.resolution_[0];
  // Hoisted out of the pixel loop so the ring test costs no allocation per pixel. Sized once;
  // every entry is (re)written before it is read, under the same `paint_marker` guard.
  std::vector<uint8_t> on_marker_ring(marker_layers.size(), 0);

  // One pass per pixel, intermediates kept in registers. This used to be four
  // full-buffer passes (memcpy into a work buffer → scale → color transform →
  // LinearToSrgbBatch → narrow), which existed only because each pass needed a
  // mutable place to leave its output for the next one; snapshot_xyz_ itself
  // must survive untouched for GetRawXyzResult(). Every one of those passes was
  // element-wise, so fusing them reorders no floating-point operation and the
  // bytes produced here are identical to what the four-pass version produced —
  // the property test_render_consumer_post_snapshot_fusion.cpp holds this code
  // to, byte for byte and without tolerance. Keep it that way: each step below
  // must stay a per-element operation on a value that never round-trips through
  // a different precision.
  for (int i = 0; i < total_pix; i++) {
    float xyz[3];
    for (int j = 0; j < 3; j++) {
      xyz[j] = snapshot_xyz_[i * 3 + j] * scale;
    }

    float rgb[3];
    if (use_real_color) {
      // Gamut clip → matrix multiply
      float clipped[3];
      GamutClipXyz(xyz, clipped);
      XyzToLinearRgb(clipped, rgb);
    } else {
      // Skip gamut clip; use D65 gray (luminance-only) → matrix multiply → ray_color tint.
      // Inline matrix multiply (no clamp before ray_color — clamp after bg blending below).
      float gray[3];
      for (int j = 0; j < 3; j++) {
        gray[j] = kWhitePointD65[j] * xyz[1];
      }
      for (int j = 0; j < 3; j++) {
        float v = 0;
        for (int k = 0; k < 3; k++) {
          v += gray[k] * kXyzToRgb[j * 3 + k];
        }
        rgb[j] = v * config_.ray_color_[j];
      }
    }

    // Background blending + clamp, then sRGB gamma and the narrowing write. The
    // gamma call is the scalar LinearToSrgb the old LinearToSrgbBatch looped
    // over element by element (color_space.cpp), not a different formula.
    //
    // The background is added only where the lens actually images visible sky
    // (visible_mask_, built once at construction). Outside that region — beyond the image
    // circle, or in the hemisphere `visible` excludes — painting it the sky colour would turn
    // e.g. a 180 deg fisheye render into a solid rectangle of background with an invisible
    // circle inside it. Clamp, gamma and the narrowing write still run for every pixel, so a
    // masked pixel goes through the identical chain.
    //
    // A masked pixel is also CLEARED of ray energy, below (478.2). Withholding the background
    // is not on its own enough to make `visible` a display clip: beyond the image circle no ray
    // can land, but inside it the excluded hemisphere is imaged normally and its rays deposit
    // energy like any other — measured at 89% (rectangular) to 99.8% (globe) of that region
    // carrying energy. Left alone they show up as lit pixels scattered through a black field.
    const bool paint_bg = masked_bg ? visible_mask_[i] != 0 : true;
    const bool paint_outline = paint_outline_layer && horizon_mask_[i] != 0;
    // Resolved once per pixel rather than per channel: the ring test does not depend on j. The
    // pixel's coordinates are recovered inside the guard so the default (markers off) path pays
    // for neither the division nor the modulo.
    if (paint_marker) {
      const auto px = static_cast<float>(i % width_px);
      const auto py = static_cast<float>(i / width_px);
      for (size_t m = 0; m < marker_layers.size(); ++m) {
        const annotation::CanvasPoint& p = marker_layers[m].point;
        // `valid` first, and per point: a default-constructed CanvasPoint sits at (0, 0), so a
        // point that missed the canvas would otherwise draw a ring in the corner for a direction
        // the picture does not contain.
        on_marker_ring[m] = static_cast<uint8_t>(
            p.valid && std::fabs(std::hypot(px - p.px, py - p.py) - marker_radius_px) < kMarkerHalfWidthPx);
      }
    }
    for (int j = 0; j < 3; j++) {
      if (paint_bg) {
        rgb[j] += config_.background_[j];
      } else if (masked_bg) {
        // SYNC:visible-mask-zero — the display clip. component_compositor.cpp's
        // ApplyCompositeBackground carries the twin of this line for the raypath-colour path;
        // both read the SAME visible_mask_ buffer, so the predicate is single-sourced and only
        // the two applications of it need to stay in step. Guarded by masked_bg so a mask that
        // disagrees with the pixel count still falls back to "paint everything", which is the
        // fallback paint_bg above already takes.
        //
        // Placed before the annotation layers on purpose: a grid line or the horizon is drawn ON
        // the clipped region, over black, exactly as it is drawn over the background elsewhere.
        // The annotations run their own hemisphere policy (annotation_overlay.cpp's
        // VisibleForLabel), so what reaches here has already been admitted.
        rgb[j] = 0.0f;
      }
      // After the background (the line is drawn ON the sky, not under it), before the clamp, and
      // in linear — the same three constraints the background term above satisfies.
      //
      // The layer order is the preview shader's own (grid -> sun circles -> horizon ->
      // zenith/nadir, overlayAuxLines in preview_renderer.cpp); the two paths have to agree about
      // which line wins where they cross.
      for (const auto& layer : grid_layers) {
        if (layer.mask[i] != 0) {
          rgb[j] = rgb[j] * (1.0f - layer.alpha) + layer.rgb[j] * layer.alpha;
        }
      }
      for (const auto& layer : angular_dist_layers) {
        if (layer.mask[i] != 0) {
          rgb[j] = rgb[j] * (1.0f - layer.alpha) + layer.rgb[j] * layer.alpha;
        }
      }
      if (paint_outline) {
        rgb[j] = rgb[j] * (1.0f - kOutlineAlpha) + outline_rgb[j] * kOutlineAlpha;
      }
      // One independent blend per layer rather than one on the union, matching the shader: where
      // two rings overlap it composites both, and so does this. In list order, which for the
      // legacy pair is zenith then nadir — the order the two hardcoded ifs this replaced had.
      if (paint_marker) {
        for (size_t m = 0; m < marker_layers.size(); ++m) {
          if (on_marker_ring[m] != 0) {
            rgb[j] = rgb[j] * (1.0f - marker_alpha) + marker_layers[m].rgb[j] * marker_alpha;
          }
        }
      }
      rgb[j] = std::clamp(rgb[j], 0.0f, 1.0f);
      rgb[j] = LinearToSrgb(rgb[j]);
      snapshot_image_buffer_[i * 3 + j] = static_cast<uint8_t>(rgb[j] * 255);
    }
  }

  // The text, last and outside the loop above. See PaintLabels' declaration for why it is not a
  // stage of that loop.
  PaintLabels();
}

// Every cached label anchor's text, blended into the finished sRGB image. Two halves: resolve each
// label's appearance from the family it belongs to, then rasterize and composite. See the
// declaration in render.hpp for why this is not a stage of PostSnapshot's fused loop.
void RenderConsumer::PaintLabels() {
  const int width_px = config_.resolution_[0];
  const int height_px = config_.resolution_[1];
  if (width_px <= 0 || height_px <= 0 || snapshot_image_buffer_ == nullptr) {
    return;
  }

  // A label plus the appearance it inherits from the family it annotates. The colour and the
  // opacity are the FAMILY's own, never a label-specific value: a grid line at opacity 0 is
  // invisible and so are its numbers, and the GUI has behaved that way since it grew labels at all
  // (overlay_labels.cpp hands every label its family's colour and alpha). Making core independent
  // here would put a divergence between the two renderers back exactly where this layer removed
  // one. See render_config.hpp's *_label_ comment for the full statement.
  struct LabelDraw {
    const annotation::Label* label;
    float rgb[3];
    float alpha;
  };
  std::vector<LabelDraw> draws;

  // Turn one family's (labels, line list) pair into draws, dropping the ones whose line is fully
  // transparent. Mirrors PostSnapshot's collect_layers, and drops for the same reason: an alpha of
  // zero composites to a no-op, and skipping it up front also skips the rasterization.
  const auto collect = [&draws](const std::vector<annotation::Label>& labels, const std::vector<GridLineParam>& lines) {
    for (const annotation::Label& label : labels) {
      const auto index = static_cast<size_t>(label.index);
      if (label.index < 0 || index >= lines.size()) {
        continue;  // a label whose line has since left the config
      }
      const float alpha = std::clamp(lines[index].opacity_, 0.0f, 1.0f);
      if (alpha <= 0.0f) {
        continue;
      }
      LabelDraw draw{ &label, { 0.0f, 0.0f, 0.0f }, alpha };
      SrgbToLinearRgb(lines[index].color_, draw.rgb);
      draws.push_back(draw);
    }
  };
  collect(elevation_labels_, config_.elevation_grid_);
  collect(longitude_labels_, config_.longitude_grid_);
  collect(angular_dist_labels_, config_.angular_dist_grid_);
  // The horizon's, in the line's own fixed colour — it has no GridLineParam to read, and no
  // user-facing opacity knob either.
  {
    float horizon_rgb[3];
    SrgbToLinearRgb(kOutlineSrgb, horizon_rgb);
    for (const annotation::Label& label : horizon_labels_) {
      draws.push_back(LabelDraw{ &label, { horizon_rgb[0], horizon_rgb[1], horizon_rgb[2] }, kOutlineAlpha });
    }
  }
  if (draws.empty()) {
    return;
  }

  for (const LabelDraw& draw : draws) {
    const annotation::TextBitmap ink = annotation::RasterizeLabel(draw.label->text);
    if (ink.Empty()) {
      continue;
    }
    // The anchor is core's canvas pixel, rounded to the grid the glyphs were rasterized on; the
    // bitmap's offsets carry the centring (annotation_font.hpp).
    const int left = static_cast<int>(std::lround(draw.label->px)) + ink.offset_x;
    const int top = static_cast<int>(std::lround(draw.label->py)) + ink.offset_y;
    for (int row = 0; row < ink.height; ++row) {
      const int y = top + row;
      if (y < 0 || y >= height_px) {
        continue;
      }
      for (int col = 0; col < ink.width; ++col) {
        const int x = left + col;
        if (x < 0 || x >= width_px) {
          continue;
        }
        const uint8_t coverage = ink.coverage[static_cast<size_t>(row) * ink.width + col];
        if (coverage == 0) {
          continue;
        }
        // Coverage AND the family's opacity. A partially covered edge pixel of a half-transparent
        // line's label is exactly as translucent as the product says.
        const float a = draw.alpha * static_cast<float>(coverage) * (1.0f / 255.0f);
        const size_t base = (static_cast<size_t>(y) * static_cast<size_t>(width_px) + static_cast<size_t>(x)) * 3u;
        for (int j = 0; j < 3; ++j) {
          // Back to linear, blend, forward again. The main loop left these bytes sRGB-encoded, and
          // compositing in that domain would darken or lighten the mix depending on the
          // background — the same reason every other annotation layer blends before the transfer
          // curve rather than after it.
          const float dst = SrgbToLinear(static_cast<float>(snapshot_image_buffer_[base + j]) * (1.0f / 255.0f));
          const float mixed = std::clamp(dst * (1.0f - a) + draw.rgb[j] * a, 0.0f, 1.0f);
          snapshot_image_buffer_[base + j] = static_cast<uint8_t>(LinearToSrgb(mixed) * 255);
        }
      }
    }
  }
}

Result RenderConsumer::GetResult() const {
  return RenderResult{ config_.id_, config_.resolution_[0], config_.resolution_[1], snapshot_image_buffer_.get() };
}

// See doc/ev-pipeline-architecture.md §2.3
RawXyzResult RenderConsumer::GetRawXyzResult() const {
  int total_pix = config_.resolution_[0] * config_.resolution_[1];
  float per_pixel_intensity = total_pix > 0 ? snapshot_intensity_ / (kNormScale * total_pix) : 0.0f;
  // Note the asymmetry, which the C header documents for callers: the intensity
  // above is pre-divided into a per-pixel figure, while the emitted energy is
  // handed over raw. Raw is what a caller needs to reproduce ExposureScale()
  // itself (scale = intensity_factor · kNormScale · total_pix / emitted_energy);
  // pre-dividing it would only force every caller to multiply the divisor back.
  //
  // The relative branch's own divisor is published the same way and for the same reason: the
  // anchor is a radiance, this buffer is a radiance times a solid angle, and without the solid
  // angle a caller cannot get from one to the other. `anchor_l99_sky_` is filled in by
  // ServerImpl::DoSnapshot (it belongs to the session, not to this renderer); this one is ours.
  RawXyzResult r{ config_.id_,
                  config_.resolution_[0],
                  config_.resolution_[1],
                  snapshot_xyz_.get(),
                  per_pixel_intensity,
                  config_.intensity_factor_,
                  {},
                  {},
                  effective_pix_,
                  snapshot_emitted_energy_ };
  r.axis_solid_angle_ = AxisSolidAngle();
  return r;
}

// See doc/ev-pipeline-architecture.md §3.2
// See doc/accumulator-consumer-architecture.md §3.1 (reset path).
void RenderConsumer::Reset() {
  total_intensity_ = 0;
  snapshot_intensity_ = 0;
  total_emitted_energy_ = 0;
  snapshot_emitted_energy_ = 0;
  effective_pix_ = 0;
  auto buf_size = static_cast<size_t>(config_.resolution_[0]) * config_.resolution_[1] * 3;
  std::memset(internal_xyz_.get(), 0, buf_size * sizeof(float));
  std::memset(comp_xyz_.get(), 0, buf_size * sizeof(float));
  // task-339.3: zero per-class lanes; snapshot_lane_y_ is not zeroed here
  // (PrepareSnapshot will memcpy over it, mirroring snapshot_xyz_).
  for (auto& lane : lane_y_) {
    std::memset(lane.get(), 0, lane_pixel_count_ * sizeof(float));
  }
  // snapshot_xyz_ not zeroed: PrepareSnapshot will memcpy over it.
  // has_ever_consumed_ = false (set in Stop) ensures the frame's xyz results report has_valid_data_=false
  // until new data arrives, preventing stale snapshot reads.
}

// See doc/accumulator-consumer-architecture.md §5.3 (ResetWith path — layout fields guaranteed identical).
void RenderConsumer::ResetWith(const RenderConfig& new_config, const SunParam& new_sun) {
  // NeedsRebuild guarantees layout fields (resolution, lens, view, visible, filter) are identical,
  // so assigning the full config is safe — layout-derived state (rot_, buffers) stays valid.
  config_ = new_config;
  sun_ = new_sun;
  // The annotation inputs this consumer holds that NeedsRebuild does NOT cover (the sun and the
  // three grid-line lists), so this is where a change in any of them has to be noticed. No-ops
  // when nothing moved.
  RebuildAngularDistMasks();
  RebuildGridMasks();
  RebuildHorizonAnnotation();
  // AND the markers, which the zenith/nadir pair this generalizes did NOT need here. Those two are
  // pure geometry and every layout field they depend on is pinned for the consumer's life; four of
  // the six ids are defined relative to sun_, and sun_ is one of the two things this function
  // exists to change. Leaving it out would leave the sun-relative markers pointing at wherever the
  // sun was when the consumer was built.
  RebuildMarkerPoints();
  Reset();
}

// The marker id space is declared twice — as core's annotation::MarkerId and as this config
// layer's MarkerRefId — because config/ cannot include core/annotation_overlay.hpp (that header
// already includes config/render_config.hpp to take a RenderConfig, so the dependency would
// close a cycle). This translation unit is the only place the two meet, so the equality that makes
// the cast below sound is asserted right here rather than described in a comment somewhere:
// reordering either side, or adding an id to one of them alone, becomes a compile error instead of
// a marker that silently resolves to the wrong direction. Same shape as c_api.cpp's assertions for
// the OTHER pairing of this id space, MarkerId against LUMICE_ANNOTATION_MARKER_*.
static_assert(static_cast<int>(MarkerRefId::kZenith) == static_cast<int>(annotation::kMarkerZenith),
              "MarkerRefId and annotation::MarkerId have diverged");
static_assert(static_cast<int>(MarkerRefId::kNadir) == static_cast<int>(annotation::kMarkerNadir),
              "MarkerRefId and annotation::MarkerId have diverged");
static_assert(static_cast<int>(MarkerRefId::kSun) == static_cast<int>(annotation::kMarkerSun),
              "MarkerRefId and annotation::MarkerId have diverged");
static_assert(static_cast<int>(MarkerRefId::kSubsun) == static_cast<int>(annotation::kMarkerSubsun),
              "MarkerRefId and annotation::MarkerId have diverged");
static_assert(static_cast<int>(MarkerRefId::kAnthelion) == static_cast<int>(annotation::kMarkerAnthelion),
              "MarkerRefId and annotation::MarkerId have diverged");
static_assert(static_cast<int>(MarkerRefId::kAntisolar) == static_cast<int>(annotation::kMarkerAntisolar),
              "MarkerRefId and annotation::MarkerId have diverged");

void RenderConsumer::RebuildMarkerPoints() {
  annotation::Request req = MakeMaskRequest(config_);
  // The sun, which four of the six ids are reflections of. ComputeOverlay resolves each id against
  // this vector through core's own direction table (ResolveMarkerDir), so the two poles ignore it
  // and the other four do not — this file does not reproduce any of that arithmetic.
  float sun_dir[3];
  annotation::SunWorldDir(sun_, sun_dir);
  std::copy(std::begin(sun_dir), std::end(sun_dir), std::begin(req.reference_dir));

  // Every id, in canonical order, so overlay.markers[i] IS marker_points_[i] and no index map is
  // needed anywhere downstream.
  req.markers.reserve(annotation::kMarkerCount);
  for (int i = 0; i < annotation::kMarkerCount; ++i) {
    req.markers.push_back(static_cast<annotation::MarkerId>(i));
  }

  const annotation::Overlay overlay = annotation::ComputeOverlay(req);
  // Defensive on the size: a degenerate view returns an empty overlay, and the table must then be
  // all-invalid rather than half-written.
  if (overlay.markers.size() != marker_points_.size()) {
    marker_points_.fill(annotation::CanvasPoint{});
    return;
  }
  std::copy(overlay.markers.begin(), overlay.markers.end(), marker_points_.begin());
}

void RenderConsumer::RebuildAngularDistMasks() {
  const auto& lines = config_.angular_dist_grid_;
  float sun_dir[3];
  annotation::SunWorldDir(sun_, sun_dir);

  std::vector<float> angles;
  angles.reserve(lines.size());
  for (const auto& line : lines) {
    angles.push_back(line.value_);
  }

  // The label switch joins the angle list and the sun in the change detector, because it is an
  // appearance field: it can flip under a reused consumer with the geometry untouched, and a
  // detector that watched only the geometry would keep the previous (empty) anchor list exactly
  // when the user has just asked for the text.
  const bool want_labels = config_.angular_dist_label_;
  if (angular_dist_masks_built_ && angles == angular_dist_mask_angles_ &&
      want_labels == angular_dist_labels_built_for_ &&
      std::equal(std::begin(sun_dir), std::end(sun_dir), std::begin(angular_dist_mask_sun_))) {
    return;
  }
  angular_dist_mask_angles_ = angles;
  std::copy(std::begin(sun_dir), std::end(sun_dir), std::begin(angular_dist_mask_sun_));
  angular_dist_masks_built_ = true;
  angular_dist_labels_built_for_ = want_labels;
  angular_dist_masks_.clear();
  angular_dist_labels_.clear();
  if (angles.empty()) {
    return;
  }

  // ONE ComputeOverlay CALL PER LINE, deliberately, even though the API accepts the whole list at
  // once: a single call returns one union mask for the whole category, which cannot say which line
  // a lit pixel belongs to — and each line carries its own opacity_ and color_. Batching would be
  // correct only for a list whose entries all share an appearance, and adding that special case
  // buys a fast path for a shape nothing produces today. The cost is bounded and paid once per
  // config change, not per frame: at most LUMICE_MAX_CONFIG_GRID_LINES sweeps of W*H.
  annotation::Request req = MakeMaskRequest(config_);
  std::copy(std::begin(sun_dir), std::end(sun_dir), std::begin(req.reference_dir));

  // Anchors ride along on the SAME per-line call the mask already costs; the curve walk is the
  // increment, not a second sweep. Off when this family's switch is off, so a caller that draws no
  // circle text pays nothing for it.
  req.labels = want_labels;

  angular_dist_masks_.reserve(angles.size());
  for (size_t k = 0; k < angles.size(); ++k) {
    req.angular_dist_deg = { angles[k] };
    annotation::Overlay overlay = annotation::ComputeOverlay(req);
    angular_dist_masks_.push_back(std::move(overlay.angular_dist));
    AppendLabels(overlay.labels, static_cast<int>(k), angular_dist_labels_);
  }
}

void RenderConsumer::RebuildGridMasks() {
  RebuildLineFamilyMasks(LineFamily::kElevation);
  RebuildLineFamilyMasks(LineFamily::kLongitude);
}

void RenderConsumer::RebuildLineFamilyMasks(LineFamily family) {
  const bool is_elevation = family == LineFamily::kElevation;
  const auto& lines = is_elevation ? config_.elevation_grid_ : config_.longitude_grid_;
  auto& masks = is_elevation ? elevation_masks_ : longitude_masks_;
  auto& built_from = is_elevation ? elevation_mask_angles_ : longitude_mask_angles_;
  bool& built = is_elevation ? elevation_masks_built_ : longitude_masks_built_;
  auto& labels = is_elevation ? elevation_labels_ : longitude_labels_;
  bool& labels_built_for = is_elevation ? elevation_labels_built_for_ : longitude_labels_built_for_;
  // ONE switch for both families: the GUI has a single grid label control, and core's schema
  // follows it rather than inventing a distinction no consumer draws.
  const bool want_labels = config_.grid_label_;

  std::vector<float> angles;
  angles.reserve(lines.size());
  for (const auto& line : lines) {
    angles.push_back(line.value_);
  }

  // No sun term in this comparison, unlike RebuildAngularDistMasks: a parallel sits at a fixed
  // altitude and a meridian at a fixed azimuth, so neither curve moves when the sun does.
  // Same reason the angular-distance detector carries its switch: *_label_ is an appearance field
  // and can change with the angle list untouched.
  if (built && angles == built_from && want_labels == labels_built_for) {
    return;
  }
  built_from = angles;
  built = true;
  labels_built_for = want_labels;
  masks.clear();
  labels.clear();
  if (angles.empty()) {
    return;
  }

  // One ComputeOverlay call per line, for the reason spelled out in RebuildAngularDistMasks:
  // each entry carries its own opacity_ / color_, and a category-wide union mask cannot say which
  // line a lit pixel belongs to.
  annotation::Request req = MakeMaskRequest(config_);
  req.labels = want_labels;
  masks.reserve(angles.size());
  for (size_t k = 0; k < angles.size(); ++k) {
    if (is_elevation) {
      req.elevation_deg = { angles[k] };
    } else {
      req.longitude_deg = { angles[k] };
    }
    annotation::Overlay overlay = annotation::ComputeOverlay(req);
    masks.push_back(std::move(is_elevation ? overlay.elevation : overlay.longitude));
    AppendLabels(overlay.labels, static_cast<int>(k), labels);
  }
}

// The horizon's line AND its label anchors, from ONE annotation::ComputeOverlay call — the same
// shape the three families above have, and the point of this task: the mask used to come from a
// second, independent path (BuildHorizonMask), so one curve was computed twice and the two
// computations were free to drift. Nothing else in the renderer decides where the line goes.
//
// `req.horizon` is unconditional, NOT gated on config_.horizon_: the mask's member declaration
// argues why (horizon_ is an appearance field that ResetWith can flip with no rebuild, so a mask
// built only for its construction-time value would be empty exactly when the user has just asked
// for the line). The LINE's own gate stays on config_.horizon_ alone, at the point of use in
// PostSnapshot; the TEXT's on config_.horizon_label_, here.
void RenderConsumer::RebuildHorizonAnnotation() {
  if (horizon_mask_built_ && config_.horizon_label_ == horizon_labels_built_for_) {
    return;  // nothing this depends on has moved (the view is fixed for a consumer's whole life)
  }
  horizon_mask_built_ = true;
  horizon_labels_built_for_ = config_.horizon_label_;
  horizon_labels_.clear();
  annotation::Request req = MakeMaskRequest(config_);
  req.horizon = true;
  // The curve walk is the only part the text switch can save, so it is the only part it gates.
  req.labels = config_.horizon_label_;
  annotation::Overlay overlay = annotation::ComputeOverlay(req);
  horizon_mask_ = std::move(overlay.horizon);
  if (!config_.horizon_label_) {
    return;
  }
  // -1, not an index: the horizon has no line list to index into, which is also how PaintLabels
  // tells it apart from the three families that do (it is collected separately there).
  AppendLabels(overlay.labels, -1, horizon_labels_);
}

}  // namespace lumice
