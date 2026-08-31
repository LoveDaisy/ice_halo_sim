#ifndef LUMICE_GUI_MONO_EXPOSURE_SCALE_HPP
#define LUMICE_GUI_MONO_EXPOSURE_SCALE_HPP

// The mono preview's exposure arithmetic, in one place.
//
// The GUI does NOT read the server's exposure scale for the mono path. It pulls raw XYZ out of a
// result frame and applies its own factor, so that dragging the EV slider (or, now, flipping the
// exposure mode) re-lights the picture on the very next frame instead of waiting for a commit and
// a poll. Three call sites need that number — the live shader uniform, the CPU texture the .lmc
// save embeds, and the PNG export — and they used to each carry their own copy of the formula.
// One copy, here, is the point of this header: three hand-written mirrors of an exposure formula
// is exactly the shape of drift this scrum exists to remove.
//
// Deliberately free of ImGui / GL / server dependencies so it can be unit-tested without a window
// (same shape as composite_exposure_push.hpp beside it).

#include <cmath>
#include <cstdio>
#include <string>

#include "gui/gui_constants.hpp"

namespace lumice::gui {

// Which anchor the displayed brightness is measured against. Mirrors core
// RenderConfig::EvMode and RenderConfig::ev_mode's integer encoding (gui_state.hpp) —
// 0 = relative, 1 = absolute — so an int straight out of GuiState can be cast in.
enum class MonoEvMode : int {
  kRelative = 0,
  kAbsolute = 1,
};

struct MonoExposure {
  // 2^EV. What the user's slider asks for, before any anchoring.
  float intensity_factor = 0.0f;
  // The multiplier applied to a raw XYZ sample to get display-linear brightness. Zero means
  // "nothing to show" — the anchor is not measurable yet (no landed energy / no emitted energy).
  float intensity_scale = 0.0f;
};

// Inputs to the mono exposure computation, gathered rather than passed as a positional list: the
// two modes read overlapping-but-different subsets of them, and a caller that mixes up two
// adjacent floats in a six-argument call gets a picture that is merely wrong, not a compile error.
struct MonoExposureInput {
  // The EV slider, in stops. Under kAbsolute this is the whole exposure statement.
  float exposure_offset = 0.0f;
  // The P99-derived auto anchor, in stops. Consumed by kRelative ONLY — see ComputeMonoExposure.
  float ev_auto = 0.0f;
  // Per-pixel LANDED intensity from LUMICE_RawXyzResult::snapshot_intensity. The kRelative
  // denominator.
  float snapshot_intensity = 0.0f;
  // Total energy the source EMITTED, from LUMICE_RawXyzResult::emitted_energy. The kAbsolute
  // denominator, and the reason absolute mode is comparable across documents: it counts what went
  // in, not what survived to a pixel.
  float snapshot_emitted_energy = 0.0f;
  // Pixel count of the frame being displayed (img_width * img_height). kAbsolute only.
  int total_pixels = 0;
};

// The mono path's scale, per mode.
//
// kRelative — unchanged from what the GUI has always done, and it must stay that way: this is the
//   default, so every existing document and every committed reference image depends on it
//   bit-for-bit. The auto anchor is folded into the exposure, and the result is divided by the
//   frame's own landed intensity, which makes the number a statement about THIS frame only. Two
//   documents at the same EV say nothing about each other.
//
// kAbsolute — mirrors RenderConsumer::ExposureScale's absolute branch (server/render.cpp), which
//   is what makes the GUI preview and the CLI agree on brightness. Two differences from relative
//   are both load-bearing:
//     * `ev_auto` is NOT added. The whole promise of absolute mode is that EV reads as "stops
//       above or below physical"; silently adding a per-frame auto anchor to it would put the
//       reading back on a moving baseline and the number would stop meaning anything across
//       documents. The auto value is still computed and still shown in the UI, labelled as not
//       applied.
//     * the denominator is EMITTED energy, not landed. Emitted is fixed by the light source and
//       the ray budget, so it does not drift as filters remove rays or as the accumulation runs —
//       which is precisely what makes two differently-configured scenes comparable at one EV.
//
// Both branches keep the guard the inline call sites had: a non-positive denominator yields a
// scale of 0 rather than an infinity that would paint the frame white.
inline MonoExposure ComputeMonoExposure(MonoEvMode mode, const MonoExposureInput& in) {
  MonoExposure out;
  if (mode == MonoEvMode::kAbsolute) {
    out.intensity_factor = std::pow(2.0f, in.exposure_offset);
    if (in.snapshot_emitted_energy > 0.0f && in.total_pixels > 0) {
      out.intensity_scale =
          out.intensity_factor * kNormScale * static_cast<float>(in.total_pixels) / in.snapshot_emitted_energy;
    }
    return out;
  }

  out.intensity_factor = std::pow(2.0f, in.exposure_offset + in.ev_auto);
  if (in.snapshot_intensity > 0.0f) {
    out.intensity_scale = out.intensity_factor / in.snapshot_intensity;
  }
  return out;
}

// Convenience overload for call sites holding the raw int out of RenderConfig::ev_mode. Anything
// outside {0, 1} reads as relative, matching core's NLOHMANN_JSON_SERIALIZE_ENUM fallback
// (render_config.hpp lists kRelative first for the same reason).
inline MonoExposure ComputeMonoExposure(int ev_mode, const MonoExposureInput& in) {
  return ComputeMonoExposure(ev_mode == 1 ? MonoEvMode::kAbsolute : MonoEvMode::kRelative, in);
}

// The one-line readout the Display group keeps on screen under the EV slider.
//
// It exists because exposure_offset is saved PER DOCUMENT (file_io.cpp) while absolute mode makes
// EV a cross-document quantity. Open two documents in absolute mode and each restores its own EV,
// so the two can sit at different heights on a shared scale with nothing on screen saying so. The
// GUI is single-document and cannot see the other file's value, so it cannot warn about a specific
// mismatch; what it can do is never leave the current height implicit. Hence a permanent readout
// rather than a tooltip.
//
// Under kAbsolute the reading is the manual offset alone, and it is labelled as measured against
// physical. Under kRelative the manual offset alone would be misleading — the auto anchor moves
// underneath it, by many stops in a sparse scene — so all three numbers are shown: what the user
// set, what the anchor contributed, and what the picture is actually exposed at.
inline std::string FormatMonoEvReadout(MonoEvMode mode, float exposure_offset, float ev_auto) {
  char buf[128];
  if (mode == MonoEvMode::kAbsolute) {
    std::snprintf(buf, sizeof(buf), "Absolute | EV %+.2f vs physical", exposure_offset);
  } else {
    std::snprintf(buf, sizeof(buf), "Relative | EV %+.2f manual %+.2f auto = %+.2f effective", exposure_offset, ev_auto,
                  exposure_offset + ev_auto);
  }
  return buf;
}

inline std::string FormatMonoEvReadout(int ev_mode, float exposure_offset, float ev_auto) {
  return FormatMonoEvReadout(ev_mode == 1 ? MonoEvMode::kAbsolute : MonoEvMode::kRelative, exposure_offset, ev_auto);
}

}  // namespace lumice::gui

#endif  // LUMICE_GUI_MONO_EXPOSURE_SCALE_HPP
