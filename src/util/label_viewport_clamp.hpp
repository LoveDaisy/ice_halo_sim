#pragma once

#include <algorithm>

namespace lumice {

// =================================================================================================
// The ONE viewport clamp for overlay text labels, shared by both renderers.
//
// Two renderers draw the same overlay labels onto the same anchors: the GUI's draw-list pass
// (src/gui/overlay_labels.cpp, AppendOverlayToDrawList) and the CLI/server compositor
// (src/server/render.cpp, RenderConsumer::PaintLabels). An anchor is placed where the curve ENTERS
// the visible region (core's annotation_overlay.cpp), which on a narrow field of view is the frame
// edge itself — so a label centred on its anchor straddles that edge routinely, not rarely.
//
// This file is that rule's SINGLE OWNER. It lives in src/util/ rather than in either renderer for
// the one reason that matters: src/gui/ may not include core/ or config/ (the C API boundary,
// enforced by scripts/check_policies.py), and core may not depend on the GUI at all, so a rule
// written on either side could only be shared by copying it. util/ is includable from both — the
// GUI already includes several util/ headers directly, and so does src/server/render.cpp — and a
// pure, stateless geometry function is exactly what may travel that way without weakening the
// boundary the gate draws around core/ and config/.
//
// Both call sites are thin: the GUI's detail::ClampLabelPosToViewport unpacks ImVec2 (core cannot
// see ImGui types), the server's PaintLabels unpacks ints. Neither repeats any of the decisions
// below. Change the inset or the degenerate-viewport rule here and both renderers move together.
// =================================================================================================

// A small visual padding, in the pixels of whichever canvas is being drawn to, keeping the text
// glyph rect away from the absolute viewport edge — it guards against the half-pixel anti-aliasing
// fringe and, in the GUI, the panel border's own 1 px line. 2 px is a visual-padding heuristic, not
// a precise geometric boundary; revisit if a future HiDPI-aware UI pass requires resolution-scaled
// padding. Deliberately NOT scaled by resolution today: both renderers have always treated it as an
// absolute pixel count, and giving one side a scaled inset would put a GUI/CLI divergence back.
inline constexpr float kLabelViewportInsetPx = 2.0f;

// The three geometric quantities, each in its own type. They are all "two or four floats", and a
// flat parameter list of them invites exactly the transposition bug this clamp's two call sites are
// most exposed to (pos vs size, width vs height), so the types carry the meaning instead.
struct LabelPos {
  float x = 0.0f;
  float y = 0.0f;
};

struct LabelSize {
  float w = 0.0f;
  float h = 0.0f;
};

struct LabelViewport {
  float x = 0.0f;
  float y = 0.0f;
  float w = 0.0f;
  float h = 0.0f;
};

// Push the text glyph rect `[pos, pos + size]` inside `vp`, leaving `kLabelViewportInsetPx` of
// margin against each edge, and return its new top-left. `pos` is the rect's top-left, i.e. the
// caller has already applied whatever centring turns an anchor into a corner.
//
// The axes are independent: a label past the left edge and comfortably inside vertically moves in x
// only. When the viewport cannot hold the text plus two insets on an axis, that axis is left
// UNTOUCHED rather than clamped to a nonsense position — clamping there would move the label to a
// corner for no benefit, whereas leaving it alone keeps the legacy "centred on its anchor, possibly
// cropped" look for degenerate targets (a collapsed panel, a tiny export).
inline LabelPos ClampLabelPosToViewport(LabelPos pos, LabelSize size, LabelViewport vp) {
  if (vp.w > size.w + 2.0f * kLabelViewportInsetPx) {
    pos.x = std::max(pos.x, vp.x + kLabelViewportInsetPx);
    pos.x = std::min(pos.x, vp.x + vp.w - size.w - kLabelViewportInsetPx);
  }
  if (vp.h > size.h + 2.0f * kLabelViewportInsetPx) {
    pos.y = std::max(pos.y, vp.y + kLabelViewportInsetPx);
    pos.y = std::min(pos.y, vp.y + vp.h - size.h - kLabelViewportInsetPx);
  }
  return pos;
}

}  // namespace lumice
