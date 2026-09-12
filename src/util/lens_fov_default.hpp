#pragma once

namespace lumice {

// =================================================================================================
// The ONE default field of view for a `lens` that states neither `fov` nor `f`, shared by both
// readers of the config document.
//
// Two readers decode that object: core's `LensParam::from_json` (src/config/render_config.cpp,
// behind the CLI and the C API) and the GUI's import path (src/gui/file_io.cpp,
// `DeserializeFromJson`). Both must load the same document at the same angle, or "what the GUI
// shows" and "what the CLI renders" diverge on a key the author never wrote — a divergence no
// round-trip test on either side can see. So the number lives here, once, and both sides call
// this function rather than each carrying its own copy of it.
//
// The numbers are the ones doc/configuration.md's lens "Defaults" section has published all along:
// 90 degrees for every lens, 30 for `globe` — the outside-in perspective view of the celestial
// sphere, whose fov is capped at 90 (MaxFov) and whose sphere fills ~96% of the short edge at 30.
//
// The parameter is a bool rather than either side's lens-type enum on purpose: core's
// `LensParam::LensType` and the GUI's `LensType` are two different types, and a shared function
// bound to one of them could only be reached from the other side by copying the enum — the exact
// duplication this header exists to remove. A bool with no simulation or configuration semantics
// is what may travel through src/util/ (see label_viewport_clamp.hpp for the boundary rule).
// =================================================================================================
constexpr float LensDefaultFovDegrees(bool is_globe) {
  return is_globe ? 30.0f : 90.0f;
}

}  // namespace lumice
