[中文版](adaptive-brightness.zh.md)

# Adaptive Brightness

This document describes Lumice's Adaptive Brightness feature: its algorithm, the two modes
(Off / On), the physical additivity invariant it preserves in Off mode, and the rationale
behind the tooltip wording.

**Target audience**: users who want to understand why enabling or disabling Adaptive
Brightness changes how filter outputs compare to each other, and contributors who need to
reason about the EV-normalization pipeline.

---

## 1. Overview

Rendering ice-halo simulations produces raw XYZ irradiance values whose absolute scale
varies enormously across scene configurations. A scene with millions of 22° halo rays
accumulates a much brighter buffer than a scene with a rare, faint arc. Without
normalization, comparing two configurations requires manual EV slider tuning.

**Adaptive Brightness** automates this EV adjustment. The GUI computes an EV offset
(displayed as `+N.NN EV` next to the checkbox) and applies it to the post-processing
pipeline so that a perceptually significant fraction of the bright pixels lands at a
user-configurable target brightness on screen.

The checkbox is found in the **right panel → Display** section.

---

## 2. Algorithm

### 2.1 P99-Anchored Normalization

The core algorithm (`ComputeP99Y` / `ComputeEvAuto` in `src/gui/gui_ev_auto.hpp`) is:

1. **Extract** all positive Y-channel values from the accumulated XYZ buffer.
2. **Compute the P99 value** (`y_p99`): the 99th percentile of those Y values.
3. **Normalize** relative to `snapshot_intensity` (total accumulated intensity):
   ```
   p99_norm = y_p99 / snapshot_intensity
   ```
4. **Map** `p99_norm` to `target_white` on the sRGB [0, 255] scale.  
   `target_white` is fixed at 200 (the GUI no longer exposes a slider; users tune the
   manual EV offset for further adjustment).  
   The mapping applies the sRGB transfer function inverse to obtain a linear target:
   ```
   t = target_white / 255
   target_linear = (t ≤ 0.04045) ? t / 12.92 : ((t + 0.055) / 1.055)^2.4
   ```
5. **Compute the EV offset** in stops, clamped to [−6, +6]:
   ```
   ev = log2(target_linear / p99_norm)
   ```

The resulting `ev` is added to the manual EV slider value before the post-processing pass.
When `p99_norm` is unavailable (no data yet), the EV contribution is 0 and the GUI shows
`(no data)`.

### 2.2 Data Source Selection

The P99 is computed from different XYZ buffers depending on the mode:

| Mode | XYZ buffer used for P99 |
|------|-------------------------|
| **Off** | `unfiltered_xyz_buffer` — all rays that exit the crystal, before any ray-path filter is applied |
| **On**  | `filtered_xyz_buffer` — only rays that survive the currently active ray-path filter |

---

## 3. Modes

### Off — Physically Comparable Outputs

In **Off** mode the P99 anchor is derived from the **unfiltered** buffer: every outgoing ray
contributes to the anchor regardless of whether it passes the active filter.

This means:

- All filter configurations share the same EV scale for a given simulation run.
- Turning a filter on or off, or switching between two complementary filters, does not shift
  the overall brightness level — only the subset of rays shown changes.
- **Physical additivity holds** (see §4).

Recommended when you want to:

- Visually compare multiple filters side-by-side.
- Verify that a set of complementary filters (a partition) sums to the unfiltered result.
- Understand the absolute contribution of each ray-path class.

### On — Adaptive Visibility

In **On** mode the P99 anchor is derived from the **filtered** buffer: only rays that
survive the active filter contribute to the anchor.

This means:

- A filter that captures a faint, isolated arc (few surviving rays, low absolute
  brightness) gets its own local EV boost — the arc becomes visible even when it would
  otherwise be lost in the noise floor under a global scale.
- Switching between two different filters changes the EV offset independently for each.
- **Additivity is not guaranteed** (see §4).

Recommended when you want to:

- Examine the visual quality of a single filter in isolation.
- Make a faint or rare arc visible without manually tweaking the EV slider.

---

## 4. Additivity

### 4.1 Linear XYZ Space — Off Mode

Let F₁, F₂, …, Fₙ be a set of **complementary ray-path filters** — filters that partition
the set of all outgoing rays (every ray matches exactly one Fᵢ, and no ray is counted
twice). Define `buf(Fᵢ)` as the accumulated XYZ buffer when filter Fᵢ is active.

In **Off** mode, all buffers are normalized by the same unfiltered anchor, so:

```
buf(F₁) + buf(F₂) + … + buf(Fₙ) = buf(unfiltered)
```

This equality holds **exactly** in the linear XYZ color space. It is a direct consequence
of the linearity of the accumulation pass: each ray's XYZ contribution is counted in
exactly one `buf(Fᵢ)`.

### 4.2 sRGB Pixel Level — Not Additive

The sRGB color space applies a nonlinear gamma transfer function (`x^(1/2.4)` in the
bright region). Because gamma is nonlinear:

```
sRGB(buf(F₁)) + sRGB(buf(F₂)) ≠ sRGB(buf(F₁) + buf(F₂))
```

In practice, the visual discrepancy is small for typical halo scenes (the gamma curve is
nearly linear for dim pixels and the brightening effect is modest for bright ones), but the
equality does not hold in general.

**Summary**:

| | XYZ linear space | sRGB pixel space |
|---|---|---|
| Off mode, complementary filters | ✅ Exactly additive | ❌ Not additive (gamma nonlinearity) |
| On mode | ❌ No additivity guarantee | ❌ No additivity guarantee |

### 4.3 On Mode

In **On** mode each filter uses its own private anchor, so the normalization factors differ
between filters. Adding two normalized pixel buffers has no physical interpretation. Off
mode should be used when comparing or combining filter outputs.

---

## 5. Tooltip Wording Rationale

The GUI tooltip reads:

> Off: filter outputs are physically comparable.  
> On: each filter adapts to its visible rays.

The wording intentionally avoids technical terms such as "XYZ linear space", "P99
percentile", or "sRGB gamma". The tooltip targets general users who do not have a color
science background. The key distinction is expressed in user-outcome terms:

- **"physically comparable"** conveys that filter outputs can be placed side-by-side and
  the relative brightness carries meaning — the full physical relationship is preserved.
- **"adapts to its visible rays"** conveys that each filter gets its own brightness scale —
  useful for inspecting a dim arc that would otherwise be invisible.

Contributors who need the technical specification should consult this document and
`src/gui/gui_ev_auto.hpp`.

---

## 5b. Behavior Change Notice (task-query-filter-uplift-v2)

Prior to this change, the simulator marked any ray that failed a configured
`scattering.entries[].filter` as stopped (the pre-T2 `kStopped` state, since
folded into the `IsTir()` predicate), so it never reached the consumer's
"unfiltered" accumulator. As a result, `unfiltered_xyz_buffer` was actually
*post-filter*, and Off-mode EV was indirectly sensitive to the filter — violating
the additivity invariant described in §4.

The fix demotes the simulator-side filter to a pure branch gate (controlling
the multi-scatter `IsContinue()` bit only), so filter-fail rays are emitted as
`IsOutgoing()` and the consumer's Path B accumulates the true unfiltered set.

**User-visible consequence**:

- Scenes that previously configured a low-pass-rate filter (e.g. a raypath
  filter where only a small percentage of rays survive) will see Off-mode EV
  shift toward "darker" because the anchor is now the full ray set rather than
  the post-filter subset. A ~0.1%-pass-rate filter can lower the Off-mode anchor
  by roughly 10 stops.
- On-mode behaviour is unchanged.
- The filtered ("On-mode") XYZ buffer and final rendered image at a given EV
  are unchanged.

If you relied on the previous Off-mode behaviour, switch to On mode or adjust
the manual EV offset.

---

## 5c. Off Mode Status After task-revert (219.4)

> **Note**: this section describes the post-task-revert (219.4) state, which is pending
> implementation.  Until 219.4 lands, the current code still follows the task-200 routing
> described in §5b.

The task-200 routing decision described in §5b is being reverted in task-revert
(219.4, scrum-prob-zero-leak / #219).  Filter evaluation is moving back to the
simulator side (Design A): filter-fail rays will be dropped inside the simulator and never
reach the consumer.

After the revert:

- `unfiltered_xyz_buffer` **will equal** `xyz_buffer` — both will reflect the
  same simulator-filtered ray set.  The distinction between "unfiltered" and "filtered"
  will no longer exist at the consumer level.
- **Off mode will be temporarily hard-disabled** in the GUI (toggle grayed out, tooltip
  explains the situation).  Since the unfiltered anchor equals the filtered anchor,
  Off mode and On mode would be identical — and the ~10-stop brightness jump that Off
  mode was designed to prevent can no longer be triggered.
- `unfiltered_xyz_buffer` and `unfiltered_snapshot_intensity` in `LUMICE_RawXyzResult`
  are **DEPRECATED**; after task-revert (219.4), use `xyz_buffer` and `snapshot_intensity`
  instead.

The redesign of Off mode on the Design A baseline (restoring the additivity invariant
through a different mechanism) is tracked as a future backlog item.
See `doc/filter-architecture.md §7` for the full specification.

---

## 6. References

### Code Paths

| Component | File | Purpose |
|-----------|------|---------|
| Algorithm | `src/gui/gui_ev_auto.hpp` | `ComputeP99Y`, `ComputeEvAuto` |
| Data-source switch | `src/gui/app.cpp` — `SyncFromPoller()` | Selects `unfiltered_xyz_buffer` vs `filtered_xyz_buffer` based on `auto_ev_enabled` |
| GUI control | `src/gui/app_panels.cpp` line 487 | Checkbox + tooltip + EV display (`target_white` is fixed at 200, no slider) |
| Unfiltered buffer | `src/server/render.cpp` — `Consume()` | Accumulates the unfiltered XYZ pass before `FilterRay` |
| C API field | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `unfiltered_xyz_buffer` and `unfiltered_snapshot_intensity` fields |

### Related Documentation

- `doc/configuration.md` — full JSON configuration reference
- `doc/gui-guide.md` — GUI layout and panel overview
- `doc/raypath-symmetry.md` — ray-path symmetry and filter semantics (P, B, D toggles)
