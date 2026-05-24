[中文版](adaptive-brightness.zh.md)

# Adaptive Brightness

This document describes Lumice's Adaptive Brightness feature: its algorithm, the F1
anchor-lane mechanism that backs it, and the physical additivity invariant the design
preserves.

**Target audience**: users who want to understand how Lumice keeps the EV scale stable
when toggling filters, and contributors who need to reason about the EV-normalization
pipeline.

---

## 1. Overview

Rendering ice-halo simulations produces raw XYZ irradiance values whose absolute scale
varies enormously across scene configurations. A scene with millions of 22° halo rays
accumulates a much brighter buffer than a scene with a rare, faint arc. Without
normalization, comparing two configurations requires manual EV slider tuning.

**Adaptive Brightness** automates this EV adjustment. The GUI computes an EV offset
(displayed as `+N.NN EV auto` next to the manual EV slider in the **right panel →
Display** section) and applies it to the post-processing pipeline so that a perceptually
significant fraction of the bright pixels lands at a user-configurable target brightness
on screen.

The feature is always on — there is no GUI toggle. Filter switches do not jump the
EV because the anchor is derived from the filter-independent F1 anchor lane.

---

## 2. Algorithm

### 2.1 P99.5-Anchored Normalization

The core algorithm (`ComputeP995Y` / `ComputeEvAuto` in `src/gui/gui_ev_auto.hpp`) is:

1. **Extract** all positive Y-channel values from the anchor XYZ buffer.
2. **Compute the P99.5 value** (`y_p995`): the 99.5th percentile of those Y values.
3. **Normalize** relative to the anchor intensity:
   ```
   p99_norm = y_p995 / anchor_snapshot_intensity
   ```
4. **Map** `p99_norm` to `target_white` on the sRGB [0, 255] scale.
   `target_white` is fixed at 135 (the GUI no longer exposes a slider; users tune the
   manual EV offset for further adjustment).
   The mapping applies the sRGB transfer function inverse to obtain a linear target:
   ```
   t = target_white / 255
   target_linear = (t ≤ 0.04045) ? t / 12.92 : ((t + 0.055) / 1.055)^2.4
   ```
5. **Compute the EV offset** in stops, clamped to [−6, +6]:
   ```
   ev_auto = log2(target_linear / p99_norm)
   ```

`ev_auto` is added to the manual EV slider value before the post-processing pass. When
no data is available yet, the EV contribution is 0 and the GUI shows `(auto: no data)`.

### 2.2 Data Source — F1 Anchor Lane

`y_p995` and the normalization intensity come from the server-side F1 anchor lane:

| Field | Meaning |
|-------|---------|
| `anchor_p995_y` | P99.5 of Y over filter-pass + filter-fail emission combined. |
| `anchor_snapshot_intensity` | Per-pixel landed intensity for the same combined set. |

Both are exposed by `LUMICE_RawXyzResult` (see `src/include/lumice.h`).

**Degenerate path (no filter)**: when no filter is configured, the simulator skips the
anchor lane entirely (no `IsFilterDropped` writes), so the server returns
`anchor_p995_y = 0` and `anchor_snapshot_intensity = 0`. `SyncFromPoller()` falls back
to the filtered snapshot:

```cpp
if (data.anchor_p995_y > 0.0f && data.anchor_snapshot_intensity > 0.0f) {
    g_state.p995_raw_y = data.anchor_p995_y;
    g_state.ev_auto = ComputeEvAuto(g_state.p995_raw_y, g_state.anchor_snapshot_intensity, target_white);
} else {
    g_state.p995_raw_y = ComputeP995Y(data.xyz_data);
    g_state.ev_auto = ComputeEvAuto(g_state.p995_raw_y, g_state.snapshot_intensity, target_white);
}
```

With no filter the filtered snapshot equals the total emission, so the two branches are
numerically equivalent — the fallback is purely a missing-data guard, not a semantic
mode switch.

---

## 3. Behavior

All filter configurations within a single simulation share the same EV scale:

- Switching the filter on or off, or swapping between two complementary filters, does
  not shift the overall brightness level — only the subset of rays shown changes.
- **Physical additivity holds** for complementary ray-path filters (see §4).

Recommended workflows:

- Visually compare multiple filters side-by-side without re-tuning EV.
- Verify that a set of complementary filters (a partition) sums to the unfiltered
  result.
- Understand the absolute contribution of each ray-path class.

**Performance note**: when a filter spec is present, the simulator runs the anchor-lane
collection pass alongside the normal simulation. Overhead scales with the filter-fail
ray volume:

- `ms_prob ≈ 0` (single-scatter or near-zero multi-scatter): overhead is negligible (~0%).
- `ms_prob ≈ 0.5`: approximately +97% overhead (~2× simulation time).

When no filter is configured the anchor lane stays empty (the simulator skips the
`IsFilterDropped` writes) and the cost is zero.

---

## 4. Additivity

### 4.1 Linear XYZ Space

Let F₁, F₂, …, Fₙ be a set of **complementary ray-path filters** — filters that partition
the set of all outgoing rays (every ray matches exactly one Fᵢ, and no ray is counted
twice). Define `buf(Fᵢ)` as the accumulated XYZ buffer when filter Fᵢ is active.

Because every filter run normalizes by the same anchor intensity:

```
buf(F₁) + buf(F₂) + … + buf(Fₙ) = buf(unfiltered)
```

This equality holds **exactly** in the linear XYZ color space. It is a direct
consequence of the linearity of the accumulation pass: each ray's XYZ contribution
lands in exactly one `buf(Fᵢ)`.

### 4.2 sRGB Pixel Level — Not Additive

The sRGB color space applies a nonlinear gamma transfer function (`x^(1/2.4)` in the
bright region). Because gamma is nonlinear:

```
sRGB(buf(F₁)) + sRGB(buf(F₂)) ≠ sRGB(buf(F₁) + buf(F₂))
```

In practice, the visual discrepancy is small for typical halo scenes (the gamma curve is
nearly linear for dim pixels and the brightening effect is modest for bright ones), but
the equality does not hold in general.

**Summary**:

| Composition | XYZ linear space | sRGB pixel space |
|---|---|---|
| Complementary filters with shared anchor | ✅ Exactly additive | ❌ Not additive (gamma nonlinearity) |

---

## 5. F1 Anchor Lane Mechanism

When a filter spec is active, the simulator runs a parallel anchor-lane pass:

```
crystal exit (outgoing candidate)
    │
    ├─ filter passes  →  xyz_buffer  +  anchor lane
    │
    └─ filter fails   →  anchor lane only
                         (not rendered; contributes to EV anchor only)
```

The anchor lane accumulates filter-pass + filter-fail emission. The C API exposes the
P99.5 and intensity via `LUMICE_RawXyzResult.anchor_p995_y` and `anchor_snapshot_intensity`.
The GUI (`SyncFromPoller()` in `src/gui/app.cpp`) uses these to compute the EV offset.

The branch table inside `CollectData` (see `src/core/simulator.cpp`) is:

```
filter-pass + prob-pass → IsContinue()      (next MS scatter)
filter-pass + prob-fail → IsOutgoing()      (emit, contributes to xyz_buffer + anchor)
filter-fail + prob-pass → IsContinue()      (anchor-lane bypass — propagates)
filter-fail + prob-fail → IsFilterDropped() (anchor lane only)
```

`IsContinue()` and `IsFilterDropped()` are mutually exclusive. See
`doc/filter-architecture.md §7` for the full design rationale.

---

## 6. Historical Notes

### 6.1 Pre-task-query-filter-uplift-v2 — Unfiltered Buffer Behavior

Prior to task-query-filter-uplift-v2 the simulator marked any ray that failed a
configured `scattering.entries[].filter` as stopped (the pre-T2 `kStopped` state, since
folded into the `IsTir()` predicate), so it never reached the consumer's "unfiltered"
accumulator. As a result, `unfiltered_xyz_buffer` was actually *post-filter*, and the
EV anchor was indirectly sensitive to the filter.

The fix demoted the simulator-side filter to a pure branch gate (controlling the
multi-scatter `IsContinue()` bit only) so filter-fail rays were emitted as
`IsOutgoing()` and the consumer's Path B accumulated the true unfiltered set.

### 6.2 scrum-221 — Mode-Gated F1

scrum-221 introduced a GUI Adaptive Brightness ON/OFF toggle that selected between
per-frame self-anchor (ON, Design A) and the F1 anchor lane (OFF). The toggle and
Design A path were removed in `task-remove-adaptive-brightness-on-mode` (this revision)
after internal testing confirmed the F1 anchor produces strictly better UX with
acceptable performance cost. The anchor-lane implementation is unchanged; only the
mode dispatch is gone.

---

## 7. References

### Code Paths

| Component | File | Purpose |
|-----------|------|---------|
| Algorithm | `src/gui/gui_ev_auto.hpp` | `ComputeP995Y`, `ComputeEvAuto` |
| EV source | `src/gui/app.cpp` — `SyncFromPoller()` | Reads `anchor_p995_y` / `anchor_snapshot_intensity` (degenerate fallback to filtered snapshot when both are 0) |
| GUI display | `src/gui/app_panels.cpp` | `(+N.NN EV auto)` text and `ev_total = exposure_offset + ev_auto` |
| Anchor lane | `src/server/render.cpp` — `Consume()` | Accumulates filter-pass + filter-fail emission into the anchor buffer when filter is present |
| Simulator | `src/core/simulator.cpp` — `CollectData()` | F1 branch table; routes filter-fail outgoing to anchor lane |
| C API fields | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `anchor_p995_y`, `anchor_snapshot_intensity`, `xyz_buffer`, `snapshot_intensity` |

### Related Documentation

- `doc/filter-architecture.md` §7 — F1 anchor lane design + branch semantics
- `doc/configuration.md` — full JSON configuration reference
- `doc/gui-guide.md` — GUI layout and panel overview
- `doc/raypath-symmetry.md` — ray-path symmetry and filter semantics (P, B, D toggles)
