[中文版](adaptive-brightness.zh.md)

# Adaptive Brightness

This document describes Lumice's Adaptive Brightness feature: the algorithm
that auto-computes an EV offset from each frame's visible framebuffer.

**Target audience**: users who want to understand how Lumice picks a
reasonable starting EV, and contributors who need to reason about the
EV-normalization pipeline.

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

The feature is always on — there is no GUI toggle. The anchor is derived from the
current frame's visible framebuffer (self-P99).

---

## 2. Algorithm

### 2.1 P99-Anchored Normalization

The core algorithm (`ComputeP99Y` / `ComputeEvAuto` in `src/gui/gui_ev_auto.hpp`) is:

1. **Extract** all positive Y-channel values from the visible XYZ buffer
   (`data.xyz_data`) shipped by the server poller.
2. **Compute the P99 value** (`y_p99`): the 99th percentile of those Y values.
3. **Normalize** relative to the per-pixel landed intensity:
   ```
   p99_norm = y_p99 / snapshot_intensity
   ```
4. **Map** `p99_norm` to `target_white` on the sRGB [0, 255] scale.
   `target_white` is fixed at 135.
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

### 2.2 Data Source

The P99 is computed in the poller thread from the visible XYZ buffer
(`PollerData::p99_y`); `snapshot_intensity` is the per-pixel landed intensity
returned by the server. Both fields are populated unconditionally — there is no
filter-dependent branching.

```cpp
g_state.p99_raw_y = data.p99_y;
g_state.ev_auto = ComputeEvAuto(g_state.p99_raw_y, g_state.snapshot_intensity, target_white);
```

### 2.3 Filter Interaction

When a ray-path filter is active, only filter-pass rays accumulate into the visible
framebuffer (Design A: filter-fail rays terminate immediately in `CollectData`).
The P99 / `snapshot_intensity` pair therefore tracks the filtered subset; switching
or toggling a filter generally changes the EV scale, since both numerator and
denominator are computed over the new visible set.

This is a deliberate trade-off: prior revisions implemented a filter-independent
anchor lane to keep EV stable across filter toggles. Internal testing showed the
feature was rarely used and the multi-scattering overhead was substantial
(~2× at `ms_prob=0.5`). The anchor lane was removed in
`task-remove-anchor-lane` in favor of the simpler self-P99 path.

---

## 3. References

### Code Paths

| Component | File | Purpose |
|-----------|------|---------|
| Algorithm | `src/gui/gui_ev_auto.hpp` | `ComputeP99Y`, `ComputeEvAuto` |
| P99 computation | `src/gui/server_poller.cpp` | Computes `p99_y` from staged XYZ data |
| EV source | `src/gui/app.cpp` — `SyncFromPoller()` | Maps `p99_y` + `snapshot_intensity` → `ev_auto` |
| GUI display | `src/gui/app_panels.cpp` | `(+N.NN EV auto)` text and `ev_total = exposure_offset + ev_auto` |
| C API fields | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `xyz_buffer`, `snapshot_intensity` |

### Related Documentation

- `doc/filter-architecture.md` — Design A filter semantics
- `doc/configuration.md` — full JSON configuration reference
- `doc/gui-guide.md` — GUI layout and panel overview
