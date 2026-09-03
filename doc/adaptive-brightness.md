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

The auto anchor itself is always computed and always shown — there is no toggle for
*that*. What became a toggle is whether the exposure it feeds actually applies: the
Display group's **Mode** control (Relative / Absolute, see §3) selects between two
exposure formulas, and this document's P99 self-anchor is consumed by only one of
them. Everything in §2 below describes the `Relative` mode, which is the default and
is unchanged from the single-mode behavior this document originally documented.

---

## 2. Algorithm (Relative Mode)

### 2.1 P99-Anchored Normalization

The anchor is a **property of the scene, not of the frame**: it is a P99 sky radiance measured
once per snapshot by the server, on a fixed full-sky equal-area buffer that no lens, FOV, camera
pose, `visible` clip or resolution touches, and the CLI divides by the very same number. The GUI
does not compute a P99 of its own; it reads `LUMICE_RawXyzResult::anchor_l99_sky` and converts it
into the units of the texture it is displaying. See
[`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §2.5 and §2.8 for the mechanism,
the migration law, and why the GUI's conversion is a multiplication where the CLI's is a division.

> ⚠️ This replaced a P99 the GUI took over its own simulation texture. That earlier rule made the
> displayed brightness a statement about *the lens looking at the sky* rather than about the sky,
> which is why the GUI and the CLI carried a constant gain against each other and why moving the
> `sim_resolution` slider used to re-expose the preview.

The core algorithm (`ComputeEvAuto`, `src/core/ev_anchor.hpp` — reached via the C API,
`LUMICE_ComputeEvAuto`) is then:

1. **Take** the published anchor, converted into the displayed texture's units:
   ```
   y_p99 = axis_solid_angle × anchor_l99_sky
   ```
   (both fields come from the same `LUMICE_RawXyzResult` row; `axis_solid_angle` is the solid
   angle one on-axis texel of that texture subtends).
2. **Normalize** relative to the per-pixel landed intensity:
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

The anchor is carried out of the server by the poller thread and used unchanged;
`snapshot_intensity` is the per-pixel landed intensity returned alongside it. All three fields are
populated unconditionally — there is no filter-dependent branching.

```cpp
g_state.p99_raw_y = payload->axis_solid_angle * payload->anchor_l99_sky;
g_state.ev_auto = ComputeEvAuto(g_state.p99_raw_y, g_state.snapshot_intensity, target_white);
```

(The composite / raypath-colour preview is the one exception: it anchors to `composite_p99_y`,
the P99 over the participating class lanes, which is already in the buffer's units and needs no
conversion.)

### 2.3 Filter Interaction

When a ray-path filter is active, only filter-pass rays accumulate at all (Design A: filter-fail
rays terminate immediately in `CollectData`), and that includes the anchor buffer. The anchor /
`snapshot_intensity` pair therefore tracks the filtered subset; switching or toggling a filter
generally changes the EV scale, since both numerator and denominator are computed over the new
set of rays.

This is a deliberate trade-off: prior revisions implemented a filter-independent
anchor lane to keep EV stable across filter toggles. Internal testing showed the
feature was rarely used and the multi-scattering overhead was substantial
(~2× at `ms_prob=0.5`). The anchor lane was removed in
`task-remove-anchor-lane` in favor of the simpler self-P99 path.

> ⚠️ **Do not confuse this with `Absolute` mode's filter-independence (§3).**
> `Absolute` mode's normalization is filter-independent — a filter changing what
> lands on a pixel no longer changes the display scale, because the denominator is
> what the source emitted, counted before the filter runs. That is **not** a
> revival of the removed anchor lane above: the anchor lane kept the *self-P99
> anchor* stable across filter toggles by accumulating a second, filter-independent
> emission statistic at real multi-scattering cost (rays had to keep tracing after
> filter-fail to feed it). `Absolute` mode does not touch the P99 anchor or
> `ev_auto` at all — it is not consumed in that mode (§3) — and it costs nothing
> extra: emitted energy is already known at emission time, before any ray is
> traced. These two "filter-independent" properties look similar from the outside
> but come from different mechanisms with different costs; do not treat one as a
> reintroduction of the other.

---

## 3. Absolute Mode

`Absolute` is the second value of the Display group's **Mode** control (the first is
`Relative`, §1–2, and remains the default). It does not change the algorithm in §2 —
`ev_auto` is still computed from the P99 anchor exactly as before — it changes
whether the computed exposure *uses* it:

- The exposure denominator switches from per-pixel landed intensity
  (`snapshot_intensity`) to the energy the light source **emitted**
  (`snapshot_emitted_energy`, from `LUMICE_RawXyzResult::emitted_energy`) — fixed by
  the source and the ray budget, unaffected by filters, scene pass rate, or lens
  clipping. This is what makes two differently-configured renders at the same EV
  comparable in absolute brightness, which is the feature `Absolute` mode exists
  for. See [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §7 for
  the exact definition, the measured `landed_fraction` displacement law, the
  documented cross-lens boundary (§7.3), and why an undersampled scene still darkens
  as `ray_num` grows in `Absolute` mode — a Monte-Carlo estimator property `Relative`
  happens to compensate for, not a defect `Absolute` introduces (§7.4).
- `ev_auto` is **not** added to the exposure. Folding a per-frame auto anchor into a
  value meant to read as "stops above or below physical" would put the reading back
  on a moving baseline, defeating the point. The auto anchor is still computed and
  still shown next to the EV readout — labelled as not applied — because a value the
  UI stops updating still occupies a label, and leaving it unlabelled would look
  like it silently stopped working.
- The EV readout itself changes shape: `Relative` shows manual + auto = effective
  (three numbers, because the auto anchor moves underneath the manual one by
  potentially many stops in a sparse scene); `Absolute` shows the manual offset
  alone, against physical.

The composite (raypath-color) path selects the same way, sharing the mono path's
absolute scale rather than deriving its own (see
[`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §2.6) — mono and
composite stay comparable at one EV in `Absolute` mode too.

`ev_mode` is a per-document field saved with the config/`.lmc`, not a global
preference, and is excluded from the resimulation field set — switching it repaints
the current frame immediately without triggering a re-run. See
[`doc/configuration.md`](configuration.md) for the config key and
[`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) for the full
mechanism (§2.6, §4, §7).

---

## 4. References

### Code Paths

| Component | File | Purpose |
|-----------|------|---------|
| Algorithm (single owner) | `src/core/ev_anchor.hpp` | `ComputeP99Y`, `ComputeEvAuto`, reached via C API |
| Anchor measurement | `src/server/anchor_consumer.cpp`, `src/core/anchor_buffer.hpp` | One full-sky plane per session; publishes `anchor_l99_sky` |
| Anchor transport | `src/gui/server_poller.cpp` | Carries `anchor_l99_sky` + `axis_solid_angle` into `TexturePayload`; computes no statistic |
| EV source | `src/gui/app.cpp` — `SyncFromPoller()` | Maps `p99_y` + `snapshot_intensity` → `ev_auto` |
| Mode-aware exposure (single owner) | `src/gui/mono_exposure_scale.hpp` | `ComputeMonoExposure()` — branches on `ev_mode`; feeds display, export, and `.lmc` thumbnail |
| GUI display | `src/gui/app_panels.cpp` | Mode combo, EV readout text |
| C API fields | `src/include/lumice.h` — `LUMICE_RawXyzResult` | `xyz_buffer`, `snapshot_intensity`, `emitted_energy` |
| C API fields | `src/include/lumice.h` — `LUMICE_RenderParam` | `ev_mode` (`LUMICE_EV_MODE_RELATIVE` / `LUMICE_EV_MODE_ABSOLUTE`) |

### Related Documentation

- [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) — full internal mechanism, both modes, the emitted-energy definition and its honest boundary (§7)
- `doc/filter-architecture.md` — Design A filter semantics
- `doc/configuration.md` — full JSON configuration reference
- `doc/gui-guide.md` — GUI layout and panel overview
