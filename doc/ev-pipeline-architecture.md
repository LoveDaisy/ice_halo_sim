# EV Pipeline Architecture

This document describes the internal invariants of Lumice's EV (Exposure Value)
pipeline — the data flow from raw XYZ accumulation through to the final
`intensity_scale` that controls pixel brightness in every consumption path.

**Target audience**: contributors who modify the rendering pipeline, the C API
surface, or the GUI EV controls.

**Scope**: internal invariants and data-flow contracts. This document does **not**
cover:

- User-visible Adaptive Brightness behavior, the P99 anchor algorithm, or the
  brightness-target rationale — see [`doc/adaptive-brightness.md`](adaptive-brightness.md).
- Simulator-side filter routing (Design A) and the `CollectData` branch table
  — see [`doc/filter-architecture.md`](filter-architecture.md).

> **Historical note**: earlier revisions of this pipeline carried a second
> "F1 anchor lane" that accumulated filter-fail emission separately to produce a
> filter-independent P99.5 statistic. That lane was **removed in PR #115**
> (`task-remove-anchor-lane`, commit `6375ee0`). The current pipeline is a single
> lane that anchors EV on the visible (filtered) framebuffer's own P99. If you find
> stray references to `anchor_d_`, `anchor_internal_xyz_`, `anchor_p995_y`, or a
> "dual lane", they are stale — the live code has none of these.

---

## §1 Terminology

| Term | Definition |
|------|------------|
| **internal buffer** | Per-`Consume()` accumulation target (`internal_xyz_`). Written under the consumer mutex; never read by the GUI. |
| **snapshot buffer** | Point-in-time copy of the internal buffer (`snapshot_xyz_`). Produced by `PrepareSnapshot()`; read by `GetRawXyzResult()` and the poller. |
| **EV anchor (P99)** | The P99 of the non-zero Y channel of the snapshot, computed **GUI-side** in the poller thread (`ComputeP99Y`, `gui_ev_auto.hpp`). Drives `ev_auto`. Not a server-side field. |
| **`kNormScale`** | Display brightness baseline constant (0.08). Maps per-pixel radiance to a [0, 1] range at EV = 0 so that the average illuminated pixel is ~5% brightness and bright halo features (~20× average) approach full white. Independent of resolution and FOV. Defined in `render.cpp:30`. |

---

## §2 Data Flow

### §2.1 Per-Consume Accumulation

`RenderConsumer::Consume()` (`render.cpp:336`) processes one batch of simulator
output. There is a **single accumulation lane**:

- Projects `data.outgoing_d_` / `data.outgoing_w_` (already filter-gated by the
  simulator — Design A, see `filter-architecture.md §2`) through the lens projection.
- Compacts valid in-viewport pixels and accumulates via
  `SpectrumToXyz()` → `internal_xyz_`.
- `total_intensity_ += landed_weight` (sum of all landed ray weights).

**Design A**: the filter runs simulator-side, so all rays in `data.outgoing_*` have
already passed the filter. Filter-fail rays are terminated in the simulator (they are
never emitted to the consumer); there is no second buffer for them. The consumer
simply projects and accumulates.

**Linearity invariant**: accumulation is linear in XYZ space — each ray's
contribution is `CMF(wavelength) × weight`, summed per pixel.

**Overlap dual-write** (`render.cpp:452-486`): for dual fisheye lens types, pass 2
re-projects overlap-zone rays to the opposite hemisphere, filling the ring
`r ∈ (r_scale, 1]`. This pass does **not** update `total_intensity_` — it preserves
normalization (the overlap ring is a small geometric artifact).

### §2.2 PrepareSnapshot and Effective Pixels

`PrepareSnapshot()` (`render.cpp:508`) creates a point-in-time snapshot under the
consumer mutex. It is intentionally minimal — no percentile is computed server-side:

1. `snapshot_xyz_` ← memcpy of `internal_xyz_`.
2. `snapshot_intensity_` ← `total_intensity_`.

`CountEffectivePixels()` (`render.cpp:515`, runs outside the mutex) then counts
non-zero pixels into `effective_pix_` (floored to ≥ 1), used for stats display.

**Zero-skip semantics**: the EV anchor population excludes pixels with zero Y. This
exclusion happens GUI-side in `ComputeP99Y()` (`gui_ev_auto.hpp`, §2.5), which only
pushes `Y > 0` entries (or `> 0` coarse bins under downsampling) into the percentile
population. The server snapshot itself is not filtered — it carries the full buffer.

### §2.3 kNormScale and Per-Pixel Intensity

`GetRawXyzResult()` (`render.cpp:593`) converts the raw accumulated
`snapshot_intensity_` to a per-pixel intensity:

```
per_pixel_intensity = snapshot_intensity_ / (kNormScale × total_pixels)
```

The `kNormScale` factor (0.08) sets the display brightness baseline: at EV = 0, the
average illuminated pixel is ~5% brightness. In the GUI's `intensity_scale`
computation, `kNormScale` algebraically cancels because `ComputeEvAuto()` receives
the per-pixel `snapshot_intensity` (which already incorporates `kNormScale`) as its
denominator input — the resulting `ev_auto` absorbs the scale so that
`2^ev_auto / snapshot_intensity` is independent of the specific `kNormScale` value.
**C API consumers** receive `snapshot_intensity` values whose absolute magnitude is
determined by `kNormScale`; this constant is not an arbitrary scale factor and must
not be changed without re-calibrating the brightness baseline.

### §2.4 C API Surface

`LUMICE_RawXyzResult` (`lumice.h:66-78`) exposes the following EV-relevant fields:

| Field | Source |
|-------|--------|
| `xyz_buffer` | `snapshot_xyz_` (filtered XYZ data; `NULL` sentinel) |
| `snapshot_intensity` | Per-pixel landed intensity (`snapshot_intensity_ / (kNormScale × total_pixels)`) |
| `intensity_factor` | CLI/config EV factor `2^EV` |
| `has_valid_data` | Non-zero once simulation has produced data (reset on `CommitConfig`/`Stop`) |
| `snapshot_generation` | Increments per snapshot; compare to detect data changes |
| `effective_pixels` | Non-zero pixel count (stats display) |

The EV anchor (P99) is **not** a C API field on the mono / full-spectrum path — it is
derived GUI-side from `xyz_buffer` (§2.5). C API consumers that need an anchor for the
mono path must compute their own.

> **Composite-path exception (task-345.3)**: `LUMICE_RenderResult::composite_p99_y`
> IS a C API field, but ONLY on the composite path (`LUMICE_GetCompositeResults` /
> `LUMICE_GetRawXyzAndCompositeResults`'s `composite_out`). Semantics:
>
> - **Composite path** — populated with the P99 over the union of NON-ZERO
>   UNEXPOSED (raw lane) Y values across every participating color class (visible or
>   solo). This is the anchor the GUI's auto-EV feeds into `ComputeEvAuto` for the
>   composite display.
> - **Mono path** (`LUMICE_GetRenderResults`) — always `0`; consumers must ignore it
>   on the mono getter.
>
> The carve-out exists because the per-color-class lane data does not cross the C API
> boundary (`src/gui/` may only see the C API surface), so the GUI cannot derive this
> statistic itself the way it does for the mono path. Also, the composite path needs
> a "participating classes union" anchor rather than the mono full-spectrum P99: mixing
> non-participating pixels back in was the "composite too dim" root cause task-345.3
> fixes. The algorithm (`nth_element` at `⌊count × 0.99⌋`) is structurally identical to
> the fine path of `ComputeP99Y` in `gui_ev_auto.hpp` — the two implementations live
> apart because pulling a shared header down would drag one layer into the other. If
> you touch one, mirror the change in the other file (cross-reference comments in
> both locations).

**Lifetime**: the `xyz_buffer` pointer is valid until the next
`LUMICE_GetRawXyzResults()` or `LUMICE_CommitConfig()` call.

**Composite EV setter**: `LUMICE_SetCompositeExposure(server, ev_total)` (task-345.3) is
the display-time counterpart to `LUMICE_SetRaypathColors` — the GUI uses it to push the
combined manual + auto EV onto the composite bake. `ev_total` is applied as `2^ev_total`
inside the compositor as a single global scalar shared by every color class (per-lane
renormalization stays structurally excluded — that was the false-color bug from the
scrum-336 spike). No accumulator reset, no epoch bump; flips `snapshot_dirty_` so the
next `Get*Results` rebakes the composite. Mono path is untouched.

### §2.5 GUI Poller and SyncFromPoller

The EV anchor is computed on the **poller thread**, not the server. In
`ServerPoller` (`server_poller.cpp:215`):

```
staged_.p99_y = ComputeP99Y(xyz_data, width, height, kEvAutoDownsampleFactor)
```

`ComputeP99Y()` (`gui_ev_auto.hpp:76`) with `kEvAutoDownsampleFactor = 8`:

1. Box-sum downsamples the Y channel onto a coarse `(w/8) × (h/8)` grid
   (`DownsampleBoxSumY`; trailing rows/cols that don't divide evenly are dropped).
2. Takes the P99 over the **non-zero coarse bins** (`nth_element` at
   `⌊count × 0.99⌋`, clamped to `count - 1`).
3. Divides by `f² = 64` to return a **fine-equivalent P99**.
4. Falls back to the fine per-pixel P99 path if `f ≤ 1` or the coarse grid collapses.

Rationale (`scrum-auto-ev-77halo-followup`): coarse bins have `f²` larger expected
hit count, so the P99-over-lit anchor stabilises earlier in sparse scenes and previews
brighten faster, while staying mathematically equivalent to the fine anchor:
`ev = log2(target_linear × snapshot_fine / (P99_coarse / f²))`.

> Because of the `/f²` rescale, `PollerData.p99_y` is **only** an EV anchor, not a
> raw per-pixel Y measurement — downstream consumers must not treat it as one.

`SyncFromPoller()` (`app.cpp:741`) then transfers the result to GUI state with a
**single path** (no filter/no-filter branch):

```
g_state.snapshot_intensity = data.snapshot_intensity
g_state.p99_raw_y          = data.p99_y
g_state.ev_auto            = ComputeEvAuto(p99_raw_y, snapshot_intensity, target_white)
```

`ComputeEvAuto()` (`gui_ev_auto.hpp:123`) returns
`log2(target_linear / (p99_raw_y / snapshot_intensity))`, clamped to `[-6, 6]`, or
`0` if either input is non-positive.

---

## §3 Snapshot and Reset Lifecycle

### §3.1 Buffer Allocation

`RenderConsumer` allocates its four buffers eagerly in the constructor
(`render.cpp:321-333`), sized to `resolution[0] × resolution[1] × 3`:
`internal_xyz_`, `snapshot_xyz_`, `snapshot_work_`, `snapshot_image_buffer_`. There
is no lazy/conditional allocation — every consumer has exactly one accumulation lane.

### §3.2 Reset Conditions

Two events clear accumulation state:

1. **`Reset()`** (`render.cpp:609`): zeros `total_intensity_`, `snapshot_intensity_`,
   `effective_pix_`, and `internal_xyz_`. It does **not** zero `snapshot_xyz_`
   (the next `PrepareSnapshot()` memcpys over it) and does **not** deallocate buffers.
   `has_ever_consumed_ = false` (set in `Stop()`) ensures `GetRawXyzResults()` reports
   `has_valid_data = false` until new data arrives, preventing stale snapshot reads.

2. **Consumer destruction**: the `RenderConsumer` destructor releases all buffers via
   `unique_ptr` RAII (the full-rebuild path, §6.2).

`Reset()` is called by `ResetWith()` (`render.cpp:621`) — the consumer-reuse path
triggered when `CommitConfig()` calls `Stop()` → `ResetWith()` → `Reset()`.

---

## §4 Three Consumption Paths

Three code paths consume the EV pipeline output and compute `intensity_scale`. All
three use the same formula with a single normalization denominator:

```
ev_total         = exposure_offset + ev_auto
intensity_factor = 2^ev_total
intensity_scale  = intensity_factor / snapshot_intensity   (0 if snapshot_intensity ≤ 0)
```

### §4.1 Display Path

`app_panels.cpp:820-823` — sets the GPU shader uniform each frame:

```cpp
float ev_total = rc.exposure_offset + g_state.ev_auto;
pp.exposure.intensity_factor = std::pow(2.0f, ev_total);
float norm_intensity = g_state.snapshot_intensity;
pp.exposure.intensity_scale = norm_intensity > 0 ? pp.exposure.intensity_factor / norm_intensity : 0.0f;
```

### §4.2 Export Path

`BuildExportParams()` (`app.cpp:284-289`) — constructs `PreviewParams` for PNG export.
Mirrors the display-path formula exactly (`norm_intensity = g_state.snapshot_intensity`).

### §4.3 Screenshot / .lmc Thumbnail

`RefreshCpuTextureForSave()` (`app.cpp:227-248`) — CPU-side XYZ→sRGB for the `.lmc`
thumbnail. Recomputes `intensity_scale` from `xyz_results[0].snapshot_intensity` and
`g_state.ev_auto` via `LUMICE_XyzToSrgbUint8`. This may lag the display path by at most
one frame, which is acceptable for thumbnails (saved on user action, not streamed).

### §4.4 Consistency Invariant

All three paths share the same formula by construction, and all three use
`snapshot_intensity` as the single normalization denominator. Because `ev_auto`'s
numerator (`p99_raw_y`) and this denominator both derive from the **same snapshot**,
the EV is source-coherent without any cross-source guard.

---

## §5 Filter Present vs No Filter (Design A)

Under Design A there is no separate filter-independent statistic — the EV always
anchors on the visible (filtered) framebuffer:

- **Filter present**: the simulator drops filter-fail rays before emission, so the
  consumer only ever sees filter-pass emission in `outgoing_*`. The snapshot, its
  P99 anchor, and `snapshot_intensity` all describe the filtered image.
- **No filter**: every ray passes the (vacuous) filter, so the same single lane
  carries the full emission. No code path differs between the two cases.

This is the substantive change from the removed anchor-lane design: brightness is now
normalized to *what is shown*, not to a reconstructed filter-independent total. A
direct consequence is that switching filters changes the EV anchor (the visible image
changed); this is intentional.

---

## §6 CommitConfig Impact

`CommitConfig()` (`server.cpp:227`) always calls `Stop()` before modifying consumers.
`Stop()` (`server.cpp:487`) transitions the server to stopped and sets
`has_ever_consumed_ = false` (`server.cpp:529`).

### §6.1 Reuse Path

When `NeedsRebuild()` returns false (layout fields unchanged):

```
CommitConfig → Stop() → ResetWith(new_config) → Reset()   (server.cpp:296)
```

`Reset()` zeros the accumulators but retains the buffers for reuse.

### §6.2 Rebuild Path

When `NeedsRebuild()` returns true (layout fields changed):

```
CommitConfig → Stop() → consumers_.clear() → new RenderConsumer(...)   (server.cpp:304)
```

`consumers_.clear()` triggers `~RenderConsumer()`, releasing all buffers via
`unique_ptr` RAII. New consumers start fresh.

### §6.3 Filter Switch Semantics

Filter configuration is **not** in the `NeedsRebuild()` field set — the filter is
applied simulator-side (Design A), not in the consumer. However, filter switches in
the GUI still trigger `CommitConfig()`, which always calls `Stop()` → `Reset()` (or
rebuild). Therefore accumulation always restarts on a filter switch, and the EV anchor
re-derives from the new filtered image (§5).

### §6.4 NeedsRebuild Trigger Fields

`NeedsRebuild()` (`render_config.cpp:165`) compares layout-affecting fields only:

| Field | Triggers rebuild |
|-------|:---:|
| `resolution_` | ✓ |
| `lens_` | ✓ |
| `lens_shift_` | ✓ |
| `view_` | ✓ |
| `visible_` | ✓ |
| `overlap_` | ✓ |
| `background_`, `ray_color_`, `opacity_`, `intensity_factor_`, grids | ✗ (appearance only) |

A `static_assert(sizeof(RenderConfig) == 136)` guards against silent field additions
(`render_config.cpp:167`).

### §6.5 GUI Run Path: `intensity_factor` is Neutral (task-346.1)

The GUI Run path (`FillLumiceConfig` in `src/gui/file_io.cpp`) **always** commits
`renderers[i].intensity_factor = 1.0f`, matching `RenderConfig::intensity_factor_`'s
default. It **never** bakes `exposure_offset` into this field.

Rationale — manual + auto EV in the GUI are entirely display-time:

- Mono preview: `app.cpp RefreshPreviewParams` writes `pp.exposure.intensity_factor = 2^ev_total`
  directly to the shader uniform (§4.1). It never reads `RenderConfig::intensity_factor_`.
- Composite preview: `RenderPreviewPanel` pushes `LUMICE_SetCompositeExposure(ev_total)`
  per frame → `display_ev_total_` → `display_exposure_scale = 2^display_ev_total_` →
  the compositor multiplies it into the single shared exposure scalar
  `s = ExposureScale() × display_exposure_scale` (§2.4 composite-path exception).
  `ExposureScale() = config_.intensity_factor_ × kNormScale × total_pix / snapshot_intensity_`
  (`render.cpp:93`); if `intensity_factor_` also carried `2^exposure_offset`, the manual EV
  portion would be counted twice in `s`, producing 2× amplification on Re-Run.

The **CLI/config export path** (`SerializeCoreConfig` in `src/gui/file_io.cpp`) is
different — it DOES bake `2^exposure_offset` into the exported JSON's
`render[].intensity_factor`. That is the legal semantic for CLI (which has no
display-time EV concept): a config exported at GUI EV=E must reproduce the same
brightness when re-run via CLI. Two paths, two semantics for the same field, both
intentional. Cross-referenced in the code comments of both call sites.

Regression pins:

- `test/gui/functional/test_gui_import_export.cpp` →
  `intensity_factor_ignores_exposure_offset_in_gui_run_path` (AC5 mechanism-layer).
- `test/gui/functional/test_gui_composite_preview.cpp` →
  `rerun_with_same_ev_produces_identical_composite` (AC1 end-to-end),
  `display_time_visibility_reanchors_participating_p99` (AC2 display-time re-anchor).

---

## §7 Source Reference Table

| Invariant / Concept | Source Location |
|---|---|
| `kNormScale` definition (0.08) | `render.cpp:30` |
| `Consume()` — single-lane accumulation | `render.cpp:336` |
| Overlap dual-write (pass 2) | `render.cpp:452-486` |
| `PrepareSnapshot()` — snapshot + intensity | `render.cpp:508` |
| `CountEffectivePixels()` | `render.cpp:515` |
| `PostSnapshot()` — CLI/server 8-bit image | `render.cpp:528` |
| `GetRawXyzResult()` — per-pixel intensity formula | `render.cpp:593` |
| `Reset()` — zeroing semantics | `render.cpp:609` |
| `ResetWith()` — config update + reset | `render.cpp:621` |
| Constructor — eager buffer allocation | `render.cpp:321-333` |
| `RawXyzResult` (internal struct) | `server.hpp:134` |
| `LUMICE_RawXyzResult` (C API) | `lumice.h:66-78` |
| `CommitConfig()` — Stop→Reset/Rebuild | `server.cpp:227` |
| `Stop()` — `has_ever_consumed_` reset | `server.cpp:487,529` |
| `NeedsRebuild()` — layout field comparison | `render_config.cpp:165-175` |
| `sizeof(RenderConfig)` static_assert (136) | `render_config.cpp:167` |
| `DownsampleBoxSumY()` / `ComputeP99Y()` / `ComputeEvAuto()` | `gui_ev_auto.hpp:27,76,123` |
| `kEvAutoDownsampleFactor` (8) | `gui_ev_auto.hpp:19` |
| Poller P99 anchor computation | `server_poller.cpp:215` |
| `SyncFromPoller()` — ev_auto computation | `app.cpp:741` |
| `BuildExportParams()` — export EV consistency | `app.cpp:284-289` |
| `RefreshCpuTextureForSave()` — .lmc thumbnail EV | `app.cpp:227-248` |
| Display path `ev_total` / `intensity_scale` | `app_panels.cpp:820-823` |
