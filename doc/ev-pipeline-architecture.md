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
| **EV anchor (P99)** | The P99 of the non-zero Y channel of the snapshot, computed via `LUMICE_ComputeP99Y` (algorithm: `core/ev_anchor.hpp`) — GUI-side for the mono preview, server-side (`ComputeParticipatingP99Y`) for the composite path. Drives `ev_auto`. Consumed only by the `kRelative` anchor (§2.6); `kAbsolute` does not read it for the exposure scale itself, though the GUI still computes and displays it (§2.6). |
| **`kNormScale`** | Display brightness baseline constant (0.08), defined once in `core/color_util.hpp`. Maps per-pixel radiance to a [0, 1] range at EV = 0 so that the average illuminated pixel is ~5% brightness and bright halo features (~20× average) approach full white. Independent of resolution and FOV. Under `ev_mode = kAbsolute` it is a live factor of `ExposureScale()` (§2.3, §2.6); under `kRelative` it only reaches the C API caller inside `snapshot_intensity` and cancels out of the self-anchored formula, exactly as before this pipeline had two modes. |
| **`ev_mode`** | `RenderConfig::EvMode` (`kRelative` default / `kAbsolute`) — a first-class, appearance-only config field (does not trigger `NeedsRebuild`) selecting which exposure anchor `ExposureScale()`/`CompositeAnchorScale()` use. See §2.6. |
| **emitted energy** | `SimData::emitted_energy_`, summed into `RenderConsumer::total_/snapshot_emitted_energy_` and exposed as `LUMICE_RawXyzResult::emitted_energy`. The total spectral energy the light source emitted into the snapshot — fixed by the source and the ray budget, unaffected by filters, scene pass rate, or lens clipping. The `kAbsolute` denominator. See §2.6 and §7. |

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
exclusion happens in `LUMICE_ComputeP99Y()` (algorithm: `core/ev_anchor.hpp`, §2.5), which only
pushes `Y > 0` entries (or `> 0` coarse bins under downsampling) into the percentile
population. The server snapshot itself is not filtered — it carries the full buffer.

### §2.3 kNormScale and Per-Pixel Intensity

`GetRawXyzResult()` (`render.cpp:593`) converts the raw accumulated
`snapshot_intensity_` to a per-pixel intensity:

```
per_pixel_intensity = snapshot_intensity_ / (kNormScale × total_pixels)
```

The `kNormScale` factor (0.08, `core/color_util.hpp`) sets the display brightness
baseline: at EV = 0, the average illuminated pixel is ~5% brightness. It was
calibrated on full-sphere views (§7), where `landed_fraction ≈ 0.98` — i.e. where
the `kRelative` and `kAbsolute` denominators nearly coincide — so it stayed at 0.08
rather than being re-derived when the `kAbsolute` mode was introduced (re-deriving
it would have moved full-sphere scenes by only +0.029 stop at the cost of
re-shooting every reference image; §7).

Whether `kNormScale` cancels out depends on `ev_mode` (§2.6):

- **`kRelative`**: it cancels. `ComputeEvAuto()` receives the per-pixel
  `snapshot_intensity` (which already incorporates `kNormScale`) as its denominator
  input, so the resulting `ev_auto` absorbs the scale and `2^ev_auto /
  snapshot_intensity` is independent of the specific `kNormScale` value.
- **`kAbsolute`**: it does **not** cancel. `ExposureScale()`'s absolute branch is
  `intensity_factor × kNormScale × total_pix / snapshot_emitted_energy_` — `kNormScale`
  is a live multiplicative factor of the displayed brightness, not an artifact that
  disappears algebraically. Changing it moves every `kAbsolute` render.

**C API consumers**: `snapshot_intensity`'s absolute magnitude is always determined
by `kNormScale`, in both modes; `emitted_energy` (§2.4, §7) is not — it is a raw sum,
unscaled by `kNormScale`, by construction so that a consumer reproducing
`ExposureScale()`'s formula supplies the `kNormScale` factor itself. This constant is
not an arbitrary scale factor and must not be changed without re-calibrating the
brightness baseline and re-shooting every visual reference.

### §2.4 C API Surface

`LUMICE_RawXyzResult` (`lumice.h:66-78`) exposes the following EV-relevant fields:

| Field | Source |
|-------|--------|
| `xyz_buffer` | `snapshot_xyz_` (filtered XYZ data; `NULL` sentinel) |
| `snapshot_intensity` | Per-pixel landed intensity (`snapshot_intensity_ / (kNormScale × total_pixels)`) |
| `intensity_factor` | CLI/config EV factor `2^EV` |
| `has_valid_data` | Non-zero once simulation has produced data (reset on commit / `Stop`) |
| `snapshot_generation` | Increments per snapshot; compare to detect data changes |
| `effective_pixels` | Non-zero pixel count (stats display) |
| `emitted_energy` | `snapshot_emitted_energy_` — total spectral energy the light source EMITTED into the snapshot, raw (NOT divided by `kNormScale × total_pixels`, unlike `snapshot_intensity` above). The `kAbsolute` denominator; see §7. |

The EV anchor (P99) is **not** a C API field on the mono / full-spectrum path — it is
derived GUI-side from `xyz_buffer` (§2.5). C API consumers that need an anchor for the
mono path must compute their own.

> **Composite-path exception (task-345.3)**: `LUMICE_RenderResult::composite_p99_y`
> IS a C API field, but ONLY on the composite path — today that is
> `LUMICE_FrameGetComposite(frame, out, max_count)` (before v4.15: `LUMICE_GetCompositeResults`
> / `LUMICE_GetRawXyzAndCompositeResults`'s `composite_out`; the pairing getter is gone because
> any two reads off ONE frame are same-generation by construction). Semantics:
>
> - **Composite path** — populated with the P99 over the union of NON-ZERO
>   UNEXPOSED (raw lane) Y values across every participating color class (visible or
>   solo). This is the anchor the GUI's auto-EV feeds into `ComputeEvAuto` for the
>   composite display.
> - **Mono path** (`LUMICE_FrameGetRender`) — always `0`; consumers must ignore it
>   on the mono read.
>
> The carve-out exists because the per-color-class lane data does not cross the C API
> boundary (`src/gui/` may only see the C API surface), so the GUI cannot derive this
> statistic itself the way it does for the mono path. Also, the composite path needs
> a "participating classes union" anchor rather than the mono full-spectrum P99: mixing
> non-participating pixels back in was the "composite too dim" root cause task-345.3
> fixes. The algorithm (`nth_element` at `⌊count × 0.99⌋`) has a **single owner**,
> `core/ev_anchor.hpp::NthElementP99`; both this composite anchor
> (`ComputeParticipatingP99Y`) and the mono-path `LUMICE_ComputeP99Y` call it, and what
> stays local to each is only the question of which values participate. No mirror
> discipline applies — there is one implementation to change.

**Lifetime**: the `xyz_buffer` pointer is valid **for as long as the caller holds the
`LUMICE_ResultFrame` it was read out of** — acquire with `LUMICE_AcquireResultFrame`, hand it
back with `LUMICE_ReleaseResultFrame`, and copy anything you still need before that Release.
A frame is immutable and separately reference-counted, so a later snapshot publishes a *new*
frame and cannot disturb this one.

> ⚠️ Before v4.15 this line read "valid until the next `LUMICE_GetRawXyzResults()` or
> `LUMICE_CommitScene()` call". **Do not write code to that contract** — it is the exact
> contract the frame model replaced, because it describes when the memory happens to still
> be there rather than a lifetime the reader controls; two use-after-free recurrences came
> out of it. See [`doc/capi-lifecycle-architecture.md`](capi-lifecycle-architecture.md) §9.

**Composite EV setter**: `LUMICE_SetCompositeExposure(server, ev_total)` (task-345.3) is
the display-time counterpart to `LUMICE_SetRaypathColors` — the GUI uses it to push the
combined manual + auto EV onto the composite bake. `ev_total` is applied as `2^ev_total`
inside the compositor as a single global scalar shared by every color class (per-lane
renormalization stays structurally excluded — that was the false-color bug from the
scrum-336 spike). No accumulator reset, no epoch bump; flips `snapshot_dirty_` so the
next result-frame acquisition rebakes the composite. Mono path is untouched.

### §2.5 GUI Poller and SyncFromPoller

The EV anchor is computed on the **poller thread**, not the server. In
`ServerPoller::PollOnce` (`server_poller.cpp:513`):

```
staged_.p99_y = ComputeP99Y(xyz_data, width, height, kEvAutoDownsampleFactor)
```

`LUMICE_ComputeP99Y()` (C API; algorithm in `core/ev_anchor.hpp`) with
`kEvAutoDownsampleFactor = 8` (`gui_constants.hpp` — the call site's choice of the coarse
branch, not part of the algorithm):

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

`LUMICE_ComputeEvAuto()` (C API; algorithm in `core/ev_anchor.hpp`) returns
`log2(target_linear / (p99_raw_y / snapshot_intensity))`, clamped to `[-6, 6]`, or
`0` if either input is non-positive.

Note: `ev_auto` is computed and displayed in **both** `ev_mode`s (§2.6), but it is
only ever consumed by the `kRelative` exposure formula. Under `kAbsolute` the GUI
still shows its value (labelled as not applied — `mono_exposure_scale.hpp`'s
`FormatMonoEvReadout`), because a value the UI stops updating still occupies a
label; showing it prevents that label from going stale-looking.

### §2.6 Two Anchors: `ev_mode` (Server-Side)

`RenderConfig::ev_mode_` (`kRelative` default / `kAbsolute`) selects which of two
independent anchors `ExposureScale()` and `CompositeAnchorScale()`
(`server/render.{hpp,cpp}`) compute. They are not two tunings of one formula — they
anchor to different physical quantities:

```
kRelative (default):
    ExposureScale() = intensity_factor_ × TargetWhiteToLinear(kAnchorTargetWhite)
                       / ComputeP99Y(snapshot, kMonoAnchorDownsampleFactor)
    — self-anchored to THIS frame's own P99. Algebraically identical to what
      §2.5's GUI two-hop computation produces (the snapshot_intensity factor
      cancels between ComputeEvAuto's numerator and the shader's
      intensity_scale = intensity_factor / snapshot_intensity step), so a CLI
      render in kRelative reproduces what the GUI displays for the same
      snapshot — see §6.5. Being self-anchored, it carries no energy term: the
      picture keeps its look as ray_num grows, and the config alone does NOT
      determine output brightness (ray_num co-determines it).

kAbsolute:
    ExposureScale() = intensity_factor_ × kNormScale × total_pix
                       / snapshot_emitted_energy_
    — anchored to the energy EMITTED (§7), not landed. Fixed by the light source
      and the ray budget alone, so it does not move when a filter, a low scene
      pass rate, or lens clipping removes rays — which is what makes two
      differently-configured scenes comparable at one EV.
```

`CompositeAnchorScale(participating_p99_y)` is the single call the compositor
makes; it hides the mode decision from `component_compositor.cpp` entirely:

```
kRelative: ParticipatingExposureScale(participating_p99_y)  — the participating
           pixels (the union of visible/solo color-class lanes) self-anchor, so
           hiding a bright class re-brightens the rest in the same DoSnapshot
           (§6.6).
kAbsolute: ExposureScale()  — the SAME scalar the mono path uses, argument
           ignored. Sharing rather than re-deriving is the point: composite
           lanes are copies of the same accumulated Y that feeds mono, and any
           independently-derived absolute composite formula would differ by
           some coefficient and break the property absolute mode exists for.
```

Both anchor algorithms (`NthElementP99`, `TargetWhiteToLinear`, `ComputeP99Y`,
`ComputeEvAuto`) have a single owner, `core/ev_anchor.hpp`, reached through the C
API (`LUMICE_ComputeP99Y` / `LUMICE_ComputeEvAuto`) by both the GUI (mono preview,
§2.5) and the server (composite path, §6.6). `ev_mode` itself is an
appearance-only field — like `intensity_factor_`, it never triggers
`NeedsRebuild()` (§6.4) because it selects which formula runs, not the
accumulation layout.

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
   `has_ever_consumed_ = false` (set in `Stop()`) ensures `LUMICE_FrameGetRawXyz` reports
   `has_valid_data = false` until new data arrives, preventing stale snapshot reads.

2. **Consumer destruction**: the `RenderConsumer` destructor releases all buffers via
   `unique_ptr` RAII (the full-rebuild path, §6.2).

`Reset()` is called by `ResetWith()` (`render.cpp:621`) — the consumer-reuse path
triggered when `CommitConfig()` calls `Stop()` → `ResetWith()` → `Reset()`.

---

## §4 Three Consumption Paths

Three GUI code paths consume the EV pipeline output and compute an
`intensity_factor` / `intensity_scale` pair for the **mono** preview. All three call
the same function, `ComputeMonoExposure()` (`src/gui/mono_exposure_scale.hpp`) —
before this scrum each site carried its own hand-written copy of the formula; three
independent mirrors of one formula is exactly the shape of drift a single shared
function removes.

`ComputeMonoExposure(ev_mode, MonoExposureInput)` branches on `ev_mode` the same way
§2.6's server-side `ExposureScale()` does, because the two must agree bit-for-bit in
`kRelative` (§6.5):

```
kRelative (default, unchanged from before this scrum — every existing document and
           every committed reference image depends on this branch bit-for-bit):
    intensity_factor = 2^(exposure_offset + ev_auto)
    intensity_scale  = intensity_factor / snapshot_intensity
                        (0 if snapshot_intensity ≤ 0)

kAbsolute:
    intensity_factor = 2^exposure_offset          — ev_auto is NOT added (see below)
    intensity_scale  = intensity_factor × kNormScale × total_pixels / snapshot_emitted_energy
                        (0 if snapshot_emitted_energy ≤ 0 or total_pixels ≤ 0)
```

Two differences in the `kAbsolute` branch are both load-bearing, not incidental:

- **`ev_auto` is not added.** The whole promise of `kAbsolute` is that EV reads as
  "stops above or below physical"; silently folding in a per-frame auto anchor would
  put the reading back on a moving baseline, and the number would stop meaning
  anything across documents. `ev_auto` is still computed and still shown in the UI
  (§2.5), labelled as not applied.
- **The denominator is emitted energy, not landed.** Emitted is fixed by the light
  source and the ray budget (§7), so it does not drift as filters remove rays or as
  the accumulation runs — which is precisely what makes two differently-configured
  scenes comparable at one EV.

Both branches keep the guard the inline call sites had before consolidation: a
non-positive denominator yields a scale of 0 rather than an infinity that would paint
the frame white.

### §4.1 Display Path

`app.cpp` / `app_panels.cpp` — sets the GPU shader uniform each frame via
`ComputeMonoExposure(g_state.ev_mode, {...})`, feeding
`pp.exposure.intensity_factor` / `intensity_scale` from the result.

### §4.2 Export Path

`BuildExportParams()` (`app.cpp`) — constructs `PreviewParams` for PNG export. Calls
the same `ComputeMonoExposure()`, so an exported PNG matches the on-screen preview in
both modes.

### §4.3 Screenshot / .lmc Thumbnail

`RefreshCpuTextureForSave()` (`app.cpp`) — CPU-side XYZ→sRGB for the `.lmc`
thumbnail. Calls `ComputeMonoExposure()` and feeds the result to
`LUMICE_XyzToSrgbUint8`. This may lag the display path by at most one frame, which is
acceptable for thumbnails (saved on user action, not streamed).

### §4.4 Consistency Invariant

All three paths call the same function by construction, so they cannot drift from
each other. Within `kRelative`, `ev_auto`'s numerator (`p99_raw_y`) and the
`snapshot_intensity` denominator both derive from the same snapshot, so the EV is
source-coherent without any cross-source guard. Within `kAbsolute`,
`snapshot_emitted_energy` and `total_pixels` both come from the same
`LUMICE_RawXyzResult` read.

There is no ABI or C API involved in this GUI-side sharing — `mono_exposure_scale.hpp`
is header-only and free of ImGui/GL/server dependencies precisely so it can be unit
tested (`test/unit-correctness/gui/test_mono_exposure_scale.cpp`) without a window.
The **server-side** anchor (`ExposureScale()` / `CompositeAnchorScale()`, §2.6) is a
separate implementation reached by the CLI and by the composite path — not by the
mono preview, which stays on its own display-time recomputation for the
same-frame-responsiveness reason given in §6.5. The two must still agree in
`kRelative`, which is exactly what `GuiConstants.ExposureScaleMirrorsCore`
(`gui_constants.hpp`'s `kNormScale` mirror) and the cross-check tests cited in §6.5
pin.

---

## §5 Filter Present vs No Filter (Design A)

Under Design A there is no separate filter-independent statistic — filter-fail rays
are dropped by the simulator before emission, so the consumer only ever sees
filter-pass emission in `outgoing_*`, in both `ev_mode`s. What differs between the
two modes is what the resulting brightness *means*:

- **`kRelative`**: the EV anchor (P99) and `snapshot_intensity` both describe the
  filtered image, and the self-anchor formula (§2.6) re-normalizes to it — the
  picture keeps roughly the same look whether or not a filter is active, and
  switching filters changes the EV anchor because the visible image changed. This is
  the historical, single-lane behavior this section originally described (no code
  path differs between filter-present and no-filter), and it is still exactly true
  of `kRelative`, which is why it is the default.
- **`kAbsolute`**: the denominator is `snapshot_emitted_energy_` — energy the source
  emitted, counted *before* the filter runs. A filter that rejects most rays now
  makes the image dimmer, by design: fewer photons reached the frame, and the
  absolute scale reports that truthfully instead of re-brightening to compensate.
  This is the entire point of `kAbsolute` — see §7's `landed_fraction` law for how
  much dimmer, and why that is a design outcome and not a defect.

So the old sentence "brightness is now normalized to what is shown" is only true of
`kRelative`. Under `kAbsolute`, brightness is normalized to what was emitted, which is
deliberately **not** the same as what is shown once a filter removes rays.

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
| `background_`, `ray_color_`, `opacity_`, `intensity_factor_`, `ev_mode_`, grids | ✗ (appearance only) |

A `static_assert(sizeof(RenderConfig) == 144)` guards against silent field additions
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
  `s = A × display_exposure_scale`, where `A = RenderConsumer::CompositeAnchorScale(participating_p99)`
  (§2.6 picks the `kRelative`/`kAbsolute` formula for `A`; §2.4 composite-path exception
  covers `participating_p99`). If `intensity_factor_` also carried `2^exposure_offset`, the
  manual EV portion would be counted twice in `s`, producing 2× amplification on Re-Run.

The **CLI/config export path** (`SerializeCoreConfig` in `src/gui/file_io.cpp`) is
different — it DOES bake `2^exposure_offset` into the exported JSON's
`render[].intensity_factor`. That is the legal semantic for CLI (which has no
display-time EV concept): a config exported at GUI EV=E, run through the CLI in
`kRelative` (the default), **does** reproduce the same brightness — this used to be a
promise this document made ahead of the mechanism; it is now a tested fact.
`ComputeMonoExposure`'s `kRelative` branch (§4) and server-side `ExposureScale()`'s
`kRelative` branch (§2.6) are two independent implementations of the same formula on
the same snapshot, and the GUI↔CLI pixel-level comparison in
`test/e2e-correctness` (101.6 dB PSNR against a live GUI render; the residual traces
to floating-point last-bit differences, not an algorithmic divergence) is the
evidence that they agree in practice, not just on paper. `kAbsolute` makes no such
promise across the GUI/CLI boundary: the CLI's `kRelative` self-anchor has no analog
of the GUI slider's own clamping behavior (§2.5's `ComputeEvAuto` clamps to
`[-6, 6]`; the CLI path deliberately does not reproduce that clamp — see
`ExposureScale()`'s comment in `render.cpp`), so CLI and GUI can diverge at the
extremes of the manual EV range even within `kRelative`. Two paths, two semantics for
the same field, both intentional. Cross-referenced in the code comments of both call
sites.

Regression pins:

- `test/composition-correctness/gui/test_scene_commit_chain.cpp` →
  `SceneCommitChain.IntentionalDivergenceFieldsMatchDocumentedSet` (AC5 mechanism-layer).
  Re-anchored 2026-08-10: this proposition used to be pinned by
  `test_gui_import_export.cpp::intensity_factor_ignores_exposure_offset_in_gui_run_path`,
  which the GUI-suite rewrite replaced with a case asserting the same claim plus a field-by-field
  comparison of the two documents with `intensity_factor` removed, so an intent-dependent
  branch appearing anywhere else also failed it.
  Renamed 2026-08-31, when the export intent started describing the on-screen picture rather
  than the simulation's fixed full-sky texture: `intensity_factor` is no longer the only field
  the intent reaches, so the case that was named `OnlyTheExposureDiffersBetweenTheRunAndExportIntents`
  now enumerates the divergence set (lens / view / visible / background / resolution / grid
  alongside `intensity_factor`) and compares everything outside it whole. The EV claim itself is
  unchanged and still asserted by value on both arms — what changed is that the surrounding
  field-by-field comparison now overlooks a named list instead of a single key.
- Resolved 2026-08-11 — this list also named
  `test_gui_composite_preview.cpp::rerun_with_same_ev_produces_identical_composite` (AC1
  end-to-end) and `::display_time_visibility_reanchors_participating_p99` (AC2 display-time
  re-anchor), and the entry standing here until now recorded that no same-named successor had
  been found. Both propositions did survive the GUI-suite rewrite, under new names in
  `test/unit-correctness/gui/test_composite_preview.cpp`:
  `CompositePreview.RerunningAtTheSameExposureReproducesTheSamePicture` and
  `CompositePreview.HidingAClassReAnchorsTheExposureOverWhatIsLeftInBothCombineModes`.
  One difference is worth carrying rather than glossing: the re-run case now asserts a
  brightness RATIO band ([0.8, 1.25], on both the mean byte and the unexposed anchor) where the
  old name promised an identical composite. Two independently seeded accumulations reach IDLE at
  different batch boundaries, so byte-identity was never the property; the band still rules out
  the ~4x this section's bug produced without calling run-to-run noise a regression. The
  mechanism layer below (`test_component_compositor.cpp`) is unaffected and still pins the scale
  arithmetic.

### §6.6 Composite-path server-side self-anchor (task-fix-composite-participating-exposure-anchor)

The composite exposure scalar `s` was originally sourced from `RenderConsumer::ExposureScale()`
— the same mono-path integrated-intensity normalization used by the standalone Y-lane. That
worked for the initial delivery but broke the display-time visibility-toggle behavior owner ⑤
wanted: "hide a bright color class → the remaining dim classes should immediately brighten in
the same DoSnapshot." The pre-fix behavior only shifted the intermediate `composite_p99_y`
proxy that GUI auto-EV read from — the actual composite RGB bytes were still anchored on the
mono integral, so hiding a class did NOT rebrighten the visible ones on-screen. GUI auto-EV
was the only path that could effect the intended rebrightening, and that path was a
multi-frame indirect feedback loop that broke under poller steady-state auto-pause.

Fix B — **serve-side self-anchor at the compositor**:

- `RenderConsumer::ParticipatingExposureScale(participating_p99_y)` (render.cpp) is a sibling
  of `ExposureScale()` that returns `intensity_factor * target_linear / participating_p99_y`,
  where `target_linear` is the sRGB reverse transform of `target_white = 135`, obtained from
  `core/ev_anchor.hpp::TargetWhiteToLinear` — the same sub-piece `ComputeEvAuto` uses. The two
  share that level and the P99 level, and deliberately nothing beyond:
  `snapshot_intensity` does **not** appear in the numerator
  — it cancels against the mono shader's downstream `intensity_scale = intensity_factor /
  snapshot_intensity` step, which the composite path lacks (composite applies `s` directly to
  `lane[p]`).
- `CompositeColorClassesLinear` (component_compositor.cpp) reorders internal steps to
  `GatherActiveClasses → ComputeParticipatingP99Y → ParticipatingExposureScale(p99) *
  display_exposure_scale → per-pixel composite`. Every visibility/solo change flips
  `snapshot_dirty_` and the next `LUMICE_FrameGetComposite` read recomputes p99 over the (now
  smaller) active set, so `s` grows, and the remaining classes' pixels brighten in the same
  call — no GUI round-trip needed.
- **GUI decoupling**: `RenderPreviewPanel` (app_panels.cpp) now pushes only the manual
  `exposure_offset` for the composite path (`LUMICE_SetCompositeExposure(composite_ev_push)`),
  not `ev_total = exposure_offset + ev_auto`. Auto-EV continues to drive only the mono
  shader uniform. Server-side auto-anchoring makes the GUI auto-EV term redundant on the
  composite side, and stripping it prevents the double-exposure (server self-anchor × GUI
  auto-EV) that would otherwise stack.

Invariants preserved:

- Single shared exposure scalar (no per-lane normalization) → hue-angle continues to hold.
- Mono path (`ExposureScale()`, mono shader `intensity_scale`) is not touched, so `AC5`
  ("no raypath_color → mono path zero regression") holds structurally, not by test.
- The `s <= 0` early-return path now writes `*out_participating_p99_y` with the actual
  computed p99 (previously left untouched). No in-tree consumer depended on the untouched
  behavior; the tightening is pinned by `EarlyReturnPublishesParticipatingP99`.

Regression pins:

- `test/unit-correctness/server/test_component_compositor.cpp` →
  `ParticipatingExposureScaleGuards`, `ParticipatingExposureScaleFormulaCrossCheck`
  (mechanism-layer independent recomputation, including a `rc_heavy` sibling asserting
  `snapshot_intensity` does not leak into `s`), `EarlyReturnPublishesParticipatingP99`.
- Resolved 2026-08-11 — this list also named
  `test_gui_composite_preview.cpp::display_time_visibility_reanchors_participating_p99` (AC1
  additive-mode pixel-byte gate) and its `_dominant` counterpart. The GUI-suite rewrite merged
  the pair into a single case that exercises both combine modes on one staging:
  `CompositePreview.HidingAClassReAnchorsTheExposureOverWhatIsLeftInBothCombineModes`
  (`test/unit-correctness/gui/test_composite_preview.cpp`). Both halves stayed pixel-byte
  assertions and are shaped to the way each mode fails — additive asserts the mean blue byte
  over an already-blue population rises by at least 1.3x once the bright class is hidden;
  dominant asserts the argmax at a probed pixel changes hands AND that the new winner is clearly
  lit (byte >= 16), since winning by one unit over black would still be a broken picture.
- CLI e2e: `test/e2e-correctness/test_raypath_color.py` (references regenerated against the
  new anchor; thresholds recalibrated).

---

## §7 Emitted Energy: Definition and Honest Boundary

This section defines the `kAbsolute` denominator precisely and states two things it
deliberately does not compensate for — a spatial one (§7.3, cross-lens) and a
sampling one (§7.4, undersampling) — so neither gets rediscovered as a bug.

### §7.1 Definition

`emitted_energy` is the sum, over every simulated batch consumed since the last
`Reset()`, of `(per-ray emission weight) × (rays emitted in that batch)` — what the
light source put into the simulation, before any ray is traced, filtered, or
clipped by the lens. It is computed on the **emission** side (`Simulator::Run()`'s
dispatch point, `src/util/illuminant.hpp`'s `MeanIlluminantWeight()` for an
illuminant spectrum) and threaded through `SimData::emitted_energy_` in parallel
with — but independent of — `root_ray_count_`, into
`RenderConsumer::total_/snapshot_emitted_energy_`, and out through
`LUMICE_RawXyzResult::emitted_energy`.

For a continuous illuminant spectrum, each batch is charged the spectrum's **band
expectation** `E[w]` (the mean SPD weight over the sampled wavelength range, e.g.
87.9532682 for D65 over `[380, 780]`nm) rather than the weight of the one wavelength
that batch happened to draw. This is deliberate: the CPU backend draws one
wavelength per batch, so charging the actual per-batch weight would make the
denominator (and therefore the displayed brightness) depend on the random
wavelength sequence — the same config at a different seed would render at a visibly
different brightness. The band-expectation charge is deterministic and matches what
a discrete spectrum already does exactly (its `Σw` is the same value regardless of
draw order).

### §7.2 The `landed_fraction` Law

`emitted_energy` and the pre-scrum landed-weight denominator differ by a **per-scene
constant, independent of `ray_num`**:

```
landed_fraction ≜ (energy that landed on a pixel) / (energy emitted)
new_scale / old_scale ≡ landed_fraction
```

Because it is independent of `ray_num`, a caller can compute their own scene's
displacement from the two fields `LUMICE_RawXyzResult` already exposes
(`snapshot_intensity` and `emitted_energy`) without re-deriving anything — this is
more useful than any table of example numbers, though the measured spans below (from
this scrum's calibration corpus) are worth knowing as orientation:

| Scene shape | `landed_fraction` | Brightness shift (`log2(landed_fraction)`) |
|---|---|---|
| Full-sphere view, no filter | ≈ 0.980 (median of 14 gold scenes) | −0.038 … −0.002 stop |
| `fisheye_equal_area` 120°, no filter | 0.42 – 0.63 | −0.67 … −1.25 stop |
| `linear` 80°, no filter | ≈ 0.498 | ≈ −1.0 stop |
| Filter active / high `ms_prob` | 0.00071 – 0.512 | −0.97 … −10.47 stop |

The full-sphere row is why `kNormScale` (0.08, §2.3) did not need re-calibration
when `kAbsolute` was introduced: `landed_fraction` is already close to 1 there, so
the constant that was tuned under the old landed-weight denominator is already
close to what the new emitted-energy denominator wants. Re-deriving it would have
moved full-sphere scenes by only +0.029 stop, at the cost of re-shooting every
committed reference image (a04: the burden of proof is on the side that changes
something, and here the something-to-gain was smaller than the something-to-pay).

A narrow lens or an active filter is not a corner case of this law — it is the part
of the range this scrum's `kAbsolute` mode exists to expose truthfully instead of
compensating for. Under `kRelative`, the self-anchor formula (§2.6) absorbs exactly
this factor, which is why a `kRelative` render's brightness barely moves when you
add a filter; `kAbsolute` has no compensating term, so it shows the light loss.
Neither is "wrong" — they answer different questions ("does this look the way it
always has" vs. "how much light actually got through").

### §7.3 Applicability Boundary: Same-Lens Comparability Only

`kAbsolute` comparability holds **within one lens, one FOV, one resolution**. It does
**not** extend across lenses — two renders of the same scene at 90° `linear` vs. 180°
`fisheye_equal_area` will differ by their own `landed_fraction` ratio (§7.2's table
shows why: `landed_fraction` moves with FOV and projection). This is a **deliberate,
permanent** design boundary, not a gap to be closed later:

1. **A single constant cannot fix it.** Only an equal-area projection has a spatially
   constant per-pixel solid angle `Ω_p`. For `fisheye_equidistant` / `linear` /
   other non-equal-area projections, `Ω_p` varies with pixel position, so a true
   radiance map would need to divide every pixel by its own `Ω_p` — that re-weights
   the entire image and changes its appearance, which is a materially different
   (and much larger) piece of work than a normalization fix.
2. **The typical use case does not need it.** The motivating request was
   "reconfigure the scene at a fixed viewport and compare brightness" — same lens,
   same FOV, same resolution, different crystal/light/filter configuration. That is
   exactly what `kAbsolute` delivers.
3. **The boundary is now written down** (here), which is the alternative this scrum
   chose over a partial fix: a documented limitation costs a paragraph, and a
   `Ω_tot`-only partial fix would have created a false sense of cross-lens
   comparability while leaving the position-dependent part of the error in place —
   worse than not attempting it.

If a future need requires true radiance (per-pixel `Ω_p` division) or cross-lens
comparability, treat it as new scope with its own AC on image-appearance regression,
not as a follow-up to this feature.

### §7.4 Undersampling Darkens Regardless of Denominator — This Is Not a Defect

§7.2 and §7.3 are about a **spatial** dimension: `landed_fraction` differs by lens,
filter, and scene pass rate, and that difference is what makes `kAbsolute`
comparable across configurations. This section is about an orthogonal, **sampling**
dimension, and is easy to conflate with §7.2 because both involve a denominator —
they are not the same claim.

At a fixed EV, an undersampled scene's display **darkens as `ray_num` grows**,
independent of which denominator is used. Measured on the same calibration corpus:
the sparse-scene N-scaling slope is essentially identical for both denominators
(landed-weight: **-1.026**; emitted-energy: **-1.027**, on log-log `display / N`).
The mechanism is ordinary Monte-Carlo behavior, not a normalization artifact: as
`ray_num` grows, a fixed set of lit pixels accumulates more hits each (their
per-pixel value does not grow, because they are already at their converged
intensity), while previously-dark pixels gradually receive their first hit and join
the lit set — so at any fixed EV, the same total energy spreads over more, fainter
pixels, and the bright end of the histogram appears to dim as the sample grows. This
is the renderer displaying its own estimator honestly, not a bug that gets worse the
longer a simulation runs.

Why `kAbsolute` shows this and `kRelative` does not, despite both denominators
having the same slope: `kRelative`'s anchor is `ray_num`-dependent by construction —
it re-derives the P99 from the current frame every time, so as the sparse-pixel
population's own statistics shift, the self-anchor gain (`target_linear / P99`, §2.6)
shifts to compensate, and the picture keeps its look. `kAbsolute`'s anchor
(`snapshot_emitted_energy`) is fixed by the light source and ray budget precisely
because that fixedness is the feature — so nothing in the formula compensates, and
the underlying sampling darkening becomes visible on screen.

The correct framing, and the one to use if this is ever reported: **`kAbsolute`
exposes a fact that was always there and that `kRelative` happens to compensate for
— it is not a defect introduced by `kAbsolute`.** Do not "fix" it by adding an
N-dependent term to `kAbsolute`'s formula; that would just rebuild `kRelative`'s
self-anchor under a different name and defeat the reason `kAbsolute` exists.

---

## §8 Source Reference Table

| Invariant / Concept | Source Location |
|---|---|
| `kNormScale` definition (0.08) | `core/color_util.hpp` |
| `RenderConfig::EvMode` (`kRelative`/`kAbsolute`) | `config/render_config.hpp` |
| `ExposureScale()` / `ParticipatingExposureScale()` / `CompositeAnchorScale()` | `server/render.{hpp,cpp}` |
| `SimData::emitted_energy_` | `config/sim_data.hpp` |
| `MeanIlluminantWeight()` (band-expectation charge) | `util/illuminant.{hpp,cpp}` |
| `LUMICE_RawXyzResult::emitted_energy` (C API) | `include/lumice.h` |
| `LUMICE_RenderParam::ev_mode`, `LUMICE_EV_MODE_*` (C API) | `include/lumice.h` |
| `ComputeMonoExposure()` / `MonoExposureInput` / `MonoEvMode` (GUI mono exposure, single owner) | `gui/mono_exposure_scale.hpp` |
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
| `sizeof(RenderConfig)` static_assert (144) | `render_config.cpp:167` |
| `DownsampleBoxSumY()` / `ComputeP99Y()` / `ComputeEvAuto()` / `NthElementP99()` / `TargetWhiteToLinear()` | `core/ev_anchor.hpp` |
| `LUMICE_ComputeP99Y` / `LUMICE_ComputeEvAuto` (C API surface) | `include/lumice.h`, `c_api.cpp` |
| `kEvAutoDownsampleFactor` (8) | `gui_constants.hpp` |
| Poller P99 anchor computation | `server_poller.cpp` — `ServerPoller::PollOnce()` |
| `SyncFromPoller()` — ev_auto computation | `app.cpp:741` |
| `BuildExportParams()` — export EV consistency | `app.cpp:284-289` |
| `RefreshCpuTextureForSave()` — .lmc thumbnail EV | `app.cpp:227-248` |
| Display path `ev_total` / `intensity_scale` | `app_panels.cpp:820-823` |
