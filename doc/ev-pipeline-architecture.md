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
| `anchor_l99_sky` (v4.22) | The session's exposure anchor: P99 sky radiance per steradian, measured on a fixed full-sky equal-area plane no renderer's lens/fov/view/`visible`/resolution touches. **Frame-level, identical on every row** — it describes the scene, not the row. The `kRelative` numerator's other half. |
| `axis_solid_angle` (v4.23) | Steradians subtended by one on-axis pixel of **this** row's view. **Per-renderer**, unlike the field above. The unit bridge between that radiance and what `xyz_buffer` holds. |

The mono/full-spectrum EV anchor **is** a C API surface as of v4.22/v4.23, and it is the same
one the renderer used rather than an input a consumer has to reconstruct:

```
scale = intensity_factor × TargetWhiteToLinear(135) / ( axis_solid_angle × anchor_l99_sky )
```

Both fields are needed, and a consumer that holds a buffer of its own must use the solid angle of
**the buffer it is exposing** — the GUI preview multiplies by the SIMULATION renderer's
`axis_solid_angle`, not the target lens's, because the texture it exposes is that renderer's
output (§2.5). Before v4.23 only the first field was published, which left the relative branch
the one formula in this API a consumer could not reproduce (§2.8).

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

The GUI does **not** compute an EV anchor. It reads the server's, which is the same scalar
`RenderConsumer::ExposureScale()` divides by, and that is the whole point of the arrangement: two
independently computed anchors over two different pixel populations were argued to be equal and
measurably were not (§2.8).

`ServerPoller::PollOnce` (`server_poller.cpp`) carries two fields out of
`LUMICE_RawXyzResult` into `TexturePayload`:

```
payload->anchor_l99_sky   = xyz_results[0].anchor_l99_sky     // P99 sky RADIANCE, per steradian
payload->axis_solid_angle = xyz_results[0].axis_solid_angle   // sr subtended by one texel of THIS texture
```

`SyncFromPoller()` (`app.cpp`) then multiplies them and hands the product to the same two-hop
computation the GUI has always used:

```
g_state.p99_raw_y = payload->axis_solid_angle * payload->anchor_l99_sky
g_state.ev_auto   = ComputeEvAuto(p99_raw_y, snapshot_intensity, target_white)
```

**Why the multiplication, and why it is not optional.** `anchor_l99_sky` is a radiance (per
steradian); the simulation texture holds a radiance *integrated over each texel's solid angle*.
Converting between them takes exactly one factor and the server publishes it per renderer
(§2.4). The CLI performs the same conversion in the other direction — it *divides* by its own
view's `axis_solid_angle`, because it holds a render rather than a texture — and the two chains
meet on one per-pixel expression (§2.8).

> ⚠️ Omitting the conversion is worth about **15 stops** on the 1024×512 all-sky texture
> (`Ω ≈ 3.05e-5`), and it does **not** present as a black preview. `ComputeEvAuto` clamps to
> `[-6, 6]`, so the error arrives as a plausible picture a few stops dark with the clamp
> silently absorbing the rest. This was observed, not imagined: it cost the `export_parity`
> fixture 5–8 dB while every other test stayed green.

`LUMICE_ComputeEvAuto()` (C API; algorithm in `core/ev_anchor.hpp`) returns
`log2(target_linear / (p99_raw_y / snapshot_intensity))`, clamped to `[-6, 6]`, or
`0` if either input is non-positive. The `snapshot_intensity` in that numerator is not decoration
and must not be replaced by 1: `ComputeMonoExposure`'s `kRelative` branch divides by the same
value again (`intensity_scale = intensity_factor / snapshot_intensity`) and the two cancel
exactly, leaving `2^exposure_offset × target_linear / anchor`. Pinned by
`MonoExposureScale.ChainedWithComputeEvAutoTheSnapshotIntensityCancels`
(`test/unit-correctness/gui/test_mono_exposure_scale.cpp`) at three values of
`snapshot_intensity`, because one value cannot tell "cancels" from "agrees here".

**The composite path is the exception and does not converge.** When `raypath_color` is active the
payload carries `composite_p99_y` instead — a P99 over the participating class lanes of this same
buffer (§6.6) — and it needs no conversion, being already in the buffer's units.
`SyncFromPoller` is the single place that picks between the two.

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
                       / (ComputeAxisSolidAngle(this view) × anchor_l99_sky_)
    — anchored to the SCENE's sky radiance. anchor_l99_sky_ is the P99 radiance
      per steradian over the session's fixed full-sky equal-area anchor plane
      (core/anchor_buffer.hpp), measured once per snapshot by AnchorConsumer and
      pushed in by ServerImpl::DoSnapshot; no lens, fov, view pose, `visible`
      clip or output resolution can move it. ComputeAxisSolidAngle
      (core/lens_proj_build.hpp) converts that radiance into the units this
      renderer's buffer holds. The GUI reads the SAME two numbers and multiplies
      where this divides (§2.5), so the two land on one per-pixel expression
      rather than on two anchors argued to be equal — see §2.8 for what this
      replaced and what it moved.
      Being anchored to a statistic over the accumulation rather than to emitted
      energy, it still carries no energy term: the picture keeps its look as
      ray_num grows, and the config alone does NOT determine output brightness
      (ray_num co-determines it).

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

### §2.7 The Metering Rule: `visible` Does Not Reach the Meter

`visible` is a **display** clip and never a metering input. The visibility mask
(`BuildVisibleMask`, applied in `PostSnapshot` and `ApplyCompositeBackground`) decides what you
see, never what the meter measured. `ParticipatingExposureScale` is filtered by color-class
participation (§6.6) and by nothing else; it is not filtered by `visible` either.

Since the two-sided anchor (§2.8) this is **structural rather than a discipline the code has to
keep**. The `kRelative` anchor is measured by `AnchorConsumer` over a fixed full-sky plane that
has no `visible` field, no lens and no view pose to consult; there is no longer a place where
the mask *could* be applied to the meter. Previously the anchor was a P99 over the renderer's own
output buffer, so "hand it the whole buffer, not the visible subset" was a rule someone could
have got wrong.

What the rule guarded is unchanged: the GUI and the server must meter the same thing, and the GUI
simulates through a commit arm that pins `visible` to FULL regardless of the user's setting
(`gui/file_io.cpp`'s `kSimCommit` branch; the divergence is declared in
`test_scene_commit_chain.cpp`'s `kDivergingKeys`). The property is now pinned one level up, as the
stronger statement it became: `RenderConsumerMeteringRule.RelativeExposureIsIndependentOfThisFramesOwnPixels`
(`test/unit-correctness/server/test_render_consumer_visible_mask.cpp`) meters one ray batch under
`visible: upper` and `visible: full` **and** with a ray deleted, demands one scale for all three,
and restates the retired per-view rule inside itself as the control that says those equalities
are not vacuous.

#### The brightness migration the retired rule charged for

> **Historical record.** The migration below shipped with the change that made `visible` a pure
> display clip, and the numbers describe *that* step. The mechanism it charges for cannot recur:
> the anchor no longer sees `visible` at all, by construction. The current step's own migration
> is §2.8, and it supersedes this one wherever the two would both apply.


`visible` became a pure display clip for all four lens families: no lens type culls ray energy
any more (previously the single-lens branch of `ProjectExitToPixel` dropped rays outside the
selected hemisphere, while `rectangular` / dual-fisheye / `globe` never did). The excluded
hemisphere therefore reaches the buffer for the first time on single-lens renders — and, by the
rule above, it reaches the P99 sample with it. **Existing CLI renders that combine a single-lens
projection (`linear` or any single `fisheye_*`) with `visible: upper`/`lower` and the default
`ev_mode: relative` change brightness.** Nothing else does: the other three families already
metered the full sky, `visible: full` is unaffected by construction, `kAbsolute` anchors to
emitted energy and is structurally immune, and the GUI does not move a pixel (its core renders
are always `visible: full`).

The direction is **not** fixed — it depends on whether the newly admitted region is brighter or
dimmer than the old anchor:

- Excluded region is mostly dark sky → the added pixels pull the 99th percentile **down** → the
  divisor shrinks → the visible part gets **brighter**.
- Excluded region holds the bulk of the halo (a high sun with `visible: lower`, say) → the added
  pixels push the percentile **up** → the visible part gets **darker**. This is the larger
  effect of the two, and it is where the worst case sits.

Measured on `fisheye_equal_area` 180° with the camera on the horizon, 4×10⁵ rays, black
background, comparing the same config before and after the cull removal (paired within one
simulation, so no run-to-run noise enters the ratio):

| Sun altitude | `visible: upper` | `visible: lower` |
|---|---|---|
| 5° | −0.09 stop | −0.16 stop |
| 20° | −0.06 stop | −0.13 stop |
| 60° | **+0.15 stop** | **−1.68 stop** (3.2× darker) |

Negative = darker after. The 60°/`lower` row is the worst case in this corpus and the reason the
spread is asymmetric: at a high sun almost all the halo energy is above the horizon, i.e. inside
the region `lower` excludes; the old behavior kept that energy out of the denominator and so
metered the remaining sky as if it were the whole picture. These are measurements, not a law —
unlike §7.2's `landed_fraction`, the shift here has no closed form a caller can evaluate, because
it depends on the energy distribution across the clip boundary. Do not extrapolate past the rows
above; re-measure for a scene that matters.

Restoring a previous look, if a specific render needs it:

- `intensity_factor` (`2^EV`, §2.6) applied per render entry cancels it exactly — the migration is
  a single global scalar per frame, not a re-weighting of the picture.
- `ev_mode: "absolute"` is unaffected outright: it anchors to `emitted_energy` (§7), which no
  hemisphere clip touches.

No compensating scale is applied anywhere in the pipeline. Compensating would mean making the
meter a function of `visible` again by a different route, which is the thing this section rules
out.

### §2.8 The Two-Sided Anchor: One Number for the CLI and the GUI

#### What it replaced

`ev_mode: relative` used to anchor each side to **its own** P99, over two different pixel
populations:

- the CLI, to a P99 over the renderer's own output buffer (`ExposureScale`);
- the GUI, to a P99 over the whole simulation texture (`server_poller.cpp`).

Both were "the frame anchoring to itself", and the two were argued to be equivalent. They are
not, and the failure is not subtle once named: **a P99 over an output buffer is a property of the
sky times the lens**, because the buffer holds `L · Ω_p` and `Ω_p` is the projection's own
vignetting. Point two different lenses at one sky and they disagree about how bright that sky is
— measured at up to **2.5 stops** across ordinary configs (table below), and the CLI/GUI pair
carried the residue as a constant gain neither side could see alone.

#### What replaced it

One measurement, two consumers, and a unit conversion at each end:

```
core     L99_sky  = P99( fixed full-sky equal-area anchor plane ) / Ω_anchor      [radiance, per sr]
                    — AnchorConsumer + core/anchor_buffer.hpp; no lens, fov, view,
                      `visible` or output resolution can move it.

CLI      scale    = intensity_factor · target_linear / ( Ω_axis(view) · L99_sky )
                    — its buffer holds L·Ω_p, so it DIVIDES by the on-axis solid angle.

GUI      ev_auto  = ComputeEvAuto( Ω_axis(sim texture) · L99_sky, snapshot_intensity, 135 )
                    — its texture holds L·Ω_texel, so it MULTIPLIES by the same kind of factor.
```

Both reduce to the same per-pixel expression, which is what makes this an elimination of the
divergence rather than a replacement of two anchors by two nearly-equal ones:

```
displayed(pos) = L(pos) · m(pos) / L99_sky ,      m = Ω_p(pos) / Ω_axis
```

`m` is the target lens's **relative illumination**, supplied per pixel by the preview shader
(`src/gui/preview_jacobian.hpp`) and baked into the CLI's buffer by the projection itself. The
two implementations therefore look different — only the CLI's expression names `Ω_axis` — and
that asymmetry is the design, not an omission.

`Ω_axis` is published per renderer as `LUMICE_RawXyzResult::axis_solid_angle` (§2.4), which is
what lets *any* C API consumer reproduce the relative scale, not only the GUI. Before it, the
header promised reproducibility for the absolute branch (`emitted_energy`) and could not deliver
it for the default one.

#### The migration law

```
new_scale / old_scale  =  p99_view / ( Ω_axis(view) · L99_sky )
```

Both factors on the right are obtainable by a caller: `Ω_axis(view)` and `L99_sky` are published
fields, and `p99_view` is the P99 the caller's own previous pipeline computed. Unlike §7.2's
`landed_fraction` this is **not** a quantity core can hand you directly — it depends on the
retired anchor, which nothing computes any more.

**Every `ev_mode: relative` render moves**, not only the ones some previous migration singled
out. There is no lens, fov or resolution for which the two anchors coincide except by accident.

Measured across 13 lens/fov/resolution combinations on two scenes (a 22° halo and a parhelion
display), paired within one simulation so no run-to-run noise enters the ratio:

| view | halo_22 | parhelion |
|---|---|---|
| `dual_fisheye_equal_area` 180° 1024×512 | −0.07 | −0.24 |
| `fisheye_equal_area` 180° 512² (zenith) | +0.20 | −0.24 |
| `fisheye_equal_area` 120° 512² | +0.67 | **+1.53** |
| `fisheye_equidistant` 180° 512² | −0.07 | −0.49 |
| `fisheye_stereographic` 180° 512² | −0.89 | **−1.02** |
| `rectangular` 360° 1024×512 | −0.38 | −0.83 |
| `linear` 120° 512×683 | −0.73 | −0.89 |
| `linear` 60° 512² | +0.60 | **+2.55** |
| `linear` 20° 512² | **−2.02** | +0.26 |
| `linear` 20° 1024² | −1.45 | −0.27 |

Stops; negative = darker after. Range **−2.02 … +2.55 stop (24×)**.

> ⚠️ The sign is **not** a function of how narrow the frame is, and "a narrow lens no longer
> auto-brightens" is the wrong summary. `linear 20°` moves −2.02 stop on one scene and +0.26 on
> the other; `linear 60°` moves **+2.55** on the scene where 20° moved up only slightly. What
> the shift actually measures is `L99(what this frame is looking at) / L99(the whole sky)`, so it
> depends on whether the framing happens to hold the bright features. Do not extrapolate from
> this table to a scene that matters — re-measure it.

#### The two behaviour changes this buys, and why they are wanted

1. **Changing the view no longer changes the brightness.** Two renders of one document at
   different fovs, lenses or camera angles are now directly comparable: the same sky direction
   is displayed at the same brightness in both. Previously a narrow frame metered only what it
   held and re-normalized to fill the tonal range, so cropping in was indistinguishable from
   the scene getting brighter.
2. **Changing the output resolution no longer changes the brightness.** `linear 20°` measured
   0.52–0.57 stop between 512² and 1024² under the retired anchor. The anchor is now a radiance,
   independent of any resolution — the renderer's and the anchor plane's alike. In the GUI this
   is the more visible half: moving the `sim_resolution` slider used to re-expose the preview,
   because the texture that slider sizes *was* the anchor.

Both are the same statement — the exposure describes the scene, not the act of looking at it,
which is what a camera does — and both are the reason the shifts in the table are accepted
rather than compensated for.

#### The cost this pays off

`§2.6` used to record, as an accepted cost, that under `relative` **the config alone does not
determine output brightness**. Half of that is now paid off and half is not, and the split is
worth stating precisely because it is easy to over-claim:

- **Paid off:** the *view* no longer co-determines brightness. Lens, fov, camera pose, `visible`
  and output resolution are all out of the anchor, structurally.
- **Still true, and deliberately so:** `ray_num` co-determines it. The anchor is a P99 over an
  accumulating Monte-Carlo estimate, so it converges as the run proceeds; that self-anchoring is
  exactly what keeps a picture's look stable as `ray_num` grows (§7.4), and giving it up means
  `ev_mode: absolute`, which exists for callers who want it.

So: "the same config renders at the same brightness however you look at it" is now true; "the
config file fully determines the brightness" is still false under `relative`, and is what
`absolute` is for.

#### What is NOT covered

`ParticipatingExposureScale` — the composite (raypath-colour) path's anchor (§6.6) — was audited
and deliberately left alone. It anchors the *participating class lanes* of this view, which is a
different physical question ("which rays are being shown"), and it has never had the divergence
this section removes: it is computed once, server-side, and the GUI reads the same
`composite_p99_y` field the CLI's own bake used. One consequence is worth knowing, because it is
new: mono `relative` is now view-independent while composite `relative` is not, so toggling
raypath colour changes how the brightness responds to a fov change.

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

Do not read point 1 as "per-pixel `Ω_p` has been ruled out everywhere". It rules out
*dividing core's pixels by it*. The GUI preview had the opposite defect — it carried no
`Ω_p` at all — and now multiplies one in, which reaches the same camera semantics from
the other side. See §7.5.

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

### §7.5 The GUI Preview's Relative Illumination — the Same Ω_p, Multiplied Rather Than Divided

§7.3 rules out dividing core's pixels by their own `Ω_p`. This section is about the GUI preview
multiplying by it, which sounds like the same subject and is its exact opposite, so the relationship
is stated here rather than left to be re-derived.

**The two sides were producing different quantities.** The CLI bins each ray into the pixel it lands
on, so a pixel accumulates the energy arriving over its own solid angle: `∫_{Ω_p} L dω ≈ L · Ω_p`.
That is camera semantics, it is what a camera pixel actually collects, and §7.3 keeps it that way.
The GUI preview does not image a scene at all — it resamples the dual equal-area all-sky texture
core produced, where every texel subtends the same solid angle, so a screen pixel carried `L` with
no `Ω_p` in it. For an equal-area target lens `Ω_p` is spatially constant and the two agreed up to a
constant nobody could see; for anything else they did not. Measured on a rectilinear lens at fov
160 on a 512×683 canvas: a GUI/CLI block-brightness ratio of 0.79 at the frame centre against 1.35
at the edges — the `cos³θ` natural vignetting of that projection, present on one side only.

**So the fix is on the GUI side, and it is a multiplication.** `src/gui/preview_renderer.cpp`'s
fragment shader now multiplies each sampled texel by the target lens's
**relative illumination** before the exposure scale reaches it; `src/gui/preview_jacobian.hpp` is
the CPU mirror, carrying the closed form and the derivation for every branch, and
`test/unit-correctness/gui/test_preview_jacobian.cpp` holds those closed forms to a numerically
rebuilt Jacobian rather than to the GLSL twin.

**This does not reopen §7.3, and the direction is why.** §7.3 refuses to *divide* core's pixels by
`Ω_p`, because that would strip the camera semantics out of the CLI's output and leave a radiance
map. This change *multiplies* the GUI's pixels by it, so that the GUI arrives at the camera
semantics the CLI already has. Both statements say the same thing — an image of this scene should
carry `Ω_p` — and neither weakens the `landed_fraction` boundary §7.2 and §7.3 draw. A GUI preview
and a CLI render remain comparable only within one document and one run; nothing here promises
comparability across lenses or across documents.

**What is multiplied is normalized on axis, and that is the load-bearing choice.** The factor is

```
m(pos) = Ω_p(pos) / Ω_p(on axis)
```

— the projection's relative illumination, dimensionless and 1 at the frame centre — and NOT the
absolute ratio `Ω_p / Ω_texel_source`.

The reason used to be an argument about two self-anchors, and since §2.8 it is an identity. Both
sides now divide by the same published sky radiance `L99_sky`, and each converts it into its own
buffer's units with that buffer's own on-axis solid angle — the CLI divides by `Ω_axis(view)`, the
GUI multiplies by `Ω_axis(sim texture)`. Writing both chains out:

```
CLI displays   L · Ω_p / ( Ω_axis · L99_sky )   =   L · m(pos) / L99_sky
GUI displays   L · m(pos) / L99_sky
```

so a factor normalized **on axis** is exactly what makes the two the same picture; any other
normalization point would leave a lens-dependent gain between them. Multiplying by the absolute
ratio would add a global gain whose physical content is the preview canvas's angular resolution,
which is a display choice. Measured: that gain is 0.331 for an equal-area fisheye at fov 96 on
512×683 (the GUI would go three times too dark) and 16 for a rectilinear lens at fov 160 on the
same canvas (sixteen times too bright, i.e. clipped). It would have turned two green parity scenes
red to fix a third.

**What this means for `kAbsolute`.** `kAbsolute` does not re-derive its anchor per frame, so
whatever the shader multiplies in lands directly in absolute brightness. Normalizing on axis decides
the answer: **the centre of the frame is unchanged at every FOV and for every lens, and what appears
is the vignetting shape alone.** Narrowing the FOV therefore does not darken the picture.

That is deliberate, and it is the second place §7.3's boundary does the work. One might expect the
opposite — a narrower field collects less light, so absolute brightness should fall — but the GUI's
`kAbsolute` denominator normalizes by the **source texture's** pixel count
(`g_preview.GetTextureWidth() * GetTextureHeight()`, `src/gui/app.cpp`) while the CLI's normalizes
by its **own canvas's** (`total_pix`, §2.3). Absolute brightness is therefore already not a per-lens
physical quantity on the GUI side, which is precisely the cross-lens boundary §7.3 declares
permanent. An absolute `Ω_p` would not have repaired that; it would have replaced one lens-dependent
gain with another and broken relative mode — the default, and the mode every committed reference
image is captured in — on the way. `test/unit-correctness/gui/test_preview_jacobian.cpp` asserts
both halves of this: unity on axis for every lens and FOV, and a corner that darkens monotonically
as the FOV widens.

**Every texture the preview shows gets the factor, and one obsolete file format does not.** This
paragraph used to record the opposite as a permanent boundary: the factor was applied only in XYZ
mode, so a document loaded from a `.lmc` — an 8-bit texture that
`LUMICE_XyzToSrgbUint8WithBackground` had already composited the sky colour into — rendered a
non-equal-area projection without its vignetting. That was a real mechanism and the wrong endpoint.
Vignetting is a property of the **target lens at display time**, not of the stored picture, so the
frontend owes it to a reopened document exactly as to a live one; what actually blocked that was the
sky being summed into the stored texels, and the fix is to stop summing it, not to try to subtract
it back out (subtraction is not invertible where the bake clipped).

So the sky colour is stored where it belongs — as a **setting**, in the document's JSON — and the
bake carries the halo's radiance alone. `PreviewRenderer::TextureMode` names the three source
semantics: `kXyz` (live float radiance), `kSrgbRadiance` (8-bit radiance-only: a `.lmc` from
format v4 on, and the raypath-colour composite the server bakes), and `kSrgbComposited` (the
1×1 blank, and a pre-v4 `.lmc` whose sky is summed in and can only be drawn as it is). The first two
take the same tail — relative illumination, then the sky, then the transfer curve — which is what
makes save-then-reopen render the picture that was on screen; `test/gui/visual/test_preview_pixels.cpp`'s
`save_open_visual_consistency` is the pixel measurement of that claim, and it moved from 19.9 dB to
38.6 dB when the two paths were unified. Equal-area projections were never affected either way:
their factor is exactly 1.

Two consequences worth naming. The raypath-colour composite is covered by the same change and had
the same defect on a **live** path — toggling the composite preview on a non-equal-area lens used to
make the vignetting appear and disappear with it — so the GUI no longer pushes a background into the
server-side composite bake (`LUMICE_SetCompositeBackground` remains for non-GUI consumers; its
all-zero default is an algebraic no-op). And a pre-v4 `.lmc` still shows no vignetting, which is not
a residual defect but the only honest reading of pixels whose sky cannot be separated back out.

The 8-bit branch's own contract — that a composited texture is drawn exactly as it is, while the XYZ
branch beside it applies the factor — stays pinned by
`test/gui/functional/test_preview_background.cpp`'s
`the_baked_document_lacks_the_projection_vignetting`, which is what keeps `kSrgbComposited` from
quietly drifting into re-lighting pixels it cannot re-light.


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
