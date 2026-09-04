[中文版](coordinate-convention_zh.md)

# Coordinate System and Rotation Convention

> **Breaking change (v3).** The rotation chain was reworked in the v3 release.
> Configurations carrying `crystal.axis.*` from earlier releases will render with
> different orientations and may need to be re-authored against the conventions
> below — including the `azimuth` sign convention and the `−180°` chain offset.
> `filter.raypath` and `light.*` / `view.*` are unaffected.

This document defines the coordinate systems, axis conventions, and rotation
chain that Lumice uses for crystal orientation, light source placement, and
camera viewing. All numeric examples use degrees unless explicitly noted as
radians.

## 1. Crystal Local Coordinate System

The crystal local frame is fixed to the hexagonal mesh and rotates with the
crystal. The convention is right-handed with:

- `N1` (face 1, top basal) outward normal → local `+z` (the c-axis)
- `N3` (face 3) outward normal → local `+x`
- `N1 × N3` → local `+y` (right-hand rule)

Other faces follow from hexagonal symmetry; see `src/core/crystal.cpp::FillHexFnMap`
for the full normal table.

`Nx` denotes the outward unit normal of face `x` (pointing from crystal interior
to exterior). The c-axis is identified with N1 throughout this document.

## 2. World Coordinate System

The world frame is fixed and right-handed:

- `+z` points to the zenith
- The `xy` plane is the local horizontal (the ground plane in halo simulations)
- `+x` is the reference azimuth direction (see §3)

The world frame is the reference for both crystal orientation sampling and
camera viewing.

## 3. Azimuth Convention

Lumice uses the **mathematical convention** for azimuth angles:

- Azimuth `az = 0°` corresponds to world `+x`
- Increasing azimuth rotates **counter-clockwise** when viewed from `+z`
  looking down toward the ground
- This is the opposite of the geographic convention (north = 0°, increasing
  toward east)

The same convention applies to all azimuth-typed fields: `crystal.axis.azimuth`,
`scene.light_source.azimuth`, and `render[].view.azimuth`.

## 4. Light Source Position

The sun direction is parameterized by `(altitude, azimuth)`:

- `altitude` (also called solar elevation) ∈ [0°, 90°]: 0° on the horizon,
  90° at the zenith
- `azimuth` follows the §3 convention; `azimuth = 0°` places the sun in the
  `+x` direction
- `diameter` controls the angular spread of the sun disc

`SampleRayDir` (in `src/core/simulator.cpp`) emits incoming photon directions
**toward** the observer (i.e. opposite to the sun position vector); this is a
sampling-side convention transparent to user-facing config.

## 5. Typical Crystal Poses (in world coordinates)

The following poses are characteristic of the four built-in axis presets.
Each description fixes the **mean** orientation; the actual sampled orientation
includes Gaussian / uniform perturbations as configured (see §7).

### 5.1 Plate

- `N1` points to world `+z` (c-axis vertical)
- The crystal rotates freely around `N1` (around world `+z`)

### 5.2 Column

- `N1` lies in the `xy` plane (c-axis horizontal)
- `N1` rotates around world `+z` (sampled by `azimuth`)
- The crystal rotates freely around its own `N1` (sampled by `roll`)

### 5.3 Parry

- `N3` points to world `+z`
- The crystal rotates freely around `N3` (around world `+z`, sampled by
  `azimuth`); `roll` is locked near 0° to keep `N3` stable upward

### 5.4 Lowitz

- `N3 × N1` lies in the `xy` plane
- `N3 × N1` rotates around world `+z` (sampled by `azimuth`)
- `zenith` carries a large Gaussian perturbation (σ ≈ 40° by default), so the
  c-axis swings widely around the zenith — this large σ is what gives Lowitz
  its visual signature, not a separate chain term

## 6. Rotation Chain

Lumice uses a single rotation chain, applied uniformly to all preset types
and all custom configurations. Given an orientation sample `(azimuth, zenith,
roll)` (in degrees), the local-to-world rotation is:

```
R(azimuth, zenith, roll) = Rz(azimuth − 180°) · Ry(−zenith) · Rz(roll)
```

The chain is applied **inner to outer** to a local-frame vector:

1. `Rz(roll)` around the local c-axis (which initially coincides with world `+z`)
2. `Ry(−zenith)` around world `−y`
3. `Rz(azimuth − 180°)` around world `+z`

`Rn(θ)` is the standard right-handed rotation matrix about axis `n` by angle `θ`.

The implementation is `lumice::BuildCrystalRotation(azimuth_rad, latitude_rad,
roll_rad)` (in `src/core/simulator.hpp`), where `latitude_rad = π/2 − zenith_rad`
to match the spherical sampling convention used internally.

## 7. Preset Default Sampling Parameters

| Preset | zenith              | azimuth                | roll                    |
|--------|---------------------|------------------------|-------------------------|
| Plate  | Gauss(μ=0°, σ)      | Uniform [0°, 360°)     | Uniform [0°, 360°)      |
| Column | Gauss(μ=90°, σ)     | Uniform [0°, 360°)     | Uniform [0°, 360°)      |
| Parry  | Gauss(μ=90°, σ)     | Uniform [0°, 360°)     | Gauss(μ=0°, σ) locked   |
| Lowitz | Gauss(μ=0°, σ_L)    | Uniform [0°, 360°)     | Gauss(μ=0°, σ) locked   |

"locked" means the GUI fixes the distribution type to Gaussian and only the σ
is user-adjustable. The default σ values follow `kAxisPresets` in
`src/gui/edit_modals.cpp` (currently σ = 1° for Plate / Column / Parry, σ_L =
40° for Lowitz).

The visual differences between presets at runtime come primarily from the
**distribution shape** (Uniform vs locked Gaussian, or large σ vs small σ),
not from differences in mean parameters; Column and Parry, in particular,
share the same `(μ_az, μ_zenith, μ_roll)` triple at default and differ only
in how `roll` is sampled.

## 8. The −180° Azimuth Offset

The offset of `−180°` on the azimuth term is required by the local-frame
choice `N3 = +x`. Without the offset, the Parry default
`(zenith = 90°, azimuth = 0°, roll = 0°)` would map `N3` to world `−x`
instead of `+z`, which would contradict §5.3.

Numerical verification at the Parry default:

```
R = Rz(−180°) · Ry(−90°) · Rz(0°)

N1_world = R · (0, 0, 1)
         = Rz(−180°) · Ry(−90°) · (0, 0, 1)
         = Rz(−180°) · (−1, 0, 0)
         = (+1, 0, 0)             → +x

N3_world = R · (1, 0, 0)
         = Rz(−180°) · Ry(−90°) · (1, 0, 0)
         = Rz(−180°) · (0, 0, 1)
         = (0, 0, +1)             → +z   ✓
```

Intuitively, the `−180°` offset aligns "azimuth = 0° in config" with "`N1` on
the `+x` side at the Parry pose", which is the natural reading for users
configuring crystals against a `+x = sun direction` reference.

## 9. Camera (View) Convention

The camera frame is independent of the crystal frame and is parameterized by
`(elevation, azimuth, roll)` under `render[].view`:

### 9.1 Forward Direction

The camera forward direction in world coordinates is:

```
forward = (cos(elevation) · cos(azimuth),
           cos(elevation) · sin(azimuth),
           sin(elevation))
```

Equivalently:

- `elevation = 0°, azimuth = 0°` → forward = `+x`
- `elevation = 90°` → forward = `+z` (looking up)
- `elevation = 0°, azimuth = 90°` → forward = `+y`

The implementation is `lumice::BuildViewMatrix` in
`src/gui/preview_renderer.cpp` and the equivalent core path.

### 9.2 Azimuth and Elevation Sign Conventions

`view.azimuth` follows the same mathematical convention as in §3
(counter-clockwise from `+x` when viewed from `+z`).

`view.elevation`: positive values look upward. `elevation = -10°` looks
slightly below the horizon.

### 9.3 Roll

`view.roll` rotates the camera around its forward axis. Positive `roll`
rotates the image counter-clockwise as seen by the viewer (i.e. the camera's
local `+x` axis rotates toward its local `+y`).

### 9.4 Relation to Light Source

The view azimuth and the light source azimuth use the **same** azimuth
convention but are independent parameters. With `light_source.azimuth = 0°`
(sun in the `+x` direction) and `view.azimuth = 0°`, the camera looks **toward**
the sun.

A common convention for halo screenshots is to set `view.azimuth = 180°` so
the camera looks **away** from the sun, with the sun behind the observer; this
is independent of the chain conventions in §6.

### 9.5 Summary of Field Semantics (No Behavior Change)

The `view.*` fields' values and semantics are not affected by the chain rework
in §6. Existing configs and `.lmc` files that set `view.*` continue to render
the same camera framing.

### 9.6 Modal Crystal Preview Camera (GUI-internal)

The modal-tab crystal preview and the entry-card thumbnail share an *internal*
camera convention that is separate from the simulator camera in §9.1–9.5.

- **Camera position (logical):** at world `(0, -dist, dist·tan(15°))` looking
  at the origin, with world `+z` as up. The 15° downward elevation is fixed
  via `kCameraTiltDeg` in `src/gui/gui_constants.hpp`.
- **Implementation:** because the GUI mesh undergoes a Y-Z swap during build
  (`crystal_preview.cpp::BuildCrystalMeshData`: core `+z` → mesh `+y`,
  core `+y` → mesh `-z`), the swap *implicitly* establishes the
  "camera at world `-y`, world `+z` up" view. `CrystalRenderer::ComputeMvp`
  appends a small `Rx(+kCameraTiltDeg)` rotation for the elevation. The
  positive sign tilts world `+z` (mesh `+y`) toward the camera so the
  crystal's top face is visible (looking down at the crystal from above).
- **Mouse-drag semantics:** rotation is composed in world coordinates by
  left-multiplying `g_crystal_rotation` with a Rodrigues axis. Camera position
  stays put; the crystal turns relative to the user's world frame.
  - Drag right (`dx>0`) → rotate around mesh `+y` (= world `+z`, vertical
    spin axis).
  - Drag down (`dy>0`) → rotate around mesh `+x` (= world `+x`, camera-right
    tilt; crystal top tips toward viewer).
- **Default orientations:** `axis_presets.hpp::DefaultPreviewRotation` is the
  single source for both modal Reset View / preset-button and the entry-card
  thumbnail. The function takes `(preset, params, out)` where `params` mirrors
  `g_axis_buf` (`[0]=zenith, [1]=azimuth, [2]=roll`):
  - `kColumn` / `kPlate` / `kParry` / `kLowitz` use the
    `kPresetTypicalChain` table (fixed typical chain inputs)
  - `kCustom` with non-null `params` uses `ChainRotationToMatrix` driven by
    the user's live distribution mean values
  - `kRandom` and `kCustom` with null params fall back to the isometric
    sentinel `Ry(25°)·Rx(35°)`
  All chain-derived outputs use `ChainRotationToMatrix` which mirrors
  `BuildCrystalRotation` from `src/core/simulator.cpp` (`Rz(az−π) · Ry(−zenith) ·
  Rz(roll)`); equivalence is guarded by a contract test in `test_axis_presets.cpp`.

These conventions are *internal* to the GUI preview pipeline; they do not
affect serialization, simulation, or the export camera (§9.1–9.5).

### 9.7 Equirectangular (`rectangular`) Lens: Core Follows the Pose, the GUI Does Not

Owner decision, 2026-09-02. The two sides answer different questions about this
lens and are *both* right; this section exists so the next reader does not have
to re-derive that, and does not "fix" one of them into the other.

**Core.** The equirectangular map follows the **full** camera pose — azimuth
**and** elevation **and** roll — through the very same rotation every other lens
consumes. `BuildProjParams` fills `ProjParams::rot` with
`MakeCameraRotation(cfg)` for every lens type, and the `kProjRectangular` branch
of `ProjectExitToPixel` starts from the same camera-frame vector the single-lens
family uses:

```
c = R^T · (−w)              // ApplyRotTranspose, identical to linear/fisheye/globe
lon = atan2(−c.x, +c.z)     // RectangularForward(c.z, −c.x, −c.y)
lat = asin(−c.y)
```

The axis assignment is the part worth stating, because several permutations
would satisfy "follows the pose" and only this one keeps the degeneracy below:
`+c.z` is the optical axis, so the boresight lands at the centre of the map;
`−c.y` is the polar axis, because the camera's local `+y` points at the world
nadir under the `Rz(−90°+roll)·Ry(90°−el)` half of the chain; `−c.x` is the
remaining quadrature axis, and its sign is what makes the degeneracy exact.

Until 2026-09-02 core reduced the camera rotation to one scalar,
`az0 = atan2((R·ẑ)_y, (R·ẑ)_x)`, and subtracted it from the longitude. Since
`R·ẑ = (cos el·cos az, cos el·sin az, sin el)`, that scalar is identically `az`
— so a tilted or rolled camera produced a **bit-identical frame**, which is what
the change fixes. The `az0` field is gone from `ProjParams`; nothing reads it.

**Degeneracy at `el = roll = 0` (why nothing that shipped moved).** Substituting
`el = 0, roll = 0` into `R = Rz(az)·Ry(90°−el)·Rz(−90°+roll)` gives

```
R = [[ sin az, 0, cos az],
     [−cos az, 0, sin az],
     [      0, −1,     0]]
```

so with `g = −w`:

```
c.x =  sin az·g_x − cos az·g_y
c.y = −g_z
c.z =  cos az·g_x + sin az·g_y
```

Writing `g_x = ρ·cos φ`, `g_y = ρ·sin φ`:

```
lon = atan2(−c.x, c.z) = atan2(ρ·sin(φ−az), ρ·cos(φ−az)) = φ − az
lat = asin(−c.y)       = asin(g_z)
```

which is *exactly* the old form (`RectangularForward(−w)` giving
`lon₀ = atan2(g_y, g_x) = φ` and `lat₀ = asin(g_z)`, then `lon = lon₀ − az0`
with `az0 = az`). The pointwise equality is asserted by
`LmProj.RectangularAtZeroElevationAndRollReproducesTheAzimuthOnlyForm`
(`test/golden-analytic/core/test_projection.cpp`), which carries the old formula
verbatim so the identity survives the field's deletion.

The **one** exception is the two poles, `g = ±ẑ`: there both `atan2` arguments
vanish on both sides and the longitude is arbitrary by the nature of the map.
The old form put the pole at column `−az`, the new one at the boresight column.
The row — the half that carries meaning — is unchanged.

Two consequences follow from the degeneracy rather than from any extra work:
every shipped `rectangular` config renders identically (they all sit at
`el = roll = 0`), and the GUI↔CLI export comparison needs no new argument, since
the GUI only ever exports this lens at a zero pose (below).

**GUI.** The preview keeps a **fixed all-sky texture** and applies no view
transform to it at all: `LensIsFullSky` puts `rectangular` in the same class as
the four dual-fisheye types, `preview_renderer.cpp` sets
`needs_view_transform = false` for them, and `RenderPreviewPanel` pins
`elevation / azimuth / roll` to zero for that class on **every frame**. This is
the division of labour, not an omission — on the GUI side every camera-pose
transform is the front end's job, applied at display time by resampling the
texture, so pushing the pose into the projection would apply it twice.

Because that per-frame rule is the single enforcement point (the `.lmc` loader,
the CLI-JSON importer and the personal-defaults overlay are all plain
transcripts, and all four `renderer.*` keys are default-eligible), it is pinned
from both directions:
`view_display_controls/an_arriving_document_cannot_smuggle_a_full_sky_pose`
covers the three arrival paths and the exported config they produce, and
`view_display_controls/which_view_sliders_apply_depends_on_the_lens` covers the
widget path.

The two sides are compared against each other in
`VisibleMaskGuiParity.RectangularFollowsTheFullCameraPoseAndTheGuiIsAFixedTexture`
and `AnnotationOverlayGuiParity.RectangularFollowsTheCameraPoseAndTheGuiDeliberatelyDoesNot`:
they agree exactly at a zero pose, differ by a pure horizontal shift under an
azimuth, and differ with a vertical component under elevation or roll.

## 10. Persistence Compatibility

### 10.1 Fields with Changed Semantics (Breaking)

- `crystal.axis.{zenith, azimuth, roll}`: The chain change in §6 means the
  rendered orientation for the same numeric input changes. Old `.lmc` /
  `config.json` files will produce visually different results.

### 10.2 Fields with Preserved Semantics

- `filter.raypath`: Face indices remain physically anchored. The `ref_norms[]`
  table in `src/core/crystal.cpp` is unchanged, so `raypath = [3, 5]` still
  selects the same physical face pair.
- `light.*`, `view.*`, `crystal.shape.*`, `filter.symmetry`: Unchanged.

### 10.3 Migration Strategy

There is no automatic migration script. Users with stored configurations
should re-author the `crystal.axis.*` fields against the new chain. The
typical pose descriptions in §5 are the canonical reference for what each
preset should look like.

For deterministic axis values (e.g. `axis.zenith = 90`), the migration is
usually a single sign or 180° swap, easily reproduced by visual inspection
of the rendered output. For probabilistic distributions (Gaussian / Uniform),
the distribution shape is unchanged — only the rendered orientation is
affected.

#### Quick orientation deltas (deterministic axes)

The following table compares N1 / N3 world directions for the same
`(azimuth, zenith, roll)` numeric input under the old and new chains. Use it
as a sanity check when re-authoring fixed orientations.

| (azimuth°, zenith°, roll°) | Old N1 → world | New N1 → world | Old N3 → world | New N3 → world |
|----------------------------|----------------|----------------|----------------|----------------|
| (0, 0, 0)                  | +z             | +z             | +x             | −x             |
| (0, 90, 0)                 | +x             | +x             | −z             | +z             |
| (180, 90, 0)               | −x             | −x             | −z             | +z             |
| (90, 0, 0)                 | +z             | +z             | +y             | −y             |

Notable patterns:
- For `zenith = 0` (Plate-like) and `roll = 0`, the N3 direction flips along
  `±x`; if the old config relied on N3 pointing to `+x`, change `azimuth` by
  180° in the new chain
- For `zenith = 90` (Column / Parry), N3 flips between `±z`; configurations
  that wanted "N3 facing the zenith" (Parry semantics) get this for free
  under the new chain at `azimuth = 0°`, whereas the old chain required
  ad-hoc adjustments
- Configurations using full `azimuth = Uniform[0°, 360°)` sampling are
  statistically invariant; no migration is needed for those

## 11. Verification

Implementation correctness is gated by three independent layers:

1. **Mathematical**: Unit tests in `test/test_simulator.cpp`
   (`BuildCrystalRotation.CaseA_AzOffsetOnly` ... `CaseD_RollAroundCAxis`)
   assert the chain output for four distinguishable inputs that probe each
   chain term individually.
2. **Structural**: GUI thumbnails (`src/gui/thumbnail_cache.cpp`) display
   the canonical preset poses described in §5.
3. **Physical**: E2E reference images (`test/e2e/references/*.jpg`) and GUI
   reference images (`test/gui/references/*.png`) verify that rendered halo
   patterns are stable across chain changes; configurations with full-azimuth
   uniform sampling are statistically invariant under the chain rework, so
   their reference images remain valid without regeneration.

If any layer fails after a chain modification, investigate in this priority:

- Sign of the `azimuth − 180°` offset (§8)
- Sign convention of azimuth (§3)
- Sign of `Ry(−zenith)` in the middle term (§6)

## Appendix: Chain Output Quick Reference

The following four cases are mathematically distinguishable, probe each chain
term in isolation, and are exactly what `test/test_simulator.cpp` asserts.

| Case | (az°, zenith°, roll°) | N1_world (= R · ê_z) | N3_world (= R · ê_x) | Probes               |
|------|------------------------|----------------------|----------------------|----------------------|
| A    | (0, 0, 0)              | +z                   | −x                   | az − 180° offset     |
| B    | (0, 90, 0)             | +x                   | +z                   | Ry(−zenith) sign     |
| C    | (90, 90, 0)            | +y                   | +z                   | Rz(az − 180°) for az ≠ 0 |
| D    | (0, 0, 90)             | +z                   | −y                   | Rz(roll)             |

Tolerance: `Dot3(N_actual, N_expected) > 1 − 1e-5`.

Preset correspondences:

- Plate at default: zenith = 0° → reduces to Case A (modulo random roll/az)
- Parry at default: zenith = 90°, roll ≈ 0° → matches Case B (modulo random az)
- Column at default: zenith = 90°, roll uniform → same chain output as Parry
  at the means; presets differ in roll **distribution**, not in chain output
- Lowitz at default: zenith = 0° (mean) with large σ → behaves like Plate
  at the mean, with c-axis swinging widely due to σ

## 10. Screen Handedness (Render Projection Convention)

Sections §1–§9 fix the **world** coordinates, the azimuth math convention, and
the camera forward direction — but they deliberately do **not** mandate which
side of the screen an increasing azimuth maps to. That screen left/right mapping
is a **render presentation** choice, made once and applied consistently across
every render path (CLI direct-output image, the GPU Metal/CUDA backends, and the
GUI preview). The core simulator (world directions + rotation chain) is
unaffected and continues to follow §3.

**Convention: `right = +az`.** For an inside-out lens facing the sun, a sky
feature at increasing azimuth appears **further to the RIGHT** on screen. This is
the user-habit-natural direction and is identical for the CLI image and the GUI
preview (they must agree — that is the whole point of standardizing here).

- **Single-lens family** (linear + the four single fisheyes: equal-area,
  equidistant, stereographic, orthographic): `+az → screen RIGHT`.
- **Globe** (option-B outside-in spherical view): intentionally **OPPOSITE**,
  `+az → screen LEFT`. Viewing the celestial sphere from *outside* mirrors it
  horizontally relative to the inside-out (naked-eye) view, so globe being the
  opposite side of the inside-out family is correct, not a bug.
- **Rectangular / equirectangular**: `+az → screen RIGHT` (the conventional
  equirectangular longitude direction); consistent with the single-lens family.

**Implementation.** The screen-x handedness lives in the single-lens branch of
`lm_proj::ProjectExitToPixel` (`src/core/shared/projection_shared.h`, the single
source shared by legacy CPU / Metal / CUDA), so all backends inherit the same
convention. The GUI's independent forward implementations
(`ProjectWorldDirToScreen` in `preview_renderer.cpp`, `WorldDirToPixel` in
`overlay_labels.cpp`) already produce the same `right = +az` result.

**Regression guard.** Because a handedness flip is invisible to forward/inverse
round-trip tests (they close by construction under any self-consistent
convention), this convention is pinned with **absolute screen-side** assertions:
`test/golden-analytic/core/test_projection.cpp` (backend absolute-column pins)
and `test/unit-correctness/gui/test_render_handedness_guard.cpp` (cross-implementation
check across backend + both GUI forwards + the interaction read-back). See
scrum-321 (azimuth-handedness-alignment) for the audit and decision record.
## 11. Pixel Centre Convention (Forward Binning ↔ Mask ↔ Shader)

§10 fixes which *side* of the screen a direction lands on. This section fixes the
sub-pixel question underneath it: given a continuous image coordinate, **which
pixel is that, and where inside the pixel is its centre?** The two halves used to
answer differently, by exactly half a pixel.

**Convention: pixel `px` covers the continuous span `[px - res/2, px + 1 - res/2)`,
and its centre is at `px + 0.5 - res/2`.** The image is symmetric about the frame
centre, and the addressable span is `[-res/2, res/2)`.

Both directions of the mapping follow from that one statement, and each is owned
by exactly one place:

- **Forward** (sky direction → pixel index): `px = floor(v + res/2)` — the binning
  at the end of every branch of `lm_proj::ProjectExitToPixel`
  (`src/core/shared/projection_shared.h`). `LM_FLOOR` / `LM_FN` make that header
  compile into all three backends, so legacy CPU, Metal and CUDA cannot disagree
  about it.
- **Inverse** (pixel index → sky direction): sample at `px + 0.5 - res/2` — the
  render-domain mask (`src/core/lens_proj_build.hpp`, `mask_detail::PixelToWorld`
  and the per-family helpers) and, independently, the GUI preview shader, whose
  `pos = v_ndc * u_resolution * 0.5` is the same symmetric convention expressed in
  NDC.

They compose exactly: substituting a pixel centre into the forward gives
`floor(px + 0.5) == px` for every integer `px`, so a round trip returns the pixel
it started from with no slack.

**What this replaced.** Forward binning used to be `floor(v + res/2 + 0.5)`, which
put its addressable span at `[-res/2 - 0.5, res/2 - 0.5)` — not symmetric about the
frame centre, and half a pixel away from what the mask and the shader assumed. The
visible consequences were all sub-pixel and none of them was wrong-looking on its
own, which is why it survived so long: a round trip slid by a pixel; a direction on
an image-circle rim could bin inside on one side and outside on the other; and the
extreme rim of a dual-fisheye disc was squeezed into slivers a few thousandths of a
pixel wide.

**Consuming this convention.** Core answers a pixel INDEX, not a continuous
coordinate. Anything that compares core's answer against a continuous position — a
GUI forward, an un-projection, an overlay anchor — must convert first, by taking
that pixel's centre (`index + 0.5` in image space). Comparing the bare index
against a continuous coordinate measures the truncation rather than the projection;
under the old `+ 0.5` binning that category error was hidden, because the index
happened to be a *round* of the continuous coordinate and the residual was a
symmetric half pixel.

**A THIRD pairing: the GUI's source-texture gather.** The two bullets above are
the two directions of one mapping over the *render* domain — the frame being
produced. There is a third place the same convention is load-bearing, and it is
easy to miss because it is neither a forward nor an inverse of the target lens:
the GUI preview re-projects an all-sky **source texture** that core produced by
this same binning, so the shader must also read that texture in this convention.
That read is `dualFisheyeToUV` in `src/gui/preview_renderer.cpp`, and the rule
there is `uv = pixel / tex_res` — the plain division, with **no** `+ 0.5`.

The reason is that GL's own texture addressing is already stated in these terms:
a sample at `uv * tex_res == px + 0.5` lands on the centre of texel `px`, which
is the same sentence as the convention above. So handing GL a coordinate already
expressed in core's binning convention is all that is required, and adding half a
texel first applies the centring **twice**. The failure that produces is not
"reads the wrong texel" — it moves every fragment onto a texel corner, so every
sample comes back a 2×2 bilinear average. Whole-frame softening is invisible
without something to compare against, but a dual-fisheye source has one: the two
discs map `y_norm` to `fx` with opposite signs, so one shared pixel offset becomes
two *opposite* sky directions, and the ±5° equator overlap band blends two
different patches of sky into a smear along the horizon. Commit `4d9e3643` wrote
that `+ 0.5` correctly against the old `floor(v + 0.5)` binning; commit `f1bf1e9e`
changed the binning and updated the render-domain pair above without this third
one, which is how the two sides came to disagree.

**Regression guard.** Exact-equality round trips over the forward and the mask's
own inverse, in `test/golden-analytic/core/test_visible_mask.cpp`
(`GlobeInverse.RoundTripsAgainstTheForwardGlobeBranch` and
`RectangularInverse.RoundTripsAgainstTheForwardRectangularBranch`). They demand
the recovered pixel back, with no tolerance, which is what makes any reappearance
of a half-pixel offset a red rather than a slack absorbed silently.

The third pairing has its own guard, in
`test/gui/functional/test_preview_dual_fisheye_gather.cpp`. It cannot be a
round trip in C++ — the gather lives in GLSL, and a mirrored copy of the formula
in a test would only assert against itself. Instead it renders the real shader
with the source format's `r_scale` set to 1, which makes the display-side inverse
and the gather-side forward the same projection at the same scale and collapses
the chain to an identity: canvas pixel `(col, row)` must read source texel
`(col, row)`. A one-texel checkerboard then measures the convention directly —
sharp means each fragment read a single texel, and a collapse to flat mid-grey
means it averaged a 2×2 block. Note its scope: it detects a **systematic offset**
shared by every pixel, which is what a convention mismatch is. A projection error
that relocated content without softening it (a rotation, a mirrored disc) keeps
the checkerboard sharp and passes here; those are covered by the `lens_proj`
reference images and `test_visible_mask_gui_parity.cpp`.
