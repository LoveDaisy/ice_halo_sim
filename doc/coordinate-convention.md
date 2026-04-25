[中文版](coordinate-convention_zh.md)

# Coordinate System and Rotation Convention

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
