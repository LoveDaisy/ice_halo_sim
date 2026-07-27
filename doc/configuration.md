[中文版](configuration_zh.md)

# Configuration Reference

This document provides a detailed description of the configuration file format, the meaning of each configuration field, default values, and validation rules.

## Configuration Overview

The configuration file uses JSON format and contains the following main sections:

- `crystal`: Array of crystal definitions
- `filter`: Array of filter definitions
- `scene`: Scene definition (single object, includes inline light source)
- `render`: Array of renderer definitions
**Important notes**:
- The example configuration file (`examples/config_example.json`) does not exhaustively cover all valid configuration patterns
- Many configuration parameters have default values that will be used when not explicitly specified
- **You should refer to the source code** for the complete configuration logic and default values
- This document extracts all default value information based on the code implementation

## Configuration Fields

### crystal (Crystal Configuration)

The crystal configuration defines the crystal shapes and orientation distributions used in the simulation.

> **Coordinate / rotation convention**: For the world frame, local frame, azimuth sign convention,
> and the rotation chain semantics behind `axis.{zenith, azimuth, roll}`, see
> [`coordinate-convention.md`](coordinate-convention.md). Configurations from earlier releases will
> render with different orientations under the current chain (deterministic break for any non
> full-sphere-uniform `axis` configuration).

#### Basic Structure

```json
{
  "id": <unique identifier>,
  "type": "prism" | "pyramid",
  "shape": { ... },
  "axis": { ... }
}
```

#### Field Descriptions

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `id` | integer | yes | - | Unique identifier, must be greater than 0 |
| `type` | string | yes | - | Crystal type: "prism" or "pyramid" |
| `shape` | object | yes | - | Shape parameters, see below |
| `axis` | object | no | see below | Crystal orientation distribution |

#### axis (Orientation Distribution) Defaults

If the `axis` field is absent, the following defaults are used:

```json
{
  "zenith": 90.0,    // horizontal orientation
  "azimuth": 0.0,
  "roll": 0.0
}
```

#### prism (Hexagonal Prism) Type

**shape structure**:

```json
{
  "height": <value or distribution>,
  "face_distance": [<6 values or distributions>],
  "sync_group": { "height": <int>, "face_distance": [<6 ints>] }
}
```

**Field descriptions**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `height` | value/distribution | no | 1.0 | Height ratio h/a, where h is the prism height and a is the base diameter |
| `face_distance` | array | no | [1,1,1,1,1,1] | Distance ratios for 6 faces; [1,1,1,1,1,1] for a regular hexagon |
| `sync_group` | object | no | all independent | Shape-scalar sync groups — see [Shape-Scalar Sync Groups](#shape-scalar-sync-groups) below |

`face_distance` values may be negative — a negative value is accepted and
participates in geometry construction with its sign (it is not silently
folded to a positive value). The resulting mesh is validated at construction:
a mesh that fails the closed-manifold check is rejected (the crystal is
dropped and contributes zero energy) rather than silently accepted.

**Example**:

```json
{
  "id": 1,
  "type": "prism",
  "shape": {
    "height": 1.3,
    "face_distance": [1, 1, 1, 1, 1, 1]
  }
}
```

#### pyramid (Hexagonal Pyramid) Type

**shape structure**:

```json
{
  "prism_h": <value or distribution>,
  "upper_h": <value or distribution>,
  "lower_h": <value or distribution>,
  "upper_indices": [<3 integers>],
  "lower_indices": [<3 integers>],
  "face_distance": [<6 values or distributions>],
  "sync_group": {
    "prism_h": <int>, "upper_h": <int>, "lower_h": <int>, "face_distance": [<6 ints>]
  }
}
```

**Field descriptions**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `prism_h` | value/distribution | yes | - | Prism segment height ratio |
| `upper_h` | value/distribution | no | 0.0 | Upper pyramid segment relative height (0.0-1.0) |
| `lower_h` | value/distribution | no | 0.0 | Lower pyramid segment relative height (0.0-1.0) |
| `upper_indices` | integer array | no | [1,0,1] | Miller indices for the upper pyramid segment |
| `lower_indices` | integer array | no | [1,0,1] | Miller indices for the lower pyramid segment |
| `face_distance` | array | no | [1,1,1,1,1,1] | Distance ratios for 6 faces |
| `sync_group` | object | no | all independent | Shape-scalar sync groups — see [Shape-Scalar Sync Groups](#shape-scalar-sync-groups) below |

`face_distance` values may be negative — same construction and rejection
semantics as the prism type above.

**Example**:

```json
{
  "id": 5,
  "type": "pyramid",
  "shape": {
    "prism_h": 1.2,
    "upper_h": 0.1,
    "lower_h": 0.5,
    "upper_indices": [2, 0, 3]
  }
}
```

#### Shape-Scalar Sync Groups

By default every randomizable shape scalar — `height` (prism), `prism_h` /
`upper_h` / `lower_h` (pyramid), and each of the 6 `face_distance` entries
(both types) — draws its own independent random sample, so a crystal built
from distributions with a symmetric mean (e.g. all six `face_distance` means
equal) is still perturbed into a generically unequal-sided hexagon on every
sample: nothing keeps the *individual draws* symmetric, only their means.

`sync_group` puts a subset of a crystal's shape scalars into the **same
group**, so all of them **share a single random draw** for that crystal
instance instead of drawing independently. This is how to express habits
that require exact same-sample symmetry — most notably a trigonal (C3)
hexagonal crystal, where faces 0/2/4 must always equal each other and faces
1/3/5 must always equal each other, sample by sample.

**Schema** — an optional object keyed by the scalar's own JSON field name
(the same names used elsewhere in the shape object); `face_distance` takes a
6-element integer array instead of a scalar, mirroring `face_distance` itself:

```json
"sync_group": {
  "height": 1,                        // prism only
  "prism_h": 1, "upper_h": 2, "lower_h": 2,   // pyramid only
  "face_distance": [1, 2, 1, 2, 1, 2]         // both types
}
```

- **`0` = independent** (the default — an absent `sync_group` key, or an
  absent per-scalar entry, means every scalar independent, identical to the
  behavior before sync groups existed). **`1..N` = group id**: every scalar
  carrying the same id shares one draw. Group ids only need to express a
  *partition* — they do not need to be dense or already sorted; `{1,2,1,2,1,2}`
  and `{2,1,2,1,2,1}` describe the same two groups and are normalized to the
  same canonical form on load (first-appearance renumbering, in the fixed
  scalar order `height`/`upper_h`/`prism_h`/`lower_h`/`face_distance[0..5]` —
  the RNG draw order, not the field declaration order above). A group left
  with only one member collapses back to independent (`0`).
- **Shared value, not a shared random variable**: a group draws exactly once
  per crystal instance; every member of the group receives that same raw
  value. It is not "these scalars are correlated" in some looser statistical
  sense — they are bit-identical for that instance.
- **The group's distribution comes from its leader**: the leader is the
  group's first member in RNG draw order (for `face_distance`, the
  lowest-indexed face in the group). Every other member's own distribution is
  overwritten with the leader's at load time. If a non-leader member declared
  a *different* distribution, that declaration is dropped and a warning is
  logged — it is not silently ignored and not rejected as an error.
- **Cross-kind groups (a height synced with a face distance) are mechanically
  legal but produce a documented asymmetry**: heights are folded through
  `abs()` before use (a negative height has no independent physical meaning)
  while `face_distance` stays signed (a negative face distance is a legal,
  origin-crossing plane offset — see above). A group mixing the two kinds
  therefore shares the same *raw* draw, but the height member consumes its
  absolute value while the face member keeps the sign — the two members are
  not numerically equal in that case, only equal up to sign. The mechanism
  does not forbid such a group; there is no dimensional-compatibility check.
- **Physical scope, stated honestly**: `height` / `face_distance`
  randomization only translates a face along its normal — it never rotates
  the face's normal direction (see
  [`geometry-randomization-value-and-measurement.md`](geometry-randomization-value-and-measurement.md)
  for the underlying measurement). Sync groups inherit that limit: they do
  not, by themselves, produce halo features at new angles. What they buy is
  **habit fidelity** — an ensemble whose individual crystals are genuinely
  symmetric (e.g. a true C3 trigonal population, not merely a C3-on-average
  one), which matters whenever roll is *not* uniformly random (Parry /
  Lowitz / fixed-roll configurations — under a full 360° uniform roll, the
  ensemble average recovers six-fold symmetry regardless of per-instance
  shape) — plus, as a side effect, syncing an opposite face pair (`i` and
  `i+3`) guarantees their distances sum to a value with the same sign
  structure they were configured with, which can reduce the geometric-validity
  rejection rate under strong face-distance randomization.

**C3 trigonal example** (verified: same-mean, same-spread Gaussian on every
face, two alternating sync groups — this is the exact scenario the
`SyncGroupPreview.C3GroupingCollapsesRandomDrawsToTwo` unit test exercises):

```json
{
  "id": 1,
  "type": "prism",
  "shape": {
    "height": 1.3,
    "face_distance": [
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 },
      { "type": "gauss", "mean": 1.0, "std": 0.1 }
    ],
    "sync_group": {
      "face_distance": [1, 2, 1, 2, 1, 2]
    }
  }
}
```

Each sampled crystal now has exactly two distinct face distances (faces
0/2/4 share one draw, faces 1/3/5 share the other) instead of six
independent ones — a strict trigonal habit on every sample, rather than only
on average.

#### Distribution Types

Many parameters support distribution types, which can be:

1. **Scalar value**: A deterministic value
   ```json
   "height": 1.3
   ```

2. **Distribution object**: Uniform or Gaussian distribution
   ```json
   "height": {
     "type": "gauss",
     "mean": 1.3,
     "std": 0.2
   }
   ```
   or
   ```json
   "height": {
     "type": "uniform",
     "mean": 0.5,
     "std": 0.4
   }
   ```

**Distribution type descriptions**:

| Type | `mean` | `std` | Description |
|------|--------|-------|-------------|
| `gauss` | center (deg) | standard deviation (deg) | Gaussian distribution for stable crystal orientation |
| `uniform` | center (deg) | full range width (deg) | Uniform distribution for random orientation or roll |
| `zigzag` | tilt offset (deg) | amplitude (deg) | Folded arcsine distribution for large-crystal zigzag oscillation |
| `laplacian` | center (deg) | scale parameter (deg) | Laplace distribution for size-aggregated tilt |
| `gauss_legacy` | center (deg) | standard deviation (deg) | Gaussian without Jacobian correction (for reproducing legacy results) |

**Notes:**
- `gauss` and `uniform` are the most commonly used types for halo simulation
- `zigzag` models the oscillatory motion of large crystals at high Reynolds numbers
- `laplacian` simplifies configuration for mixed-size crystal populations (see [crystal-orientation-sampling.md](crystal-orientation-sampling.md) for physical background)
- `gauss_legacy` reproduces the sampling behavior of earlier program versions that did not apply the spherical area Jacobian correction; use only when comparing against legacy simulation outputs

**Examples:**
```json
"zenith": { "type": "zigzag", "mean": 5, "std": 30 }
```
```json
"zenith": { "type": "laplacian", "mean": 90, "std": 2.0 }
```

### filter (Filter Configuration)

Filters are used to filter ray paths or directions.

#### Basic Structure

```json
{
  "id": <unique identifier>,
  "type": "none" | "raypath" | "entry_exit" | "direction" | "crystal" | "complex",
  "symmetry": "P" | "B" | "D" | "PBD" | ...,
  "action": "filter_in" | "filter_out",
  ...
}
```

#### Field Descriptions

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `id` | integer | yes | - | Unique identifier |
| `type` | string | yes | - | Filter type |
| `symmetry` | string | no | "" | Symmetry toggles: any combination of "P" (C6 rotational), "B" (horizontal mirror), "D" (vertical mirror). See [Raypath Symmetry](raypath-symmetry.md) for enabling conditions and precise semantics. |
| `action` | string | no | "filter_in" | Action: "filter_in" or "filter_out" |

#### Type-Specific Parameters

**1. none (No filter)**
```json
{
  "id": 1,
  "type": "none"
}
```

**2. raypath (Ray Path)**
```json
{
  "id": 2,
  "type": "raypath",
  "raypath": [3, 5],
  "symmetry": "P"
}
```
- `raypath`: Integer array of face numbers along the ray path.
  Face numbers follow the hex-crystal convention: basal = 1/2, prism = 3–8,
  upper pyramidal = 13–18, lower pyramidal = 23–28. The Crystal edit modal
  overlays the corresponding number on each visible face of the 3D preview.

**3. entry_exit (Entry/Exit)**
```json
{
  "id": 3,
  "type": "entry_exit",
  "entry": 3,
  "exit": 5,
  "action": "filter_in"
}
```
- `entry`: Entry face number
- `exit`: Exit face number

**4. direction (Direction)**
```json
{
  "id": 4,
  "type": "direction",
  "az": 180,
  "el": 25,
  "radii": 0.5,
  "action": "filter_out"
}
```
- `az`: Azimuth angle (degrees)
- `el`: Elevation angle (degrees)
- `radii`: Radius (degrees)

**5. crystal (Crystal)**
```json
{
  "id": 5,
  "type": "crystal",
  "crystal_id": 3
}
```
- `crystal_id`: Crystal ID

**6. complex (Composite)**
```json
{
  "id": 6,
  "type": "complex",
  "composition": [1, [2, 6], 5]
}
```
- `composition`: Filter composition expression

### scene (Scene Configuration)

The scene configuration defines the simulation scene, including the light source, crystal combinations, and number of rays. It is a single object (not an array).

#### Basic Structure

```json
{
  "light_source": { ... },
  "ray_num": <integer or "infinite">,
  "max_hits": <integer>,
  "scattering": [ ... ]
}
```

#### Field Descriptions

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `light_source` | object | yes | - | Inline light source configuration (see below) |
| `ray_num` | integer or string | yes | - | **Total** rays across all spectrum wavelengths; use `"infinite"` for continuous simulation |
| `max_hits` | integer | yes | - | Maximum number of hits |
| `scattering` | array | yes | - | Scattering configuration array |

> **`ray_num` is the total across all wavelengths.**
> Before task-323 (2026-07) `ray_num` was interpreted as *rays per discrete
> wavelength*, so a 9-line spectrum with `ray_num = 5_000_000` silently
> simulated 45,000,000 rays. That was a footgun: changing the wavelength count
> changed the total ray budget (and thus brightness/convergence).
>
> The server now interprets `ray_num` as the **grand total**. Internally it
> derives `per_wl = ⌈ray_num / N_wavelengths⌉` before dispatching to the trace
> loop, so the actual simulated total is `per_wl × N_wavelengths ≥ ray_num`
> (any small overshoot is due to the ceiling division). Illuminant / continuous
> spectra have `N_wavelengths = 1` so the transform is the identity.
>
> **Migration.** External hand-written configs that used the old per-wavelength
> semantics must be updated: multiply `ray_num` by the number of discrete
> wavelengths in the spectrum. The two bundled configs with discrete spectra
> (`examples/config_example.json`, `test/e2e/configs/color.json`) were migrated
> as part of task-323 and produce bit-equivalent trace output (PSNR unchanged).

#### light_source (Light Source Configuration)

The light source is defined inline within the `scene` object.

```json
{
  "type": "sun",
  "altitude": <angle>,
  "azimuth": <angle>,
  "diameter": <angle>,
  "spectrum": <spectrum configuration>
}
```

**Field descriptions**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `type` | string | yes | - | Light source type: "sun" |
| `altitude` | float | yes | - | Altitude above the horizon (degrees) |
| `azimuth` | float | no | 0.0 | Azimuth angle (degrees) |
| `diameter` | float | no | 0.5 | Diameter (degrees), typically 0.5 for the real Sun |
| `spectrum` | string or object array | yes | - | Spectrum configuration, see below |

##### spectrum (Spectrum Configuration)

`spectrum` supports two formats:

**1. Standard illuminant mode** (string) -- uses the spectral power distribution (SPD) of a CIE standard illuminant:
```json
"spectrum": "D65"
```
Supported standard illuminants: `"D50"`, `"D55"`, `"D65"`, `"D75"`, `"A"`, `"E"`

**2. Discrete wavelength mode** (object array) -- manually specifies wavelengths and weights:
```json
"spectrum": [
  {"wavelength": 420, "weight": 1.0},
  {"wavelength": 550, "weight": 1.0}
]
```

##### Light Source Notes

- Wavelength determines the refractive index; data is sourced from [Refractive Index of Crystals](https://refractiveindex.info/?shelf=3d&book=crystals&page=ice)
- `azimuth` and `diameter` are optional; default values are used when not specified
- In standard illuminant mode, the simulator uniformly samples wavelengths from the [380, 780] nm range, weighted by SPD

##### Light Source Examples

```json
"light_source": {
  "type": "sun",
  "altitude": 20.0,
  "azimuth": 0,
  "diameter": 0.5,
  "spectrum": "D65"
}
```

```json
"light_source": {
  "type": "sun",
  "altitude": 20.0,
  "diameter": 0.5,
  "spectrum": [
    {"wavelength": 420, "weight": 1.0},
    {"wavelength": 460, "weight": 1.0},
    {"wavelength": 500, "weight": 1.0},
    {"wavelength": 540, "weight": 1.0},
    {"wavelength": 580, "weight": 1.0},
    {"wavelength": 620, "weight": 1.0}
  ]
}
```

#### scattering (Scattering Configuration)

Each scattering configuration entry has the following structure:

```json
{
  "prob": <probability>,
  "entries": [
    {
      "crystal": <crystal ID>,
      "proportion": <float>,
      "filter": <filter ID>
    },
    ...
  ]
}
```

**Scattering entry fields**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `prob` | float | no | 0.0 | Multi-scattering probability |
| `entries` | object array | yes | - | Array of crystal entries |

**Entry object fields**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `crystal` | integer | yes | - | Crystal ID reference |
| `proportion` | float | no | 100.0 | Proportion weight |
| `filter` | integer | no | (none) | Filter ID reference; omit for no filter |

**Example**:

```json
"scene": {
  "light_source": {
    "type": "sun",
    "altitude": 20.0,
    "spectrum": "D65"
  },
  "ray_num": 1000000,
  "max_hits": 7,
  "scattering": [
    {
      "prob": 0.2,
      "entries": [
        {"crystal": 1, "proportion": 100},
        {"crystal": 2, "proportion": 50},
        {"crystal": 3, "proportion": 30}
      ]
    },
    {
      "entries": [
        {"crystal": 2, "proportion": 20, "filter": 2},
        {"crystal": 3, "proportion": 100, "filter": 1}
      ]
    }
  ]
}
```

### render (Render Configuration)

The render configuration defines the renderer parameters.

#### Basic Structure

```json
{
  "id": <unique identifier>,
  "lens": { ... },
  "resolution": [<width>, <height>],
  "lens_shift": [<x offset>, <y offset>],
  "view": { ... },
  "visible": "upper" | "lower" | "full",
  "background": [<r>, <g>, <b>],
  "ray_color": [<r>, <g>, <b>],
  "opacity": <float>,
  "intensity_factor": <float>,
  "grid": { ... },
  "filter": [<filter ID array>]
}
```

#### Field Descriptions

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `id` | integer | yes | - | Unique identifier |
| `lens` | object | no | see below | Lens configuration |
| `resolution` | integer array | yes | - | Resolution [width, height] |
| `lens_shift` | integer array | no | [0, 0] | Lens shift [x, y] |
| `view` | object | no | see below | View configuration |
| `visible` | string | no | "upper" | Visible hemisphere: "upper", "lower", or "full" |
| `background` | float array | no | [0, 0, 0] | Background color RGB |
| `ray_color` | float array | no | [-1, -1, -1] | Ray color RGB; -1 means use true color |
| `opacity` | float | no | 1.0 | Opacity |
| `intensity_factor` | float | no | 1.0 | Intensity factor |
| `grid` | object | no | see below | Grid configuration |
| `filter` | integer array | no | [] | Multi-scattering filter ID array |

#### lens (Lens Configuration)

```json
{
  "type": "linear" | "fisheye_equal_area" | "fisheye_equidistant" | "fisheye_stereographic" | "dual_fisheye_equal_area" | "dual_fisheye_equidistant" | "dual_fisheye_stereographic" | "rectangular" | "fisheye_orthographic" | "dual_fisheye_orthographic" | "globe",
  "fov": <angle>  // or "f": <focal length>
}
```

**Defaults**:
- `type`: "linear"
- `fov`: 90.0 (degrees); `globe` defaults to 30.0

**Note**:
- `fov` is the **full diagonal field of view** in degrees. For `rectangular` and `dual_*` types, `fov` is ignored (these are always full-sky projections).
- `fisheye_orthographic` and `dual_fisheye_orthographic` are capped at **180°** (the projection formula `r = f·sin(θ)` aliases past θ=90°); values above 180 are rejected.
- `dual_fisheye_orthographic` does not support the `overlap` parameter (silently ignored with a VERBOSE log entry).
- You can use `f` (focal length in mm, based on 35mm film) instead of `fov`. The program converts `f` to `fov` using the correct formula for each projection model:
  - Linear: `fov = 2·atan(d/f)`
  - Equal area: `fov = 4·arcsin(d/(2f))`
  - Equidistant: `fov = 2d/f` (radians → degrees)
  - Stereographic: `fov = 4·arctan(d/(2f))`
  - Orthographic: `fov = 2·arcsin(d/f)` (requires `f ≥ 12mm` for fov=180)
  - Rectangular: `f` is ignored (always full-sky)
  - Globe: `f` is not supported (use `fov` directly)

**`globe` lens (outside-in perspective view of the celestial sphere)**:
- Projection model: a pinhole perspective camera placed at distance `D = 4.0` (in unit-sphere radii) from the unit sphere centered at the world origin, looking toward the sphere center. The shader ray-traces against the unit sphere and shades the sample pointed to by the hit point.
- `fov` range: `(0°, 90°]`; default `30°`. With the default the sphere fills approximately 96% of the viewport's short edge.
- `view.roll`: stored as a normal `view` field, but at render time it is **forced to 0** for the `globe` lens; the right-panel Roll slider is greyed out while `globe` is selected. Switching to a non-Globe lens restores the stored value (the field is preserved, only the rendered roll is overridden).
- `view.azimuth` / `view.elevation` semantics: under `globe`, they describe the **observer's orbit around the sphere**, not the camera's own attitude. The view matrix is mathematically the same as for inside-out lenses, but the user-facing intuition is inverted (Az/El point the camera *at* a place on the sphere instead of *toward* a direction in the sky).
- `view.elevation` clamp under `globe`: trackball drag and the right-panel slider both clamp to `[-89°, +89°]` to avoid the view-matrix degeneracy at ±90°.
- `.lmc` compatibility: no new fields are introduced; older `.lmc` files load unchanged.

#### view (View Configuration)

```json
{
  "azimuth": <angle>,
  "elevation": <angle>,
  "roll": <angle>
}
```

**Defaults**:
- `azimuth`: 0.0
- `elevation`: 0.0
- `roll`: 0.0

#### grid (Grid Configuration)

```json
{
  "central": [ ... ],
  "elevation": [ ... ],
  "outline": <boolean>
}
```

**Field descriptions**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `central` | object array | no | [] | Central grid line configuration |
| `elevation` | object array | no | [] | Elevation grid line configuration |
| `outline` | boolean | no | true | Whether to show the celestial sphere outline |

**Grid line configuration**:

```json
{
  "value": <angle>,
  "color": [<r>, <g>, <b>],
  "opacity": <float>,
  "width": <float>
}
```

**Defaults**:
- `color`: [1.0, 1.0, 1.0] (white)
- `opacity`: 1.0
- `width`: 1.0

## Configuration Validation Rules

### ID Uniqueness Validation

- All `id` fields must be unique within their respective section (applies to `crystal`, `filter`, and `render` arrays)
- `id` must be greater than 0

### ID Reference Validity

- Crystal IDs referenced by `scene.scattering[].entries[].crystal` must exist in the `crystal` array
- Filter IDs referenced by `scene.scattering[].entries[].filter` must exist in the `filter` array
**Note**: `scene` is a single object, not an array. All renderers defined in the `render` array are automatically active.

### Array Length Validation

- `scene.light_source.spectrum` must be either a string (standard illuminant name) or an object array (each object containing `wavelength` and `weight`)
- `crystal[].shape.face_distance` array length must be 6 (if specified)
- `crystal[].shape.upper_indices` array length must be 3 (if specified)
- `crystal[].shape.lower_indices` array length must be 3 (if specified)
- `crystal[].shape.sync_group.face_distance` array length must be 6 (if specified); a shorter or
  longer array is truncated/zero-padded rather than rejected — see [Shape-Scalar Sync
  Groups](#shape-scalar-sync-groups)
- `render[].resolution` array length must be 2

### Value Range Validation

- Angle values are typically between -180 and 180 degrees (may exceed in certain cases)
- `crystal[].shape.upper_h` and `lower_h` should be between 0.0 and 1.0
- `render[].opacity` should be between 0.0 and 1.0
- `render[].background` and `ray_color` color values should be between 0.0 and 1.0

### Required Field Validation

- `scene.light_source`: `type`, `altitude`, `spectrum` are required
- `crystal`: `id`, `type`, `shape` are required
- `filter`: `id`, `type` are required
- `scene`: `light_source`, `ray_num`, `max_hits`, `scattering` are required
- `scene.scattering[].entries[]`: `crystal` is required
- `render`: `id`, `resolution` are required
### Type Validation

- `crystal[].type` must be "prism" or "pyramid"
- `scene.light_source.type` must be "sun"
- `filter[].type` must be "none", "raypath", "entry_exit", "direction", "crystal", or "complex"
- `render[].visible` must be "upper", "lower", or "full"
- `render[].lens.type` must be "linear", "fisheye_equal_area", "fisheye_equidistant", "fisheye_stereographic", "dual_fisheye_equal_area", "dual_fisheye_equidistant", "dual_fisheye_stereographic", "rectangular", "fisheye_orthographic", "dual_fisheye_orthographic", or "globe"

## Common Configuration Errors

### 1. Undefined ID Error

**Description**: Referencing a non-existent ID

**Incorrect example**:
```json
{
  "scene": {
    "scattering": [
      {
        "entries": [
          {"crystal": 999}  // Error: crystal ID 999 does not exist
        ]
      }
    ]
  }
}
```

**Correct example**:
```json
{
  "crystal": [
    { "id": 1, ... }
  ],
  "scene": {
    "scattering": [
      {
        "entries": [
          {"crystal": 1}  // Correct: references a defined crystal
        ]
      }
    ]
  }
}
```

### 2. Missing Crystal in Scattering Entry

**Description**: A scattering entry is missing the required `crystal` field

**Incorrect example**:
```json
{
  "scattering": [
    {
      "entries": [
        {"proportion": 50}  // Error: missing "crystal" field
      ]
    }
  ]
}
```

**Correct example**:
```json
{
  "scattering": [
    {
      "entries": [
        {"crystal": 1, "proportion": 50}  // Correct: crystal ID specified
      ]
    }
  ]
}
```

### 3. Type Error

**Description**: Using an incorrect crystal type name

**Incorrect example**:
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "HexPrism"  // Error: should use "prism"
    }
  ]
}
```

**Correct example**:
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism"  // Correct: use "prism"
    }
  ]
}
```

### 4. Missing Required Field

**Description**: A required field is missing

**Incorrect example**:
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism"
      // Error: missing "shape" field
    }
  ]
}
```

**Correct example**:
```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": {
        "height": 1.3
      }
    }
  ]
}
```

### 5. Incorrect Configuration Structure

**Description**: The configuration structure does not meet requirements

**Incorrect example**:
```json
{
  "scene": [  // Error: scene should be a single object, not an array
    {
      "light_source": { "type": "sun", "altitude": 20.0, "spectrum": "D65" },
      "ray_num": 1000000
    }
  ]
}
```

**Correct example**:
```json
{
  "scene": {  // Correct: scene is a single object
    "light_source": { "type": "sun", "altitude": 20.0, "spectrum": "D65" },
    "ray_num": 1000000,
    "max_hits": 7,
    "scattering": [
      { "entries": [{"crystal": 1}] }
    ]
  }
}
```

## Configuration Best Practices

### Performance Optimization Tips

1. **Ray count settings**:
   - Use a small `ray_num` (e.g., 10000) for testing
   - Adjust according to requirements for production; typically 1000000 or more

2. **Configuration reuse**:
   - Avoid defining duplicate crystals; reuse them via ID references in scattering entries

3. **Filter usage**:
   - Proper use of filters can reduce unnecessary computation
   - Use the `filter` field in `scene.scattering[].entries[]`

### Common Scene Configuration Templates

#### Simple Halo Simulation

```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": { "height": 1.2 }
    }
  ],
  "scene": {
    "light_source": {
      "type": "sun",
      "altitude": 20.0,
      "diameter": 0.5,
      "spectrum": "D65"
    },
    "ray_num": 1000000,
    "max_hits": 7,
    "scattering": [
      {
        "entries": [
          {"crystal": 1}
        ]
      }
    ]
  },
  "render": [
    {
      "id": 1,
      "resolution": [1920, 1080],
      "lens": { "type": "linear", "fov": 40 }
    }
  ]
}
```

#### Multi-Crystal Scattering Simulation

```json
{
  "scene": {
    "light_source": {
      "type": "sun",
      "altitude": 20.0,
      "spectrum": "D65"
    },
    "ray_num": 1000000,
    "max_hits": 7,
    "scattering": [
      {
        "prob": 0.2,
        "entries": [
          {"crystal": 1, "proportion": 100},
          {"crystal": 2, "proportion": 50},
          {"crystal": 3, "proportion": 30}
        ]
      },
      {
        "entries": [
          {"crystal": 2, "proportion": 20, "filter": 2},
          {"crystal": 3, "proportion": 100, "filter": 1}
        ]
      }
    ]
  }
}
```

### Debugging Tips

1. **Small-scale test configuration**:
   - `ray_num`: 100 or less
   - Use a single wavelength (`"spectrum": [{"wavelength": 550, "weight": 1.0}]`)
   - Reduce the number of crystals

2. **Detailed logging**:
   - Check configuration parsing logs to locate issues

## GUI Personal Defaults (User Overrides)

This is a **separate file from the scene configuration above** — it does not live next to `crystal`/`filter`/`scene`/`render` and is only read by the GUI, never by the CLI. It lets a user carry personal preferences (a favorite lens, a retuned axis preset) across every new document on their own machine, without hard-coding one user's habit as everyone's factory default.

#### File location

One JSON file per OS user-config directory, never next to the executable (a read-only install or a multi-user machine may not be writable there):

| Platform | Location |
|----------|----------|
| Windows | `%APPDATA%\Lumice\user_defaults.json` |
| macOS | `~/Library/Application Support/Lumice/user_defaults.json` |
| Linux | `$XDG_CONFIG_HOME/lumice/user_defaults.json`, falling back to `~/.config/lumice/user_defaults.json` |

The GUI's log file, previously written unconditionally to `$HOME`, now lives in the same directory.

#### Format: a sparse GuiState document, plus a presets subtree

The bulk of the file is **not a new format** — it is a partial dump of the same JSON the GUI's `.lmc` files serialize their configuration section from (the same document-level fields: `sun`, `sim`, `renderer`, and the view-only fields such as overlay colors, aspect preset, background image). Any key it omits falls back to the factory value, because the parser that reads it is the same one used everywhere else, and that parser has always treated a missing key as "use the factory default" — there is no separate merge logic for this file.

Alongside that, a `presets` subtree carries per-preset overrides for the built-in axis-orientation presets, e.g.:

```json
{
  "renderer": { "lens_type": "..." },
  "presets": {
    "axis": {
      "column": { "zenith_std": 0.3 }
    }
  }
}
```

Only the zenith-std of a built-in preset can be overridden, and only within that preset's existing classification tolerance — a value outside the domain is clamped to the boundary rather than rejected, and the clamp is reported (see below). Presets with no adjustable face (e.g. the fully-uniform preset) refuse a stored value entirely.

#### Degraded-input behavior

An unparseable or non-object file, a field with the wrong JSON type, a non-numeric/non-finite preset value, or a clamped preset value are all reported through one notice channel rather than several ad hoc warnings, so a user can always find the full list of what was ignored or adjusted in one place. A field-level type error discards the whole GUI-state half of the overlay (never a half-applied result); the presets subtree is parsed independently.

**Personal defaults only ever apply to a brand-new document** — they never override a value already present in a file being opened, whether that file is a `.lmc` project or a CLI JSON config imported through the GUI. Loading someone else's file always reproduces what that file itself specifies (or the factory value for anything it omits), regardless of what is saved on the machine doing the loading.

#### `--user-config` / `--no-user-config`

Two CLI switches (deliberately not environment variables — a user-facing behavior switch left to an env var causes silent per-machine drift):

- `--user-config <dir>`: use the given directory instead of the OS default, for testing or for running multiple isolated profiles.
- `--no-user-config`: skip personal defaults entirely; every new document uses factory values only.

The interactive GUI binary defaults to auto-detecting the OS directory when neither flag is passed. The GUI test binary defaults to disabled instead, so a visual-regression reference image never depends on whichever `user_defaults.json` happens to exist on the machine that captured it.

## Related Documentation

- [README](../README.md): User documentation
- [System Architecture](architecture.md): System architecture documentation
- [Example Configuration File](../examples/config_example.json): Example configuration file
