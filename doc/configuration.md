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

There are two different "missing" cases here, and they produce different behavior —
do not treat one as a shorthand for the other.

**`axis` entirely absent** — the crystal takes one fixed orientation, no randomization at all:

```json
{
  "zenith": 0.0,
  "azimuth": 0.0,
  "roll": 0.0
}
```

**`axis` present but `azimuth` / `roll` omitted** — the omitted axis rotates freely and
uniformly over 0-360°. For example, writing only `"axis": {"zenith": 30}` is equivalent to:

```json
{
  "zenith": 30.0,
  "azimuth": { "type": "uniform", "mean": 180.0, "std": 360.0 },
  "roll": { "type": "uniform", "mean": 180.0, "std": 360.0 }
}
```

(`mean`/`std` are the on-disk keys for every distribution type; for `uniform` they hold the
interval midpoint and full width, not a statistical mean/standard-deviation.)

`zenith` has no such fallback: if `axis` is written at all, `zenith` is required — see
[Required Field Validation](#required-field-validation) below.

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
| `upper_h` | value/distribution | no | 0.0 | Upper pyramid segment relative height — see [Pyramid Shape Legality](#pyramid-shape-legality) below |
| `lower_h` | value/distribution | no | 0.0 | Lower pyramid segment relative height — see [Pyramid Shape Legality](#pyramid-shape-legality) below |
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
| `prob` | float | yes | - | Multi-scattering probability. Was optional and silently defaulted to `0.0`; omitting it is now an error. To keep the old behavior, write `"prob": 0` explicitly. |
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
  "front": <bool>,
  "background": [<r>, <g>, <b>],
  "ray_color": [<r>, <g>, <b>],
  "intensity_factor": <float>,
  "ev_mode": "relative" | "absolute",
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
| `front` | boolean | no | false | Front-hemisphere clip: keep only what the camera faces. A SECOND clip dimension, independent of `visible` and ANDed with it — not a fourth `visible` value. (Writing `"visible": "front"` is silently read as `"upper"`, so use this key.) |
| `background` | float array | no | [0, 0, 0] | Background color RGB, in **sRGB** (the numbers a color picker shows) |
| `ray_color` | float array | no | [-1, -1, -1] | Ray color RGB; -1 means use true color |
| `intensity_factor` | float | no | 1.0 | Intensity factor (`2^EV`) |
| `ev_mode` | string | no | "relative" | Exposure anchor: `"relative"` self-anchors to the frame's own P99 (the historical behavior — the image keeps its look as `ray_num` grows, but the config alone does not determine output brightness); `"absolute"` anchors to the light source's emitted energy, so two renders at the same `intensity_factor` are directly comparable across configs (only within one lens/FOV/resolution — see [`doc/ev-pipeline-architecture.md`](ev-pipeline-architecture.md) §7). A missing key or an unrecognized string both mean `"relative"`. See [`doc/adaptive-brightness.md`](adaptive-brightness.md) §3. |
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
  "angular_dist": [ ... ],
  "elevation": [ ... ],
  "longitude": [ ... ],
  "horizon": <boolean>,
  "horizon_label": <boolean>,
  "label": <boolean>,
  "angular_dist_label": <boolean>
}
```

**Field descriptions**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `angular_dist` | object array | no | [] | Circles of constant angular distance from the sun — 22 and 46 being the halos most configs draw. **Rendered.** `value`, `opacity` and `color` take effect; `width` does not (see below). Read from the legacy key `central` when this one is absent. |
| `elevation` | object array | no | [] | Parallels — lines of constant elevation, in degrees. **Rendered** (as of v4.18; earlier versions parsed the list and drew nothing). Same rules as `angular_dist`: `value`, `opacity` and `color` take effect, `width` does not. |
| `longitude` | object array | no | [] | Meridians — lines of constant azimuth, in degrees. **Rendered.** Same rules as `elevation`. Added in v4.18; a file without the key gets an empty list, which draws nothing. |
| `horizon` | boolean | no | false | Draw a line along the celestial horizon (altitude 0), in the visible hemisphere only. Opt-in: set it to `true` to get the line. |
| `horizon_label` | boolean | no | false | Draw the horizon's TEXT label (`0°`). Added in v4.21. Independent of `horizon`: the label appears with the line switched off. |
| `label` | boolean | no | false | Draw the TEXT labels for `elevation` and `longitude` — the angle each line stands for. One switch for both families, matching the GUI's single grid label control. Added in v4.21. |
| `angular_dist_label` | boolean | no | false | Draw the TEXT labels for `angular_dist` (`22°`, `46°`). Added in v4.21. |

**The three `*_label` switches, and the one thing they do NOT control**

They decide whether the label GEOMETRY is computed, which is independent of whether the family's
own line is drawn. For the horizon that independence is directly usable — `horizon_label: true`
with `horizon: false` renders the numbers and no line. For the other two families it is not, and
the reason is the schema rather than the switch: "is this family drawn" IS "is its angle list
non-empty", so there is no way to ask for `elevation` labels without also asking for the parallels
themselves.

What they do NOT control is the label's OPACITY. A label is painted in its family's own colour and
opacity — each `angular_dist` / `elevation` / `longitude` entry's own `opacity` and `color`, and a
fixed constant for the horizon — so a line at `"opacity": 0` is invisible together with its labels.
This mirrors the GUI, where a label has always taken its family's appearance; a renderer that let a
label outlive its line would show something the preview cannot.

Rendering is core's own: the CLI has no ImGui, so it embeds a typeface and rasterizes the labels
itself, from the same anchors the GUI preview reads. The two agree about WHICH labels appear and
where the curve puts them. They differ in two ways that are deliberate and will not be reconciled
by a threshold: the glyphs come out of two different rasterizers, and the GUI additionally clamps a
label inside the viewport and drops one that collides with another, while the CLI draws every
anchor where it lands.

**`central` is the old name for `angular_dist`**

The key was renamed because `central` never said what the number is: it is the angular distance
from the sun. Both decoders read `central` as an alias, with `angular_dist` winning if a file
somehow carries both; the encoder only ever writes `angular_dist`, so a config loaded and saved
comes back under the new name. Old files keep working indefinitely — there is no version gate on
this, because the two spellings never meant different things.

The C API field was renamed to match (`LUMICE_RenderParam.angular_dist` / `angular_dist_count`,
was `central_grid` / `central_grid_count`). That is a source-compatibility break with no layout
change; see the BREAKING note at `LUMICE_API_VERSION` in `src/include/lumice.h`.

**What the three line families draw, and what they ignore**

`value`, `opacity` and `color` all take effect on every one of them: each line is drawn as its own
curve, in its own colour, blended at its own opacity. `width` is read, validated and round-tripped
but changes no pixel — a line's thickness comes from the local gradient of the field it is a level
set of, which keeps a curve one line wide wherever it runs across a projection that stretches, and
takes no width input. A config that sets `width` is not in error; it simply gets the same picture.

**`longitude`, and why not `azimuth`**

The meridians are named for the same word the annotation layer already uses for them everywhere it
is public (`LUMICE_ANNOTATION_LONGITUDE`, `LUMICE_AnnotationRequest::longitude_deg`). Calling the
persisted key `azimuth` would have given one concept two vocabularies for no gain.

**How the GUI fills `elevation` and `longitude` when it exports**

The GUI has no per-line grid controls: it derives ONE step from the current field of view (30 deg
at 120 and wider, down to 0.5 deg below 2) and one shared colour and opacity for the whole grid.
Exporting expands that step into the explicit list this schema is built on — parallels every step
over ±80 deg with 0 excluded (the horizon owns that curve and carries its own colour), meridians
every step over the half-open (-180, 180] so the anti-meridian appears once — and repeats the
shared colour and opacity on every entry.

The adaptive step is therefore a display-side convenience, not part of the model: the list is the
model, and a config can name any parallels and meridians it likes, at any spacing, with a different
colour on each.

One consequence is worth knowing before zooming in: a narrow field of view picks a fine step, and a
fine step can expand past `LUMICE_MAX_CONFIG_GRID_LINES` (64) — 72 meridians at a 20 deg field of
view. The GUI REFUSES that export with a message naming the family and the count rather than
writing a truncated grid, because a shortened grid would be a CLI render that quietly differs from
what is on screen. Widen the field of view, or turn the grid off, and export again.

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
- `crystal[].shape.upper_h` and `lower_h` are not clamped to `[0.0, 1.0]`: negative
  values fold to their absolute value before use, and any value `>= 1.0` reaches the
  same full-apex result as exactly `1.0` — it is not an error. See [Pyramid Shape
  Legality](#pyramid-shape-legality) below for what each range actually produces.
- `render[].background` and `ray_color` color values should be between 0.0 and 1.0
- `render[].background` is **sRGB**: on a pixel with no halo energy, the rendered color is exactly
  the triple written here. It is converted to linear when the config is read, because the
  background is added to the halo's radiance and that addition only means anything in linear
  space; it is converted back to sRGB whenever a config is written out. (`ray_color` is linear —
  it is a tint applied to radiance, not a color the viewer sees directly. The `LUMICE_RenderParam`
  C struct is linear on both fields; only the JSON keys differ.)

### Pyramid Shape Legality

- **`upper_h` / `lower_h` boundaries**:
  - `0.0` (or any value that folds to `0.0`): that side has no cone at all — it is
    capped by a basal face instead (see the `prism_h == 0.0` cases below for how that
    cap attaches when there is no straight prism band either).
  - `(0.0, 1.0)`: the cone is truncated below its apex (a frustum), and that side gets
    a basal face at the truncation plane.
  - `>= 1.0`: the cone reaches its full apex — no basal face on that side. A value
    above `1.0` is not an error; it clamps to the same result as exactly `1.0`.
  - A narrow band of `upper_h`/`lower_h` values immediately below `1.0` (on the order
    of `1e-4` wide) snaps to the exact full apex rather than the fractional height
    technically requested. This is intentional: it replaces a former defect where
    crystals landing in that band silently lost the cone entirely. See [Common
    Configuration Errors](#common-configuration-errors) below for the one case where
    the snap logs a warning.
- **Wedge angle legality**: a cone's wedge angle — `upper_alpha`/`lower_alpha` when set
  directly, or derived from `upper_indices`/`lower_indices` (Miller indices) — must
  fall within `[0.1°, 89.9°]` — both endpoints included, so exactly `0.1` and exactly
  `89.9` still build a cone. Outside that range, the cone on that side is
  treated as absent, the same as `upper_h`/`lower_h` folding to `0.0` — silently, with
  no warning of its own (the shape still builds fine as long as the rest of the
  crystal — the other side's cone, or the prism band — still gives it enough faces).
- **`prism_h == 0.0` (no straight prism band)** is legal in every combination of
  cones:
  - *one side has a cone, the other does not*: legal. The cone-less side's basal cap
    is built directly from the corners of the cone-bearing side's wall ring — there is
    no prism wall to attach it to instead.
  - *both sides have a cone*: legal — a bipyramid with no straight band. The two cones
    meet at a single ring; neither side gets a basal cap (a basal cap only exists on a
    side that has no cone).
  - *neither side has a cone*: zero-volume. The crystal is dropped and contributes
    zero energy, same as any other degenerate shape — this is the only one of the
    three combinations that is not a legal solid.
- **Irregular `face_distance` can turn an apex into a ridge, not a point** — this is
  legal geometry, not a degenerate input. When the six `face_distance` values are not
  all equal, the search that locates a cone's apex can have a *tied* maximum along a
  line segment instead of a single point, so the "tip" is a two-endpoint ridge. Each
  endpoint belongs only to the subset of the six cone faces whose planes actually pass
  through it — not all six.

### Required Field Validation

- `scene.light_source`: `type`, `altitude`, `spectrum` are required
- `crystal`: `id`, `type`, `shape` are required
- `filter`: `id`, `type` are required
- `scene`: `light_source`, `ray_num`, `max_hits`, `scattering` are required
- `scene.scattering[]`: `prob` is required (no default — missing `prob` is rejected; there is
  no implicit fallback to 0, so a config that wants the historical no-multi-scattering behavior
  must write `"prob": 0` explicitly)
- `scene.scattering[].entries[]`: `crystal` is required
- `render`: `id`, `resolution` are required
- `crystal[].axis`, if present at all, requires `zenith`; `azimuth` and `roll` may each be
  omitted independently (see [axis Defaults](#axis-orientation-distribution-defaults) above)
- Any distribution slot written as an object — `axis.{zenith,azimuth,roll}`, and the shape
  scalars `height` / `prism_h` / `upper_h` / `lower_h` / `face_distance[]` — requires `type`.
  A distribution slot may instead be written as a bare number for a fixed value (e.g.
  `"zenith": 30`), which needs no `type`.
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
        "prob": 0,
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
        "prob": 0,
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
      "prob": 0,
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
      "prob": 0,
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
      { "prob": 0, "entries": [{"crystal": 1}] }
    ]
  }
}
```

### 6. Reading the Apex-Rescue Warning

**Description**: `upper_h` or `lower_h` landed close enough to `1.0` that the solver
snapped that cone to its exact full apex instead of building the fractional height
requested. The resulting crystal is still a valid solid — this is not a rejection —
but the message is worth reading, because it means the request landed on the
tolerance boundary between "truncated" and "full apex" (see [Pyramid Shape
Legality](#pyramid-shape-legality) above).

**What it looks like** (from `src/core/geo3d_closedform.cpp`):
```
ComputeClosedFormPyramid: <upper|lower> cone has no cross-section at inset <m>
(apex inset <apex_m>), yet is not recognised as collapsed; face_distance=[...].
Degrading that cone to a single apex point. To get the truncated shape instead,
move upper_h/lower_h away from 1.0 or widen the spread between face_distance
entries.
```

**How to respond**: follow the message's own suggestion — move `upper_h`/`lower_h`
further from `1.0`, or widen the spread between the `face_distance` entries. On the
shipped tolerance binding this path is not expected to fire at all (a 413,280-point
sweep across the near-apex band never triggered it); if you do see it, you have found
an edge case narrower than the ones already closed, and the same two remedies apply.

### 7. Reading the Dropped-Face Warning

**Description**: One of the crystal's faces came out thinner than the geometry solver
can resolve — typically a side face whose two bounding corners are a few times
`1e-5` apart relative to the crystal's own size. The solver reached that face while
walking the cross sections, but could not keep three distinct vertices on it, so the
face is dropped. Unlike the apex-rescue case above, the result is **not** a valid
solid: the faces beside the dropped one keep the edges it was supposed to close, so
the surface has a hole in it.

Nothing downstream rejects such a crystal — the legality check counts present faces
and does not test whether they form a closed surface — so this warning is the only
signal you get. Rays traced through it will refract at the remaining faces and the
run will report statistics as usual.

**What it looks like** (from `src/core/geo3d_closedform.cpp`):
```
ComputeClosedFormPyramid: face slot(s) [13] were reached by the emitter but kept
fewer than 3 distinct vertices, so they are dropped and the surface they bounded is
left open; face_distance=[...], upper_h_inset=..., lower_h_inset=... This crystal has
a face thinner than the solver can resolve. To get a well-formed solid instead, widen
the spread between the face_distance entries around the offending face, or move
upper_h/lower_h away from that face's collapse height.
```

**How to respond**: the slot numbers in the message name the offending faces — slots
`0`/`1` are the two basal faces, `2`–`7` the six prism sides, `8`–`13` the upper cone
faces, `14`–`19` the lower cone faces, each `i`-th entry sharing direction `i` with
the corresponding `face_distance[i]`. Widen the spread between the `face_distance`
entries around that direction so the face is either comfortably present or cleanly
absent, rather than sitting on the resolution boundary. Moving `upper_h`/`lower_h`
also helps when the face collapses partway up a cone rather than at the shoulder.

### 8. Missing `prob` in a Scattering Layer

**Description**: A `scene.scattering[]` layer is missing the required `prob` field. There is no
implicit default — this used to silently fall back to `0.0`, but that fallback has been removed.

**Error message**:
```text
scene.scattering[0] is missing required field "prob" (multi-scattering probability). The
historical default was 0.0; add "prob": 0 explicitly to keep that behavior.
```

**Incorrect example**:
```json
{
  "scattering": [
    {
      // Error: missing "prob"
      "entries": [{"crystal": 1}]
    }
  ]
}
```

**Correct example**:
```json
{
  "scattering": [
    {
      "prob": 0,  // Correct: explicit even if you want the old no-multi-scattering behavior
      "entries": [{"crystal": 1}]
    }
  ]
}
```

### 9. Distribution Object Missing `type`

**Description**: A distribution slot (`axis.zenith` / `axis.azimuth` / `axis.roll`, or a shape
scalar such as `height`, `prism_h`, `upper_h`, `lower_h`, or an entry of `face_distance[]`) is
written as a JSON object but omits `type`. A bare number needs no `type`; an object does.

**Error message** (axis slots name themselves; shape scalars share one generic message that does
not name the field):
```text
axis.zenith is a distribution object with no "type". Write axis.zenith either as a bare number
for a fixed angle (e.g. "zenith": 20) or as an object naming the distribution (e.g. "zenith":
{"type": "gauss", "mean": 20, "std": 5}).
```
```text
distribution object is missing required key "type". Write either a bare number (e.g. 20) or an
object naming the distribution (e.g. {"type": "gauss", "mean": 20, "std": 5}).
```

**Incorrect example**:
```json
{
  "axis": {
    "zenith": { "mean": 20, "std": 5 }  // Error: missing "type"
  }
}
```

**Correct example**:
```json
{
  "axis": {
    "zenith": { "type": "gauss", "mean": 20, "std": 5 }  // Correct
  }
}
```

### 10. `axis` Present but Missing `zenith`

**Description**: Once `axis` is written at all, `zenith` is required — there is no default for
it in that case. (Omit `axis` entirely to get the default fixed orientation instead; see
[axis Defaults](#axis-orientation-distribution-defaults) above.)

**Error message**:
```text
axis is present but has no "zenith", which is required whenever `axis` is written at all (omit
`axis` entirely to get the default orientation instead). Write axis.zenith either as a bare
number for a fixed angle (e.g. "zenith": 20) or as an object naming the distribution (e.g.
"zenith": {"type": "gauss", "mean": 20, "std": 5}).
```

**Incorrect example**:
```json
{
  "axis": { "azimuth": 0 }  // Error: "axis" is present but "zenith" is missing
}
```

**Correct example**:
```json
{
  "axis": { "zenith": 30, "azimuth": 0 }  // Correct
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
        "prob": 0,
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

#### Editing: one write per Save, never a live view of the file

The `Settings` panel (opened from the top-bar `Settings` button; `src/gui/defaults_panel.cpp`) is not a live view of this file. It reads the file once when it opens, every edit made in the panel — a checkbox, a value typed into a cell, "Reset all" — changes only that in-memory copy, and **the file itself is written exactly once, when Save is pressed**. Closing the panel any other way (Close, the title-bar X, Esc) discards the copy; nothing on disk moves.

The panel's table names two different values of the same key, which is worth spelling out here because the two are easy to conflate:

- **"Current value"** — the working copy's value for this key, i.e. what Save would write right now. It is not what the file on disk currently holds; that is a separate "Source" column ("Mine" when the key is present in the file as it was when the panel opened, "Factory" otherwise).
- **"Origin value"** — the literal factory value, `GuiState{}` serialized with nothing layered on top. It is not "the effective default before you started editing" — a key you have saved a non-factory value for shows the same Origin value regardless of what is stored.

See `doc/gui-state-governance.md` §8 for the panel's internal architecture (how a row's existence is generated from the serialized document while its editor is a separate registered table, and the full shape of the copy model above).

#### `--user-config` / `--no-user-config`

Two CLI switches (deliberately not environment variables — a user-facing behavior switch left to an env var causes silent per-machine drift):

- `--user-config <dir>`: use the given directory instead of the OS default, for testing or for running multiple isolated profiles.
- `--no-user-config`: skip personal defaults entirely; every new document uses factory values only.

The interactive GUI binary defaults to auto-detecting the OS directory when neither flag is passed. The GUI test binary defaults to disabled instead, so a visual-regression reference image never depends on whichever `user_defaults.json` happens to exist on the machine that captured it.

## Related Documentation

- [README](../README.md): User documentation
- [System Architecture](architecture.md): System architecture documentation
- [Example Configuration File](../examples/config_example.json): Example configuration file
