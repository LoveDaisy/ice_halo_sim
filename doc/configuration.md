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
- The example configuration file (`config_example.json`) does not exhaustively cover all valid configuration patterns
- Many configuration parameters have default values that will be used when not explicitly specified
- **You should refer to the source code** for the complete configuration logic and default values
- This document extracts all default value information based on the code implementation

## Configuration Fields

### crystal (Crystal Configuration)

The crystal configuration defines the crystal shapes and orientation distributions used in the simulation.

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
  "face_distance": [<6 values or distributions>]
}
```

**Field descriptions**:

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `height` | value/distribution | no | 1.0 | Height ratio h/a, where h is the prism height and a is the base diameter |
| `face_distance` | array | no | [1,1,1,1,1,1] | Distance ratios for 6 faces; [1,1,1,1,1,1] for a regular hexagon |

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
  "face_distance": [<6 values or distributions>]
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
- `gauss`: Gaussian distribution, where `mean` is the mean and `std` is the standard deviation
- `uniform`: Uniform distribution, where `mean` is the center value and `std` is half the range

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
| `symmetry` | string | no | "" | Symmetry: "P" (planar), "B" (basal), "D" (direction) |
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
- `raypath`: Integer array of face numbers along the ray path

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
| `ray_num` | integer or string | yes | - | Number of rays; use `"infinite"` for continuous simulation |
| `max_hits` | integer | yes | - | Maximum number of hits |
| `scattering` | array | yes | - | Scattering configuration array |

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
  "type": "linear" | "fisheye_equal_area" | "fisheye_equidistant" | "fisheye_stereographic" | "dual_fisheye_equal_area" | "dual_fisheye_equidistant" | "dual_fisheye_stereographic" | "rectangular",
  "fov": <angle>  // or "f": <focal length>
}
```

**Defaults**:
- `type`: "linear"
- `fov`: 90.0 (degrees)

**Note**: You can use either `fov` (field of view in degrees) or `f` (focal length in mm). If `f` is used, the program automatically calculates the corresponding `fov`.

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
- `render[].lens.type` must be "linear", "fisheye_equal_area", "fisheye_equidistant", "fisheye_stereographic", "dual_fisheye_equal_area", "dual_fisheye_equidistant", "dual_fisheye_stereographic", or "rectangular"

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

## Related Documentation

- [README](../README.md): User documentation
- [System Architecture](architecture.md): System architecture documentation
- [Example Configuration File](../config_example.json): Example configuration file
