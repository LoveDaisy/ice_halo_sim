[中文版](04-recipes_zh.md)

# Recipes — Reproduce Classic Halos

This chapter gives you three runnable recipes that reproduce well-known halo phenomena. Each recipe lists:

- the phenomenon and what to look for in the output;
- the minimal JSON config (drop-in for `Lumice -f <file>`);
- expected runtime and visual signature;
- pointers into the schema reference for tweaking.

> Recipes are minimal by design: they keep `ray_num` modest so the first run finishes quickly. To get a clean, low-noise image, raise `ray_num` to `5e7` or higher once the recipe behaves as expected.

## Recipe 1 — 22° halo (the classic)

The most familiar halo: a bright ring at exactly 22° from the sun, caused by light refracting through the 60° prism faces of randomly-oriented hexagonal columns.

**Expected output**: a continuous ring centred on the sun position, brightest on its inner edge, fading outwards. The reference render below uses 50M rays:

![22° halo reference render](../figs/sim05E_50M.jpg)

**Config** (`recipes/22-halo.json`, inline):

```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": { "height": 1.2 },
      "axis": {
        "azimuth": { "type": "uniform", "mean": 0, "std": 360 },
        "zenith":  { "type": "uniform", "mean": 0, "std": 360 },
        "roll":    { "type": "uniform", "mean": 0, "std": 360 }
      }
    }
  ],
  "filter": [{ "id": 1, "type": "none", "symmetry": "P" }],
  "scene": {
    "light_source": {
      "type": "sun", "altitude": 20.0,
      "spectrum": [
        {"wavelength": 420, "weight": 1.0},
        {"wavelength": 550, "weight": 1.0},
        {"wavelength": 660, "weight": 1.0}
      ]
    },
    "ray_num": 1000000, "max_hits": 7,
    "scattering": [
      { "prob": 0.0, "entries": [{ "crystal": 1, "proportion": 1.0, "filter": 1 }] }
    ]
  },
  "render": [
    {
      "id": 1, "lens": { "type": "fisheye_equidistant", "fov": 60 },
      "resolution": [800, 800],
      "view": { "elevation": 20, "azimuth": 0 }
    }
  ]
}
```

The three `uniform` distributions (`mean=0, std=360` on each Euler angle) make the orientation isotropic on the sphere — the sampler recognises this as the canonical full-sphere case (`AxisDistribution::IsFullSphereUniform` in `src/core/math.cpp`). Without `axis`, the crystal is locked to a single fixed orientation and you get arcs/spots, not a closed 22° ring. The single scattering entry with `prob: 0.0` makes every outgoing ray exit (single-scatter recipe); see Recipe 3 for the multi-scatter form.

Run: `./build/cmake_install/Lumice -f recipes/22-halo.json -o /tmp/out`.

For the prism field semantics, see [`../configuration.md`](../configuration.md) §`crystal`.

---

## Recipe 2 — Sun dogs (parhelia)

Bright spots ~22° to the **left and right** of the sun, caused by hexagonal **plate** crystals oriented with their c-axis vertical (basal faces horizontal). They are common around sunrise/sunset when the sun is low.

**Expected output**: two concentrated bright patches at the sun's altitude, on each side at ~22° azimuth. Use `sun.altitude=10` for a textbook view.

> 📷 待补：Recipe 2 reference render (registered in `progress.md` placeholder list — to be folded into SUMMARY.md "待补充清单" at closeout).

**Config** (`recipes/sun-dogs.json`, inline):

```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": { "height": 0.3 },
      "axis": {
        "zenith": { "type": "gauss", "mean": 0,   "std": 1.0 },
        "roll":   { "type": "uniform", "mean": 0, "std": 360 }
      }
    }
  ],
  "filter": [{ "id": 1, "type": "none", "symmetry": "P" }],
  "scene": {
    "light_source": {
      "type": "sun", "altitude": 10.0,
      "spectrum": [
        {"wavelength": 420, "weight": 1.0},
        {"wavelength": 550, "weight": 1.0},
        {"wavelength": 660, "weight": 1.0}
      ]
    },
    "ray_num": 1000000, "max_hits": 7,
    "scattering": [
      { "prob": 0.0, "entries": [{ "crystal": 1, "proportion": 1.0, "filter": 1 }] }
    ]
  },
  "render": [
    {
      "id": 1, "lens": { "type": "fisheye_equidistant", "fov": 60 },
      "resolution": [800, 800],
      "view": { "elevation": 10, "azimuth": 0 }
    }
  ]
}
```

Notes:

- `height: 0.3` makes the crystal a flat plate (low aspect ratio).
- `axis.zenith ~ Gauss(0, 1°)` keeps the c-axis nearly vertical (plate orientation), with a small wobble for realism.
- `roll ~ Uniform(0, 360°)` lets the prism rotate freely around its vertical axis.

For the full distribution syntax (`gauss`, `uniform`, `laplacian`, `zigzag`), see [`../crystal-orientation-sampling.md`](../crystal-orientation-sampling.md).

---

## Recipe 3 — 44° parhelia from multiple scattering

> **CLI only — multi-layer scattering requires JSON config; see [`05-faq.md`](05-faq.md) §"GUI vs JSON capabilities".**

When light is scattered twice by parallel plate crystals, a fainter ring appears at ~44° from the sun (twice the prism deviation). Reproducing it requires **two scattering layers**, each independently configurable — something only the JSON config supports.

**Expected output**: a dim outer ring at ~44°, in addition to the bright 22° halo (from single scattering). Reference renders:

![44° parhelia reference](../figs/44-degree%20parhelia.jpg)
![Plate multi-scatter GUI preview](../figs/gui_plate_multi_scatter.jpg)

**Config** (`recipes/44-parhelia.json`, inline):

```json
{
  "crystal": [
    {
      "id": 1,
      "type": "prism",
      "shape": { "height": 0.3 },
      "axis": {
        "zenith": { "type": "gauss", "mean": 0,   "std": 1.0 },
        "roll":   { "type": "uniform", "mean": 0, "std": 360 }
      }
    }
  ],
  "filter": [{ "id": 1, "type": "none", "symmetry": "P" }],
  "scene": {
    "light_source": {
      "type": "sun", "altitude": 20.0,
      "spectrum": [
        {"wavelength": 420, "weight": 1.0},
        {"wavelength": 550, "weight": 1.0},
        {"wavelength": 660, "weight": 1.0}
      ]
    },
    "ray_num": 5000000, "max_hits": 12,
    "scattering": [
      { "prob": 1.0, "entries": [{ "crystal": 1, "proportion": 1.0, "filter": 1 }] },
      { "prob": 0.0, "entries": [{ "crystal": 1, "proportion": 1.0, "filter": 1 }] }
    ]
  },
  "render": [
    {
      "id": 1, "lens": { "type": "fisheye_equidistant", "fov": 90 },
      "resolution": [900, 900],
      "view": { "elevation": 20, "azimuth": 0 }
    }
  ]
}
```

Notes:

- The **two scattering entries** model "every ray hits a plate, then every survivor hits another plate". `scattering[i].prob` is the probability that an outgoing ray from layer `i` *continues* into layer `i+1` (the last entry's `prob` flips that ray to "outgoing", so set it to `0.0`). With `[1.0, 0.0]` every ray scatters exactly twice — the canonical setup for "multiple scattering" — see [`../configuration.md`](../configuration.md) §`scattering`.
- `max_hits` is raised to `12` because a doubly-scattered ray can take more bounces before exiting.
- The 44° ring is dim — `ray_num=5e6` gives a usable preview; raise to `5e7` for a clean image.

## Where to go from here

- Tweak any recipe and re-run from the CLI → [`03-cli-quickstart.md`](03-cli-quickstart.md).
- Understand performance trade-offs (`ray_num × wavelengths`) → [`05-faq.md`](05-faq.md).
- Full schema with every field → [`../configuration.md`](../configuration.md).
- Coordinate system used by `altitude` / `zenith` / `azimuth` → [`../coordinate-convention.md`](../coordinate-convention.md).
