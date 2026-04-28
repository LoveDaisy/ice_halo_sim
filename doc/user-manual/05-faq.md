[中文版](05-faq_zh.md)

# FAQ, Defaults, and Known Limitations

A grab bag of "things that surprise new users". Skim the headings and dive into whichever matches your question.

## 1. Default config quick reference

Lumice's JSON schema requires only a handful of fields; the rest fall back to defaults. The table below summarises the most-asked-about ones (full spec: [`../configuration.md`](../configuration.md)).

| Field | Required? | Default | Notes |
|-------|-----------|---------|-------|
| `scene.ray_num` | yes | — | Per-wavelength ray budget; can be `"infinite"` for an open-ended run |
| `scene.max_hits` | yes | — | Maximum internal bounces before a ray is dropped |
| `scene.light_source.altitude` | yes | — | Sun altitude above horizon (degrees) |
| `scene.light_source.azimuth` | no | `0.0` | Sun azimuth (degrees) |
| `scene.light_source.diameter` | no | `0.5` | Sun angular diameter (degrees); `0.5` ≈ real Sun |
| `scene.light_source.spectrum` | yes | — | Either a CIE illuminant name (`"D65"`, …) or a `[{wavelength, weight}, …]` array |
| `crystal[].axis` | no | `{zenith: 90, azimuth: 0, roll: 0}` | "Horizontal" (column) orientation |
| `render[].lens.type` | no | `"linear"` | Other choices include the `fisheye_*` and `dual_fisheye_*` projections |
| `render[].lens.fov` | no | `90.0` | Diagonal FOV in degrees; ignored for `rectangular` and `dual_*` lenses |
| `render[].view.{azimuth,elevation,roll}` | no | `0.0` each | Camera direction relative to the world frame |
| `render[].visible` | no | `"upper"` | Hemisphere mask: `"upper"`, `"lower"`, or `"full"` |
| `render[].background` | no | `[0, 0, 0]` | RGB background |
| `render[].opacity` | no | `1.0` | Render opacity |

> All field names are exactly as accepted by the JSON parser. The C API mirrors a subset of these — see [`../c_api.md`](../c_api.md).

## 2. `ray_num` × wavelength semantics — what does "ray count" really mean?

**Short answer**: in discrete-spectrum mode (the typical case), `ray_num` is the number of rays traced **per wavelength**. Total work is `ray_num × N(wavelengths)`.

**Why this matters**: if you switch from one wavelength to a 9-band spectrum without changing `ray_num`, the simulation gets ~9× slower. New users routinely "tune" `ray_num` and then attribute the slowdown to a config bug. It is not a bug — it is the integration scheme.

**Source-anchored evidence** (`src/core/simulator.cpp:482-498`):

```cpp
const auto& spectrum = config.light_source_.spectrum_;
if (auto* illuminant = std::get_if<IlluminantType>(&spectrum)) {
  // Standard illuminant (e.g. D65): each batch picks one random wavelength
  float wl = 380.0f + rng_.GetUniform() * 400.0f;
  float weight = GetIlluminantSpd(*illuminant, wl);
  SimulateOneWavelength(config, WlParam{ wl, weight }, batch.ray_num_, ...);
} else {
  // Discrete wavelengths: trace ray_num for each
  const auto& wl_params = std::get<std::vector<WlParam>>(spectrum);
  for (const auto& wl_param : wl_params) {
    SimulateOneWavelength(config, wl_param, batch.ray_num_, ...);
  }
}
```

So the two spectrum modes have different cost profiles:

- **Discrete spectrum** (`spectrum: [{wavelength, weight}, ...]`): cost ≈ `ray_num × N(wavelengths)`. The bundled `examples/config_example.json` uses 9 bands × `ray_num=5e7` ⇒ 4.5 × 10⁸ rays.
- **Standard illuminant** (`spectrum: "D65"`, `"D50"`, …): cost ≈ `ray_num`. Each batch picks one wavelength uniformly from `[380, 780]` weighted by the SPD.

**`batch.ray_num_` clarified**: the server-side `GenerateScene` slices the user's `scene.ray_num` into smaller batches (`batch_ray_num = min(kDefaultRayNum, remaining)`) and dispatches them to the simulator. Each batch traces `batch.ray_num_` rays per wavelength; summing batches recovers the user-visible total. Code comment at `src/config/proj_config.hpp:28` confirms the semantics: `// For every single wavelength.`

**Practical advice**:

- First run? `ray_num=1e6` + a single wavelength (`[{"wavelength": 550, "weight": 1.0}]`) finishes in seconds.
- Need a low-noise final image? `ray_num=5e7` + the full discrete spectrum is the usual recipe.
- Need a continuous live preview in the GUI? Set `ray_num: "infinite"` and stop manually.

## 3. GUI vs JSON capabilities

The GUI wraps the same engine as the CLI but is **not a complete superset** of JSON config. Some things are JSON-only; others are GUI-only and not persisted.

| Feature | GUI | JSON / CLI | Notes |
|---------|-----|------------|-------|
| Multiple `render` entries | ❌ (only one preview) | ✅ | A JSON config can declare any number of `render: [...]` entries; CLI emits one image per entry. |
| Multi-layer scattering (≥ 2 `scattering` entries) | ❌ | ✅ | Recipe 3 in [`04-recipes.md`](04-recipes.md) requires JSON. |
| Lens projection switch | ✅ (Floating Lens Bar) | ✅ (`render[].lens.type`) | GUI re-projects on the fly; JSON sets it once per render entry. |
| Overlay grid | ✅ (live, GUI-only) | ❌ | The GUI overlay is a viewing aid and is **not** written into the saved `.lmc`. The JSON `render[].grid` field is accepted by the parser but is not consumed by the CLI render path — see §4. |
| Crystal preview style (wireframe / hidden line / x-ray / shaded) | ✅ | ❌ | Pure GUI state; not part of the simulation. |
| Save/load `.lmc` | ✅ | ✅ | The `.lmc` is plain JSON; both sides read the same file. |

Rule of thumb: **interactive exploration** in the GUI, **batch / reproducible runs** via JSON + CLI.

## 4. Field-naming caveats

A few JSON fields exist in the schema for forward compatibility but are not active in the current build. They are accepted by the parser and ignored by the engine, so they will not error — they just have no effect:

- `render[].grid` — the engine renders the simulation; grid overlays are added by the GUI on top of the rendered image at view time. The `grid` block in JSON is not used by the headless CLI render path.
- Some `crystal[].shape` distribution fields under unusual combinations — see the `known_mismatch.md` registry maintained alongside the documentation tasks for the current set.

If you hit "I set X in the JSON and nothing happened", check this list first, then [`../configuration.md`](../configuration.md), then file an issue.

## 5. Where do output files go?

- CLI: into the directory passed via `-o <dir>` (default: current working directory). One image per `render` entry, named `<render_id>.<format>` (default `jpg`, override with `--format png`).
- GUI: rendered images stay in memory until you `File ▶ Export` them. The GUI never writes to disk on its own.

## 6. "My halo looks faint / noisy / wrong" — checklist

1. **Faint?** Raise `scene.ray_num` (4× more rays = 2× cleaner image, roughly).
2. **Noisy?** Increase `scene.ray_num`, or narrow the wavelength band if the colour fringing dominates.
3. **No halo at all?** Verify the camera points at the sun: set `render[].view.elevation` ≈ `scene.light_source.altitude` and `view.azimuth` ≈ `light_source.azimuth`.
4. **Wrong shape?** Check `crystal[].axis` — random orientation gives circles; oriented plates give arcs and spots.
5. **Still stuck?** Re-run with `-v` and inspect the per-batch logs for unexpected zero-hit batches.

## Further reading

- Full schema → [`../configuration.md`](../configuration.md)
- Coordinate convention (altitude / zenith / azimuth signs) → [`../coordinate-convention.md`](../coordinate-convention.md)
- Performance tuning → [`../performance-testing.md`](../performance-testing.md)
- GUI panel reference → [`../gui-guide.md`](../gui-guide.md)
- C API for embedding → [`../c_api.md`](../c_api.md)
