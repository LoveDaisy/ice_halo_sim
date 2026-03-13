# End-to-End Tests

E2E tests verify the full simulation pipeline: config parsing → ray tracing → image output.
They run the built `Lumice` binary with test configurations and check results.

## Prerequisites

```bash
# Build Lumice (release)
./scripts/build.sh -j release

# Install Python test dependency
pip install Pillow
```

The tests expect the binary at `build/cmake_install/Lumice`. Override with `LUMICE_BIN` env var.

## Running Tests

```bash
# All E2E tests
pytest test/e2e/ -v

# Individual test modules
pytest test/e2e/test_cli.py -v       # CLI behavior
pytest test/e2e/test_smoke.py -v     # Smoke tests (all configs)
pytest test/e2e/test_errors.py -v    # Error handling
```

## Test Structure

```
test/e2e/
├── README.md           # This file
├── __init__.py
├── runner.py           # Binary discovery and execution
├── base.py             # LumiceTestCase base class
├── image_utils.py      # JPEG dimensions, MSE/PSNR computation (Pillow)
├── test_cli.py         # CLI behavior: -h, no args, -o output dir
├── test_smoke.py       # Smoke: exit code, output files, dimensions, PSNR
├── test_errors.py      # Error handling: invalid configs, missing fields
├── configs/            # Test configuration files (9 scenarios)
│   ├── halo_22.json    # 22° halo — random prism, D65
│   ├── parhelion.json  # Parhelion — horizontal prism, 550nm
│   ├── cza.json        # Circumzenithal arc — plate crystal, illuminant A
│   ├── pyramid.json    # Pyramid crystal halo, D50
│   ├── color.json      # Multi-wavelength (440-640nm), parhelion-style
│   ├── multi_lens.json # 3 renderers: linear + fisheye + dual_fisheye
│   ├── render_opts.json# Render options: visible, background, grid
│   ├── filters.json    # Raypath filter [3,5] with PBD symmetry
│   ├── multi_scatter.json # Two-layer scattering, D75
│   └── error/          # Invalid configs for error tests
│       ├── invalid_json.json
│       ├── missing_scene.json
│       ├── missing_render.json
│       └── missing_crystal.json
└── references/         # Reference images for PSNR comparison
    └── *.jpg           # Tracked in git (~500KB total)
```

## Reference Images

Reference images are used by `test_image_psnr` to verify that simulation output
is consistent with known-good baselines via PSNR (Peak Signal-to-Noise Ratio).

Reference images are tracked in git (`test/e2e/references/*.jpg`).
To regenerate after rendering code changes:

```bash
for cfg in test/e2e/configs/*.json; do
    name=$(basename "$cfg" .json)
    tmpdir=$(mktemp -d)
    ./build/cmake_install/Lumice -f "$cfg" -o "$tmpdir"
    for img in "$tmpdir"/img_*.jpg; do
        rid=$(echo "$img" | grep -o '[0-9]*\.jpg' | sed 's/\.jpg//')
        cp "$img" "test/e2e/references/${name}_$(printf '%02d' $rid).jpg"
    done
    rm -rf "$tmpdir"
done
```

### Behavior without reference images

- `test_image_psnr` is **skipped** (not failed) when reference images are absent.
- All other tests (exit code, file existence, dimensions, error handling) run normally.
- If Pillow is not installed, image dimension checks are also skipped; basic tests still run.

## PSNR Thresholds

Thresholds are calibrated by running each config 3 times and recording the minimum
PSNR against the reference. The threshold is set to `min_psnr - 3 dB`.

At `ray_num = 10,000,000` (256×256 resolution), PSNR variance is < 0.3 dB across
runs, so the 3 dB margin provides a very stable test.

| Output            | Min PSNR (dB) | Threshold (dB) |
|-------------------|---------------|-----------------|
| halo_22_01        | 21.0          | 18.0            |
| parhelion_01      | 28.7          | 25.7            |
| cza_01            | 38.1          | 35.1            |
| pyramid_01        | 21.5          | 18.5            |
| color_01          | 21.4          | 18.4            |
| multi_lens_01     | 28.6          | 25.6            |
| multi_lens_02     | 29.3          | 26.3            |
| multi_lens_03     | 41.5          | 38.5            |
| render_opts_01    | 25.6          | 22.6            |
| filters_01        | 24.0          | 21.0            |
| multi_scatter_01  | 20.8          | 17.8            |
