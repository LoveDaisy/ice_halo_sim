# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- **`grid.outline` now draws** (469.7): the render config's celestial-outline flag draws a line
  along the horizon (altitude 0) in CLI/core renders, where for four years it was parsed,
  validated, serialized and then ignored. The line is placed from the same per-pixel inverse
  projection the render-domain mask is built from — not a second copy of that math — and is
  clipped to the hemisphere `visible` admits, so a `visible: upper` render shows the horizon as
  the edge of its sky. Its width follows the local degrees-per-pixel (the preview shader's own
  rule), so it stays a couple of pixels across the whole lens/FOV range, and its colour is the
  GUI overlay's horizon red blended in linear RGB before the sRGB transfer curve. **The default
  changed to off** — see Changed below.
- **GUI background colour reaches the picture** (469.4): the preview, the three PNG exports
  (screenshot / dual-fisheye / equirectangular) and the frame baked into a saved `.lmc` now all
  paint the configured background colour behind the halo. Previously only the CLI did, so the GUI
  colour picker moved and nothing on screen changed. The colour is composited additively in linear
  RGB before the sRGB transfer curve, which makes a pixel carrying no halo energy render as exactly
  the sRGB triple the picker showed; it is painted only where the lens actually images sky, so the
  black surround outside a fisheye's image circle stays black. Expect halo-against-sky contrast to
  drop against a bright background — that is what the sRGB curve does, and EV is its control.
- **Raypath-colour (composite) display honours the background colour** (469.5): with raypath
  colouring on, the picture now carries the same background as with it off. Previously the
  composite was baked server-side with a black surround, so toggling colouring changed the
  background out from under the user — the visible inconsistency this closes. The colour is added
  in linear RGB after all exposure handling and before the sRGB transfer curve, only on the pixels
  the lens actually images, so it agrees byte-for-byte with the mono path outside the halo and
  leaves the region outside a fisheye's image circle black.
- **One new C API setter** (ABI addition, non-breaking):
  `LUMICE_SetCompositeBackground(server, background_linear)` — a display-time push of the
  composite path's additive linear-RGB background, shaped exactly like `LUMICE_SetCompositeExposure`
  (no epoch bump, no accumulator reset, no re-simulation; the next acquired result frame re-bakes
  the composite). All-zero, the default, is an algebraic no-op, so a consumer that never calls it
  sees byte-identical composites.
- **One new C API pure function** (ABI addition, non-breaking):
  `LUMICE_XyzToSrgbUint8WithBackground(xyz_in, out, pixel_count, intensity_scale, background_linear)`
  — the existing `LUMICE_XyzToSrgbUint8` with an additive linear-RGB background composited before
  the final clamp and gamma, for a consumer baking a frame that has to match what the renderer put
  on screen. The inverse sRGB transfer curve a caller needs to convert a picker colour into that
  `background_linear` argument (or into `LUMICE_RenderParam::background`) stays a C++-only inline
  function, `lumice::SrgbToLinearRgb` in `src/util/color_space.hpp` — no new public C API for it.
- **GUI custom discrete-spectrum editor** (task-323): the Sun panel Spectrum combo now
  offers a "Custom..." entry that opens a wavelength/weight list editor. Custom spectra are
  persisted in `.lmc` files and core JSON configs.

### Changed
- **`render[].grid.outline` now defaults to `false`** (469.7). It defaulted to `true` for as long
  as it existed, which cost nothing while nothing drew it; now that it draws, leaving it on would
  put a horizon line into every existing config that never asked for one. Add `"outline": true` to
  a renderer to get the line back. Turning an annotation on for every render is a product decision
  nobody has made, so the default states the one thing that is certain: draw it when asked.
- **`render[].grid.central` / `grid.elevation` documented as not rendered** (469.7). Both keys are
  still parsed, validated and round-tripped, and no code draws either — they are now labelled that
  way in `doc/configuration.md` (and `_zh`) instead of sitting in the same table as the keys that
  do something, and the shipped `examples/config_example.json` no longer demonstrates a 22 deg
  circle that never appears in the output.
- **`crystal_num` / `Stats: crystals=N` redefined** (no ABI change — same field name and
  type): the value is now **how many distinct crystal geometries the run actually sampled**,
  not how many crystal objects it built. A scene with no random shape distributions reports
  exactly its (scattering layer × entry) count regardless of `ray_num`; give a shape a
  distribution and the count rises with the geometries actually drawn. Previously the value
  tracked the batch schedule instead of the scene — sweeping `LUMICE_DISPATCH_RAY_NUM` alone
  moved it by two orders of magnitude, and a randomized scene reported the same number as its
  fixed-shape twin. **Expect the reported number to drop sharply** for fixed-shape scenes
  (e.g. 785 → 5 on a 5-population 20k-ray scene, or 60 → 5 on the default multi-worker CLI);
  that is the fix, not a regression. The value is now independent of `num_workers` and of the
  dispatch grain — the fixed-shape part of a scene is counted once from the committed config
  rather than once per worker per batch — but remains non-comparable across backends (CPU
  samples per ray-group, the GPU K-shape clock is off by default). See `doc/c_api.md` and the
  contract block on `TraceBackend::GetLastBatchStochasticCrystalSampleCount`.
- **Breaking config change (discrete spectra)**: `ray_num` now means the TOTAL number of
  rays traced across ALL wavelengths (previously it was per-wavelength). The server derives
  the per-wavelength budget via `ceil(ray_num / n_wavelengths)`. Externally hand-written
  discrete-spectrum configs must multiply their old `ray_num` by the wavelength count to
  preserve the previous total. Single-wavelength / illuminant configs are unaffected.
- Adaptive Brightness no longer has an ON/OFF toggle. The simulator always uses the
  F1 anchor lane introduced by scrum-221; the GUI Display panel now shows
  `(+N.NN EV auto)` next to the manual EV slider with no checkbox. Filter switches
  no longer jump the EV.
- **Adaptive Brightness defaults updated**: EV anchor now uses the P99.5 percentile
  (previously P99) and maps to `target_white = 135` (previously 200). These values
  produce better perceptual balance across typical halo scenes.
- **Anchor lane removed**: Adaptive Brightness now uses per-frame visible-framebuffer
  self-P99.5 normalization. Filter-fail rays terminate immediately in `CollectData`
  (Design A semantics). The EV scale may shift when toggling a filter, since the
  P99.5 is computed over the current visible set. Filter early-kill is fully
  restored: at `ms_prob=0.5` filter-on runs +74% faster than filter-off (multi-worker,
  macOS); at `ms_prob=0.8` it is +114% faster. Under the prior F1 anchor lane this
  speedup was lost because filter-failed rays still completed full multi-scattering
  trajectories. See `scratchpad/task-remove-anchor-lane/bench/bench_results.md`.

### Removed
- **Breaking ABI #4**: `LUMICE_RenderParam::opacity` removed, and `render[].opacity` is no longer
  parsed from JSON (`RenderConfig::opacity_` deleted). The field had no drawing consumer anywhere
  in the tree since the first commit — setting it changed nothing — and it has no counterpart in
  the GUI, so nothing was ever going to grow into it. `LUMICE_API_VERSION` is bumped to 416.
  Drop the assignment from C/FFI callers; old JSON configs keep loading (unknown keys are
  ignored). Note this is a DIFFERENT field from `LUMICE_GridLine::opacity`, which is untouched.
  The GUI's mirror of the same field (`renderer.opacity`, editable in the Settings panel and
  persisted in `.lmc`) is removed with it; a `.lmc` written by an older build still opens.
- **Breaking ABI #3**: `LUMICE_RawXyzResult::anchor_p995_y` and `anchor_snapshot_intensity`
  fields removed. Struct shrinks from 64 bytes to 56 bytes on 64-bit platforms.
  Update all ctypes / FFI / C callers that reference these fields.
  (#1 was the removal of `ab_mode` in task-remove-adaptive-brightness-on-mode;
  #2 was the `anchor_p99_y` → `anchor_p995_y` rename in apply-new-defaults.)
- `RaySeg::is_filter_dropped_` and `is_prior_filter_failed_` fields and the
  `IsFilterDropped()` helper removed. `IsOutgoing()` simplified accordingly.
- `SimData::anchor_d_` and `anchor_w_` fields removed (sizeof: 216 → 168 bytes on
  Apple Silicon libc++).
- `RenderConsumer` anchor accumulators (`anchor_internal_xyz_`, `anchor_snapshot_xyz_`,
  `anchor_total_intensity_`, `anchor_snapshot_intensity_`, `anchor_p995_y_`) removed.
- `test_partition_buffer_additivity` (`test/e2e/test_additivity.py`) and its 9 config
  files removed; the additivity invariant only held under the anchor-lane normalization.
- **Breaking ABI #2**: `LUMICE_RawXyzResult::anchor_p99_y` renamed to `anchor_p995_y`.
  Update all ctypes / FFI / C callers that reference this field by name.
  (ABI break #1 was the removal of `ab_mode` in task-remove-adaptive-brightness-on-mode.)
- **Breaking ABI**: `LUMICE_RenderParam::ab_mode` field removed from the C API.
  Callers should drop the assignment; behavior matches the prior OFF mode (F1).
- `AdaptiveBrightnessMode` enum (`src/config/render_config.hpp`) removed.
- `RenderConfig::ab_mode_`, `SimBatch::ab_mode_`, `GuiState::auto_ev_enabled`,
  and the GUI-local `AdaptiveBrightnessMode` mirror enum removed.
- `adaptive_brightness.mode` JSON config key is no longer parsed (nlohmann ignores
  unknown keys silently, so old configs load without error but the field is a
  no-op).

## [4.1.3] - 2026-03-17

### Fixed
- GUI no longer stutters during long simulations (moved server polling to background thread)
- Unicode superscript characters replaced with ASCII in ray number display

## [4.1.2] - 2026-03-16

### Fixed
- Windows exe now statically links GCC runtime (no more `libgcc_s_seh-1.dll` / `libwinpthread-1.dll` missing errors)
- Release workflow `if` conditions fixed for macOS signing secrets

## [4.1.1] - 2026-03-16

### Fixed
- macOS deployment target set to 13.0 (was defaulting to runner OS version 15.0)
- macOS release binaries now code-signed and notarized (Developer ID + Apple notarization)
- `version.py` encoding fix for Windows (`utf-8` explicit)
- Git LFS checkout enabled for CI/Release GUI builds and E2E tests

## [4.1.0] - 2026-03-15

### Added
- Application icon for macOS and Windows (generated from source PNG)
- macOS `.app` bundle with `Info.plist` and icon (Finder-friendly)
- Windows GUI subsystem (hidden console) with embedded icon resource
- Icon generation script (`scripts/generate_icons.sh`)
- Windows CI and Release support (GitHub Actions)
- Preview aspect ratio presets, image export, background overlay, ray number formatting

## [4.0.0] - 2026-03-12

### Added
- GUI application (Dear ImGui + GLFW + OpenGL) with crystal preview, simulation control, and render preview
- `.lmc` binary file format for GUI project save/load
- GUI automated tests (ImGui Test Engine) with visual regression
- GitHub Actions CI pipeline (Ubuntu x64/ARM64, macOS ARM64, Windows x64)
- Release workflow with automated packaging on tag push
- Linux OpenGL support for GUI build
- `version.py` script for version consistency checking and management
- `.editorconfig`, PR/Issue templates, Dependabot configuration
- `CONTRIBUTING.md` with development workflow documentation

### Changed
- Project version updated from 2.1.1 to 4.0.0 (aligning with git tags)

### Fixed
- `asin()` NaN causing white pixel artifacts on ARM64
- Crystal preview coordinate transform (Core Z-up to screen Y-up)
- Various GUI interaction and rendering fixes

## [3.4.1] - 2026-03-09

### Added
- Fisheye projection support (equidistant, equisolid, stereographic, orthographic)
- E2E test infrastructure under `test/e2e/`

### Fixed
- `asin()` input clamping to prevent NaN pixel artifacts
- FOV consistency: corrected f→fov conversion per projection model

## [3.4.0] - 2026-03-02

### Changed
- Simplified JSON configuration format: flattened scene/light/project structure
- Scattering config restructured from parallel arrays to entry-based format
- `ray_num` uses `"infinite"` string instead of `-1`
- Removed unused `StreetLightParam` and `view.distance` config

## [3.3.0] - 2026-02-24

### Added
- Performance optimization: deterministic crystal caching and buffer reuse
- `BM_SimLoop` single-thread benchmark

### Changed
- Deleted `matlab/` directory, promoted `cpp/` contents to root
- Dual-language README (English + Chinese)

### Fixed
- `CommitConfig` race condition causing benchmark SIGSEGV
- Documentation corrections across `doc/` and README

## [3.2.0] - 2026-02-17

### Changed
- Project renamed from IceHalo to **Lumice** (namespace, CMake, C API, documentation)
- C API prefix changed from `HS_*` to `LUMICE_*`, header from `icehalo.h` to `lumice.h`
- Replaced raw `new` with `std::make_unique` throughout codebase
- Refactored logging system to instance-level logger control via spdlog

## [3.1.0] - 2026-02-10

### Changed
- Eliminated `FOR_TEST`, `RANDOM_SEED`, `MULTI_THREAD` compile switches
- Unified test infrastructure (removed `icehalo_test_lib`)
- Migrated dependencies to CPM.cmake with URL-based downloads

### Removed
- Deprecated benchmark files

## [3.0.0] - 2025-12-19

### Added
- C API documentation and developer guide
- Doxygen configuration for API documentation

### Changed
- Major codebase modernization: CMake build system cleanup, dependency management
- V3 rewrite with clean namespace structure

## [2.3.0] - 2021-09-29

### Changed
- Server architecture improvements
- Configuration parsing enhancements

## [2.2.0] - 2021-03-26

### Changed
- Rendering pipeline updates

## [2.1.2] - 2020-02-16

### Fixed
- Minor bug fixes

## [2.1.1] - 2020-01-27

### Fixed
- Minor bug fixes and stability improvements

## [2.1] - 2019-03-25

### Added
- Multi-scattering support improvements

## [2.0] - 2019-03-11

### Added
- Multi-scattering simulation support
- Server/client architecture

### Changed
- Major architecture rewrite

## [1.2] - 2019-03-02

### Added
- Natural color rendering based on spectrum simulation

## [1.1] - 2019-02-19

### Added
- Configuration file support

## [1.0] - 2019-02-16

### Added
- Initial release
- Basic ice crystal halo simulation
- Support for common crystal types (hexagonal prism, plate, column)
