# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed
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
