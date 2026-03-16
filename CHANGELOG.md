# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- macOS deployment target set to 13.0 (was defaulting to runner OS version 15.0)
- macOS release binaries now code-signed and notarized (Developer ID + Apple notarization)

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
