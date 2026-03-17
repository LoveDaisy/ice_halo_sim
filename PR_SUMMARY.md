# PR: GPU Tone Mapping & Live Parameter Adjustment

## Overview

Two interconnected features that significantly improve the interactive preview experience:

1. **GPU Tone Mapping** — Move XYZ→sRGB color conversion from CPU to fragment shader, enabling real-time preview updates
2. **Live Parameter Adjustment** — Edit simulation parameters (crystals, sun, scattering, filters) while the simulation is running, with instant visual feedback

---

## Feature 1: GPU Tone Mapping

### Problem

The CPU-side `RenderConsumer::GetResult()` performed XYZ→sRGB conversion on every poll, creating a bottleneck that limited preview refresh to ~1fps.

### Solution

Expose raw XYZ float data via a new API path and perform the color conversion in a GLSL fragment shader.

### Changes

#### New Raw Data API (Core/Server)

| File | Change |
|------|--------|
| `render.hpp` / `render.cpp` | Added `GetRawResult()` — returns raw XYZ float pointer + total intensity without conversion |
| `server.hpp` / `server.cpp` | Added `RawRenderResult` struct, `GetRawRenderResults()` with snapshot-under-lock |
| `lumice.h` / `c_api.cpp` | Added `LUMICE_RawRenderResult` and `LUMICE_GetRawRenderResults()` C API |

#### GPU Rendering Pipeline (GUI)

| File | Change |
|------|--------|
| `server_poller.hpp` / `.cpp` | `PollerData::texture_data` → `vector<float>`, added `texture_intensity`, poll interval 1s → 33ms |
| `preview_renderer.hpp` / `.cpp` | `UploadTexture(const float*)` with `GL_RGB32F`, new fragment shader with `toneMapXyzToSrgb()` |
| `gui_state.hpp` | Added `last_intensity` field |
| `app.cpp` | Updated `SyncFromPoller` and `RenderPreviewPanel` to pass intensity uniforms |
| `file_io.cpp` | Disabled `.lmc` texture embedding (raw XYZ can't be PNG-encoded) |

#### Fragment Shader Tone Mapping

The shader's `toneMapXyzToSrgb()` replicates the CPU `GetResult()` pipeline:
1. Normalize by `intensity_factor / total_intensity × 1e5`
2. Gamut map via D65 white point desaturation (always spectral/real-color)
3. XYZ → linear sRGB matrix multiply
4. Apply `ray_color` tint + background
5. Clamp + sRGB gamma correction

> **Note:** The Core always produces spectral XYZ data (`ray_color = {-1,-1,-1}`). The shader always does real-color gamut mapping and applies the GUI's `ray_color` as a post-conversion tint. This differs from the old CPU path where `ray_color[0] < 0` was a mode switch.

### Known Limitation

`.lmc` file texture embedding is disabled — raw XYZ data can't be saved as PNG. Old `.lmc` files with embedded textures load config but skip the texture. Re-run simulation to regenerate preview.

---

## Feature 2: Live Parameter Adjustment

### Problem

All parameter panels (Crystal, Scene, Filter) were disabled during simulation, requiring stop → edit → re-run for every change.

### Solution

Enable panels during simulation and auto-restart on every parameter change, giving a "scrub a slider and watch the halo morph" experience.

### Changes

| File | Change |
|------|--------|
| `panels.cpp` | Removed `BeginDisabled`/`EndDisabled` guards from Crystal, Scene, and Filter tabs |
| `main.cpp` | Added auto-restart in main loop: if `dirty && (kSimulating \|\| kDone \|\| kModified)` → `DoStop(); DoRun();` |

The auto-restart fires every frame the config changes — no debounce. During slider drag, the simulation restarts per-frame, producing a dim but real-time preview. Once the slider is released, the simulation runs uninterrupted and the preview fills in.

> **Design note:** `kDone` and `kModified` states are also handled because transient state transitions during rapid restarts could break the restart chain (see bug fix below).

---

## Bug Fixes

### 1. `GetStatus()` Startup Race → False Idle

**Symptom:** After rapid slider scrubbing, simulation stops ("Done") but CPU stays at 100%.

**Root cause:** `GetStatus()` has a race window between `work_started_ = true` and the first `scene_queue_->Emplace()`. During this gap, `sim_scene_cnt_ == 0` and all simulators are idle → falsely returns `kIdle`.

**Fix in `server.cpp`:**
- `Start()` initializes `sim_scene_cnt_ = 1` (sentinel)
- `GenerateScene()` consumes the sentinel only after its first real `Emplace`
- `GetStatus()` never sees `sim_scene_cnt_ == 0` during startup

### 2. Use-After-Free on Restart

**Symptom:** Crash during rapid slider scrubbing.

**Root cause:** In the `kDone`/`kModified` auto-restart path, `DoRun()` was called without `DoStop()`. The poller thread could still be calling `LUMICE_GetRawRenderResults()` while `CommitConfig` was destroying consumers.

**Fix in `main.cpp`:** Always call `DoStop()` before `DoRun()` to join the poller thread first.

---

## Files Changed (15 total)

| Layer | Files |
|-------|-------|
| **Core** | `render.hpp`, `render.cpp`, `server.hpp`, `server.cpp`, `lumice.h`, `c_api.cpp` |
| **GUI** | `server_poller.hpp`, `server_poller.cpp`, `preview_renderer.hpp`, `preview_renderer.cpp`, `gui_state.hpp`, `app.cpp`, `file_io.cpp`, `main.cpp`, `panels.cpp` |
