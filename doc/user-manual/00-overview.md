[中文版](00-overview_zh.md)

# Lumice User Manual — Overview

Welcome to the Lumice user manual. Lumice is a C++17 ice halo ray-tracing simulator: it traces light through ice crystals to reproduce halo patterns you might see around the sun on a cold day.

This manual is the **getting-started entry point**. It is organised by user journey, not by reference topic. If you want a complete configuration schema, internal architecture, or developer-side details, follow the "Further reading" links at the end of each chapter back into the technical references in `doc/`.

> **Goal**: a new user, starting from a clean checkout, should reach their first rendered halo image in under 30 minutes by following one of the journeys below.

## User journeys

Pick the path that matches your goal. Each path is self-contained — you do not need to read the others first.

```
   ┌────────────┐     ┌──────────────────┐     ┌─────────────────┐
   │ 01-install │ ──▶ │ 02-gui-quickstart│ ──▶ │   04-recipes    │
   └────────────┘     │ (interactive)    │     │ (reproduce      │
        │             └──────────────────┘     │  classic halos) │
        │                                      └─────────────────┘
        │             ┌──────────────────┐               ▲
        └───────────▶ │ 03-cli-quickstart│ ──────────────┘
                      │ (batch / JSON)   │
                      └──────────────────┘
                              │
                              ▼
                      ┌──────────────────┐
                      │ 05-faq           │
                      │ (defaults, GUI   │
                      │  vs JSON, edge   │
                      │  cases)          │
                      └──────────────────┘
```

- **GUI-first** (recommended for first-time users): `01-install` → `02-gui-quickstart` → `04-recipes`.
- **CLI-first** (recommended for batch / scripted runs): `01-install` → `03-cli-quickstart` → `04-recipes`.
- **Stuck?** Jump to `05-faq` for default values, "GUI vs JSON" capability differences, and known limitations.

## What this manual is not

This manual does **not** replicate the technical references already in `doc/`. For deep dives, see:

| Topic | Reference |
|-------|-----------|
| Full JSON configuration schema | [`configuration.md`](../configuration.md) |
| Coordinate system conventions | [`coordinate-convention.md`](../coordinate-convention.md) |
| GUI internals and panel reference | [`gui-guide.md`](../gui-guide.md) |
| C API for embedding | [`c_api.md`](../c_api.md) |
| Performance tuning and benchmarks | [`performance-testing.md`](../performance-testing.md) |
| System architecture | [`architecture.md`](../architecture.md) |

## Conventions in this manual

- All shell commands assume you are at the project root (the directory containing `CMakeLists.txt`).
- Paths inside the manual use `../` to reach `doc/` siblings (e.g. `../figs/<image>.jpg`).
- Code blocks are tested against the build artefact at `build/cmake_install/Lumice` produced by `./scripts/build.sh -j release`.
- Where a fact is anchored to source code, the source line is cited (e.g. `src/core/simulator.cpp:482-498`).

## Further reading

- Next, install Lumice → [`01-install.md`](01-install.md)
- All technical references → [`../README.md`](../README.md)
