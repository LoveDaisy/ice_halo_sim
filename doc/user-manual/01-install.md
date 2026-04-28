[中文版](01-install_zh.md)

# Install and Build Lumice

This chapter walks you from a fresh clone to a working `Lumice` and `LumiceGUI` binary, plus a smoke test that confirms the toolchain is wired up correctly.

## Prerequisites

Lumice is a C++17 project built with CMake. You need the following on `PATH`:

| Tool | Minimum | Notes |
|------|---------|-------|
| C++ compiler | C++17 | Apple Clang ≥ 14, GCC ≥ 11, MSVC 2022 |
| CMake | 3.20 | `cmake --version` |
| Ninja | 1.10 | recommended generator (`ninja --version`) |
| Python | 3.9 | only required for E2E tests under `test/e2e/` |
| Qt 6 | 6.5 | optional, required only for the GUI binary |

External libraries (spdlog, nlohmann/json, stb, googletest, …) are fetched automatically via [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake) at configure time — you do not need to install them by hand.

## Build

The convenience script `scripts/build.sh` is the supported way to build:

```bash
# Release build, parallel, install artefacts under build/cmake_install/
./scripts/build.sh -j release
```

Other useful flavours (see `scripts/build.sh -h` for the full list):

```bash
./scripts/build.sh -tj release    # build + run unit tests
./scripts/build.sh -gtj release   # build + run GUI tests (needs a display server)
LUMICE_SKIP_GUI_TESTS=1 ./scripts/build.sh -gtj release   # skip GUI tests on a headless box
./scripts/build.sh -k release     # clean and rebuild
```

After a successful release build, the artefacts you care about live here:

| Artefact | Path | Purpose |
|----------|------|---------|
| CLI binary | `build/cmake_install/Lumice` | Run a JSON config from the command line |
| GUI binary | `build/cmake_install/LumiceGUI` | Interactive GUI app (requires Qt) |

> Debug builds land in `build/cmake_build/` instead. Release artefacts in `build/cmake_install/` are what the rest of this manual assumes.

![Lumice CLI startup banner](../figs/cli_screenshot_01.jpg)

## Smoke test

Run the bundled example config to confirm the binary works end-to-end. From the project root:

```bash
./build/cmake_install/Lumice -f examples/config_example.json -o /tmp/lumice-smoke
```

You should see:

1. A short banner identifying the build.
2. Per-batch progress lines while rays are traced.
3. A final `Stats: ...` block with the total simulated rays.
4. Four `.jpg` files written to `/tmp/lumice-smoke/`, named `example_img_01.jpg` … `example_img_04.jpg`.

![Lumice CLI completion output](../figs/cli_screenshot_02.jpg)

If you see all four images, the toolchain is healthy.

> The example config uses 9 spectral wavelengths × `ray_num=5e7`, so the simulator traces ~4.5 × 10⁸ rays. On a recent multi-core laptop this takes about 2 minutes. To get a faster smoke test, copy the example and lower `scene.ray_num` to `1e6`.

## Common build issues

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `cmake: command not found` | CMake not installed or not on `PATH` | Install via Homebrew / apt / official installer |
| Build fails downloading dependencies | First build with no CPM cache and no network | Re-run with network access; CPM caches under `~/.cache/CPM` |
| GUI fails at runtime with Qt error | Qt 6 not on the loader path | Either install Qt 6 system-wide, or build with `-DLUMICE_BUILD_GUI=OFF` |
| `LUMICE_SKIP_GUI_TESTS` ignored on CI | Stale cached config | `./scripts/build.sh -k release` to clean rebuild |

## Further reading

- Run your first config in the GUI → [`02-gui-quickstart.md`](02-gui-quickstart.md)
- Or run it from the CLI → [`03-cli-quickstart.md`](03-cli-quickstart.md)
- Build flag reference → [`../developer-guide.md`](../developer-guide.md)
