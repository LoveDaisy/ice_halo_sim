[中文版](03-cli-quickstart_zh.md)

# CLI Quickstart

This chapter shows how to run Lumice from the command line — useful for batch jobs, CI, headless servers, and reproducible recipes.

> **Prerequisite**: `build/cmake_install/static/Lumice` exists. If not, see [`01-install.md`](01-install.md).

## 1. Basic invocation

The minimal call is:

```bash
./build/cmake_install/static/Lumice -f examples/config_example.json
```

This writes JPEG output(s) to the current working directory. To pick an output directory:

```bash
./build/cmake_install/static/Lumice -f examples/config_example.json -o /tmp/lumice-out
```

![Lumice startup](../figs/cli_screenshot_01.jpg)

## 2. Reading the output

After the run, the output directory contains one image per render entry in your config. The bundled example defines four render entries, so you get four files:

| File | Lens / view | Notes |
|------|-------------|-------|
| `example_img_01.jpg` | Equal-area dual fisheye, full sky | The default "everything in the sky" view |
| `example_img_02.jpg` | Linear lens, narrower FOV | Closer to a camera-with-fisheye-removed view |
| `example_img_03.jpg` | Equidistant fisheye | Useful for angle measurement |
| `example_img_04.jpg` | Stereographic fisheye | Preserves circle shapes near the horizon |

![Example output 1](../figs/example_img_01.jpg)
![Example output 2](../figs/example_img_02.jpg)
![Example output 3](../figs/example_img_03.jpg)
![Example output 4](../figs/example_img_04.jpg)

The console also prints a `Stats:` block summarising the simulation (ray counts, elapsed time, per-wavelength accumulation). Capture this if you want a reproducibility receipt.

Two streams, on purpose: the product lines — `Saved:` / `Stats:` here, the `[BENCHMARK]` JSON of `benchmark`, the CSV of `analyze` — go to **stdout**, and every diagnostic log line (`-v` and the engine's own messages alike) goes to **stderr**. So `Lumice -f config.json -o out 2>/dev/null` prints exactly the product lines, and `> run.log 2>&1` keeps both together when you want the receipt and the log in one file.

## 3. Verbose and debug modes

```bash
./build/cmake_install/static/Lumice -f config.json -v   # trace-level logs
./build/cmake_install/static/Lumice -f config.json -d   # debug-level logs
```

`-v` is roughly "show me the per-batch ray counts as they are produced"; `-d` adds extra diagnostics (RNG seeds, scattering layer dispatch). Use `-v` first; reach for `-d` only when chasing a bug.

## 4. All flags at a glance

The CLI has three subcommands, `render`, `benchmark` and `analyze`. `render` is the default: `Lumice -f config.json` and `Lumice render -f config.json` are the same command, so every example above is a render. Each subcommand accepts only its own options; `Lumice <subcommand> -h` prints that subcommand's page.

The complete set as printed by `Lumice -h` (anchor source: `./build/cmake_install/static/Lumice -h`):

```text
Usage: ./build/cmake_install/static/Lumice [render] -f <config_file> [options]
       ./build/cmake_install/static/Lumice benchmark -f <config_file> [options]
       ./build/cmake_install/static/Lumice analyze -f <config_file> [options]
       ./build/cmake_install/static/Lumice <subcommand> -h

Lumice — simulate ice halos by tracing rays through ice crystals.

Subcommands:
  render             Simulate and write halo images. This is the default when
                     no subcommand is given: `./build/cmake_install/static/Lumice -f ...` is a render.
  benchmark          Run a throughput benchmark and print [BENCHMARK] JSON
                     (`./build/cmake_install/static/Lumice benchmark -h` for its options)
  analyze            List the raypath chains that light a region of the sky, as CSV
                     (`./build/cmake_install/static/Lumice analyze -h` for its options)

Options for render (the default subcommand):
  -f <file>          Specify the configuration file (required)
  -o <dir>           Output directory for rendered images (default: current directory)
  --format <fmt>     Output image format: jpg or png (default: jpg)
  --quality <1-100>  JPEG quality (default: 95, ignored for PNG)
  --backend <name>   Trace backend: auto, cpu, metal, or cuda (default: auto).
                     'auto' and 'cpu' both select the CPU route today; 'metal'
                     falls back to CPU if unavailable. The LUMICE_TRACE_BACKEND
                     env var, if set, still overrides this (debug/CI only).
  --workers <N>      Number of CPU simulation worker threads (default: automatic —
                     one per physical core, capped at a ceiling above which no
                     machine measured ran faster; an explicit N is never capped).
                     Machine-dependent, so it is a command-line switch rather than
                     a config-file field: a config travels between machines and a
                     worker count should not travel with it. Ignored on a GPU route
                     (single engine).
  -v                 Verbose output (trace level logging)
  -d                 Debug output (debug level logging)
  -h, --help         Show this help message and exit

Examples:
  ./build/cmake_install/static/Lumice -f config.json
  ./build/cmake_install/static/Lumice -f config.json -o /tmp/output
  ./build/cmake_install/static/Lumice -f config.json --format png
  ./build/cmake_install/static/Lumice -f config.json --quality 80
  ./build/cmake_install/static/Lumice -f config.json --backend metal
  ./build/cmake_install/static/Lumice -f config.json --workers 4
  ./build/cmake_install/static/Lumice -f config.json -v
  ./build/cmake_install/static/Lumice benchmark -f examples/bench_config.json
  ./build/cmake_install/static/Lumice analyze -f config.json --roi cone --center 43,0 --radius 2
```

`Lumice benchmark -h`:

```text
Usage: ./build/cmake_install/static/Lumice benchmark -f <config_file> [options]

Run a throughput benchmark and print [BENCHMARK] JSON. The legacy CPU route
runs a dual pass (single-worker + multi-worker → per-core and parallel-
efficiency data); a GPU route is single-engine, so it runs one steady pass
only (single/multi would not be parallel). The worker counts are part of the
measurement methodology, which is why there is no --workers here; nothing is
written to disk, which is why there is no -o.

Options:
  -f <file>          Specify the configuration file (required)
  --backend <name>   Trace backend: auto, cpu, metal, or cuda (default: auto).
                     'auto' and 'cpu' both select the CPU route today; 'metal'
                     falls back to CPU if unavailable. The LUMICE_TRACE_BACKEND
                     env var, if set, still overrides this (debug/CI only).
  -v                 Verbose output (trace level logging)
  -d                 Debug output (debug level logging)
  -h, --help         Show this help message and exit

Examples:
  ./build/cmake_install/static/Lumice benchmark -f examples/bench_config.json
  ./build/cmake_install/static/Lumice benchmark -f examples/bench_config.json --backend metal
```

`Lumice analyze -h` — the raypath analysis the GUI's Raypath Analysis window runs (see [`06-raypath-analysis.md`](06-raypath-analysis.md) for what the result means), from the command line, as the same CSV that window exports:

```text
Usage: ./build/cmake_install/static/Lumice analyze -f <config_file> [options]

Trace the scene and list the raypath chains that delivered energy into a region
of the sky, most energetic first, as CSV — the same file the GUI's Raypath
Analysis window exports. The config is the scene; the question asked of it is
given by the options below and never read from the config. The analysis always
traces on the CPU (the chain record exists on that route only).

Output: the CSV goes to stdout, or to --csv <path> instead (never both). A `#`
head names the region, the symmetry, the ray total and the export time; then
one row per chain: Raypath, Energy (% of the total), Cumulative %, +/- (the
row's 1/sqrt(count) relative noise, in %). Progress goes to stderr, one line per
second. Ctrl-C ends the run early and still writes the result accumulated so far
(exit 0) — which is how a scene whose ray_num is "infinite" is meant to be run.
With --csv the file is rewritten atomically every second, so it is complete at
any moment it is read.

Options:
  -f <file>          Specify the configuration file (required)
  --roi <region>     Which rays count: sky (every outgoing ray; default), frame (the
                     rays that land inside one of the config's render[] frames), or
                     cone (the rays within --radius of --center).
  --render-id <id>   frame only: the render[] entry whose lens / view / visible /
                     front / resolution define the frame (default: the first entry).
  --center <alt>,<az>
                     cone only (required): the cone's centre as the altitude and
                     azimuth, in degrees, of the sky point — azimuth measured as the
                     sun's is, so the sun sits at --center <sun_altitude>,0.
  --radius <deg>     cone only (required): the cone's angular radius in degrees.
  --symmetry <spec>  Merge chains that are the same path up to crystal symmetry when
                     listing: any combination of P, B, D (case-insensitive) or
                     `none` (default: PBD). Changes the grouping, never the totals.
  --rays <N>         This run's ray budget, total across wavelengths; N may carry a
                     K, M or G suffix (e.g. 20M). Default: the scene's own ray_num,
                     including "infinite".
  --seed <N>         Fix the simulation's random seed (a positive integer) so two
                     runs of one question are the same run; this also sizes the pool
                     to one worker (a seeded run is single-threaded by contract).
                     Default: random.
  --csv <path>       Write the CSV to this file instead of stdout.
  --backend <name>   Trace backend: auto, cpu, metal, or cuda (default: auto).
                     'auto' and 'cpu' both select the CPU route today; 'metal'
                     falls back to CPU if unavailable. The LUMICE_TRACE_BACKEND
                     env var, if set, still overrides this (debug/CI only).
  --workers <N>      Number of CPU simulation worker threads (default: automatic —
                     one per physical core, capped at a ceiling above which no
                     machine measured ran faster; an explicit N is never capped).
                     Machine-dependent, so it is a command-line switch rather than
                     a config-file field: a config travels between machines and a
                     worker count should not travel with it. Ignored on a GPU route
                     (single engine).
  -v                 Verbose output (trace level logging)
  -d                 Debug output (debug level logging)
  -h, --help         Show this help message and exit

Examples:
  ./build/cmake_install/static/Lumice analyze -f config.json
  ./build/cmake_install/static/Lumice analyze -f config.json --roi cone --center 43,0 --radius 2
  ./build/cmake_install/static/Lumice analyze -f config.json --roi frame --render-id 1 --csv frame.csv
  ./build/cmake_install/static/Lumice analyze -f config.json --symmetry none --rays 5M --seed 7
```

Notes:

- `-f` is the only required flag. Without it, Lumice exits non-zero with a usage hint.
- `--format png` switches to lossless PNG; `--quality` is ignored in that case.
- `--workers <N>` overrides the automatic worker count (one per physical core, capped at a measured ceiling; the cap applies to the automatic value only, never to an `N` you name). It is a switch rather than a config field on purpose: a worker count describes the machine, and a config file travels between machines. An illegal value (`0`, negative, non-numeric) exits non-zero rather than falling back to the default.
- `Lumice analyze -f <config>` asks a question of the scene rather than rendering it: which raypath chains delivered energy into a region of the sky, as CSV on stdout (or `--csv <path>`). The config is the scene; the region, the symmetry the rows are merged under, the ray budget and the seed are all options, never config fields — so one config can be asked several questions from a script, and a `--seed` makes any of them reproducible. A scene whose `ray_num` is `"infinite"` runs until Ctrl-C and still writes its result; with `--csv` the file is rewritten atomically every second, so it is complete whenever it is read. Progress goes to stderr; stdout is the CSV alone.
- `Lumice benchmark -f <config>` is for performance regression testing — see [`../performance-testing.md`](../performance-testing.md). It is **not** how you run a normal simulation, and it takes only `-f`, `--backend`, `-v`, `-d`, `-h` (no `-o`: it writes nothing; no `--workers`: the worker counts are the measurement itself). The former `--benchmark` flag exits with a hint pointing here.

## 5. Performance expectations

Lumice traces light wavelength-by-wavelength. For a discrete spectrum (the typical case in `light_source.spectrum: [{wavelength, weight}, ...]`), the work scales as **`ray_num × N(wavelengths)`**. The example config uses 9 wavelengths × `ray_num=5e7` ⇒ ~4.5 × 10⁸ rays.

Practical first-run advice:

- Want a result in seconds? Drop `ray_num` to `1e6` and use a single wavelength (e.g. `[{"wavelength": 550, "weight": 1.0}]`).
- Want a publication-quality image? Keep `ray_num=5e7` or higher and the full 9-wavelength spectrum, and expect about 2 minutes on a modern multi-core laptop.

For the precise relationship between `ray_num`, batches, and wavelengths, and for performance tuning beyond the basics, see [`05-faq.md`](05-faq.md) "ray_num × wavelength semantics" and [`../performance-testing.md`](../performance-testing.md).

## Further reading

- Try ready-made recipes → [`04-recipes.md`](04-recipes.md)
- FAQ, defaults, GUI vs JSON differences → [`05-faq.md`](05-faq.md)
- Full schema → [`../configuration.md`](../configuration.md)
