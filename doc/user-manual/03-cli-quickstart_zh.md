[English version](03-cli-quickstart.md)

# CLI 快速上手

本章介绍如何在命令行下跑 Lumice — 适合批处理、CI、无显示器的服务器、可复现的 recipe。

> **前置**：`build/cmake_install/static/Lumice` 已构建。如果还没有，参见 [`01-install_zh.md`](01-install_zh.md)。

## 1. 最简调用

最少参数：

```bash
./build/cmake_install/static/Lumice -f examples/config_example.json
```

这会把 JPEG 输出写到当前工作目录。指定输出目录：

```bash
./build/cmake_install/static/Lumice -f examples/config_example.json -o /tmp/lumice-out
```

![Lumice 启动](../figs/cli_screenshot_01.jpg)

## 2. 看输出

跑完后输出目录里会有"每个 render 条目一张图"。内置示例定义了 4 个渲染条目，因此会得到 4 张图：

| 文件 | 镜头 / 视图 | 说明 |
|------|-------------|------|
| `example_img_01.jpg` | 等积双鱼眼，全天空 | 默认的"全天看光晕"视图 |
| `example_img_02.jpg` | 线性镜头，较窄视场 | 接近"普通相机去畸变"的视图 |
| `example_img_03.jpg` | 等距鱼眼 | 适合按角度量取光晕半径 |
| `example_img_04.jpg` | 立体投影鱼眼 | 靠近地平线区域圆形保持得好 |

![示例输出 1](../figs/example_img_01.jpg)
![示例输出 2](../figs/example_img_02.jpg)
![示例输出 3](../figs/example_img_03.jpg)
![示例输出 4](../figs/example_img_04.jpg)

控制台还会打印一段 `Stats:` 总结（光线数、耗时、按波长累积等）。想要复现某次跑出的图，把这段保存下来即可。

## 3. Verbose 与 Debug 模式

```bash
./build/cmake_install/static/Lumice -f config.json -v   # trace 级日志
./build/cmake_install/static/Lumice -f config.json -d   # debug 级日志
```

`-v` 大致是"按 batch 显示生成进度"；`-d` 额外输出 RNG seed、散射层调度等诊断信息。先用 `-v`，只在追 bug 时再上 `-d`。

## 4. 完整 flag 一览

CLI 有两个子命令：`render` 与 `benchmark`。`render` 是默认值：`Lumice -f config.json` 与 `Lumice render -f config.json` 是同一条命令，所以上面所有示例跑的都是 render。每个子命令只接受自己的选项；`Lumice <子命令> -h` 打印该子命令的帮助页。

`Lumice -h` 打印的完整列表（事实锚：`./build/cmake_install/static/Lumice -h`）：

```text
Usage: ./build/cmake_install/static/Lumice [render] -f <config_file> [options]
       ./build/cmake_install/static/Lumice benchmark -f <config_file> [options]
       ./build/cmake_install/static/Lumice <subcommand> -h

Lumice — simulate ice halos by tracing rays through ice crystals.

Subcommands:
  render             Simulate and write halo images. This is the default when
                     no subcommand is given: `./build/cmake_install/static/Lumice -f ...` is a render.
  benchmark          Run a throughput benchmark and print [BENCHMARK] JSON
                     (`./build/cmake_install/static/Lumice benchmark -h` for its options)

Options for render (the default subcommand):
  -f <file>          Specify the configuration file (required)
  --backend <name>   Trace backend: auto, cpu, metal, or cuda (default: auto).
                     'auto' and 'cpu' both select the CPU route today; 'metal'
                     falls back to CPU if unavailable. The LUMICE_TRACE_BACKEND
                     env var, if set, still overrides this (debug/CI only).
  -v                 Verbose output (trace level logging)
  -d                 Debug output (debug level logging)
  -h, --help         Show this help message and exit
  -o <dir>           Output directory for rendered images (default: current directory)
  --format <fmt>     Output image format: jpg or png (default: jpg)
  --quality <1-100>  JPEG quality (default: 95, ignored for PNG)
  --workers <N>      Number of CPU simulation worker threads (default: automatic —
                     one per physical core, capped at a ceiling above which no
                     machine measured ran faster; an explicit N is never capped).
                     Machine-dependent, so it is a command-line switch rather than
                     a config-file field: a config travels between machines and a
                     worker count should not travel with it. Ignored on a GPU route
                     (single engine).

Examples:
  ./build/cmake_install/static/Lumice -f config.json
  ./build/cmake_install/static/Lumice -f config.json -o /tmp/output
  ./build/cmake_install/static/Lumice -f config.json --format png
  ./build/cmake_install/static/Lumice -f config.json --quality 80
  ./build/cmake_install/static/Lumice -f config.json --backend metal
  ./build/cmake_install/static/Lumice -f config.json --workers 4
  ./build/cmake_install/static/Lumice -f config.json -v
  ./build/cmake_install/static/Lumice benchmark -f examples/bench_config.json
```

`Lumice benchmark -h`：

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

要点：

- `-f` 是唯一必需 flag。不带它会以非零退出并打印 usage 提示。
- `--format png` 切到无损 PNG，此时 `--quality` 被忽略。
- `--workers <N>` 覆盖自动 worker 数（每个物理核一个，但有一个实测上限；该上限只作用于自动值，你显式给出的 `N` 永远不受它约束）。它是命令行开关而不是 config 字段，是有意的：worker 数描述的是**机器**，而 config 文件会在机器之间流转。非法值（`0` / 负数 / 非数字）以非零退出，不静默回退到默认值。
- `Lumice benchmark -f <config>` 用于性能回归测试 — 详见 [`../performance-testing_zh.md`](../performance-testing_zh.md)，**不是**普通模拟用法；它只接受 `-f`、`--backend`、`-v`、`-d`、`-h`（没有 `-o`：它不写文件；没有 `--workers`：worker 数就是测量口径本身）。原来的 `--benchmark` 旗现在会报错并给出指向这里的迁移提示。

## 5. 性能预期

Lumice 按波长追踪光线。对于离散波长 spectrum（典型场景：`light_source.spectrum: [{wavelength, weight}, ...]`），总工作量约为 **`ray_num × N(wavelengths)`**。示例配置 9 段波长 × `ray_num=5e7` ⇒ 约 4.5 × 10⁸ 条光线。

新手首跑建议：

- 想几秒看到结果？把 `ray_num` 降到 `1e6`，spectrum 改成单波长（`[{"wavelength": 550, "weight": 1.0}]`）。
- 想出版级清晰度？保持 `ray_num=5e7` 以上 + 完整 9 段 spectrum，预期在现代多核笔记本上约 2 分钟。

`ray_num` × batch × wavelength 的精确关系，以及更深入的性能调优，见 [`05-faq_zh.md`](05-faq_zh.md) "ray_num × wavelength 语义" 和 [`../performance-testing_zh.md`](../performance-testing_zh.md)。

## 延伸阅读

- 试现成 recipe → [`04-recipes_zh.md`](04-recipes_zh.md)
- FAQ、默认值、GUI 与 JSON 差异 → [`05-faq_zh.md`](05-faq_zh.md)
- 完整 schema → [`../configuration_zh.md`](../configuration_zh.md)
