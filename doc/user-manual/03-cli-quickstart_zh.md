[English version](03-cli-quickstart.md)

# CLI 快速上手

本章介绍如何在命令行下跑 Lumice — 适合批处理、CI、无显示器的服务器、可复现的 recipe。

> **前置**：`build/cmake_install/Lumice` 已构建。如果还没有，参见 [`01-install_zh.md`](01-install_zh.md)。

## 1. 最简调用

最少参数：

```bash
./build/cmake_install/Lumice -f examples/config_example.json
```

这会把 JPEG 输出写到当前工作目录。指定输出目录：

```bash
./build/cmake_install/Lumice -f examples/config_example.json -o /tmp/lumice-out
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
./build/cmake_install/Lumice -f config.json -v   # trace 级日志
./build/cmake_install/Lumice -f config.json -d   # debug 级日志
```

`-v` 大致是"按 batch 显示生成进度"；`-d` 额外输出 RNG seed、散射层调度等诊断信息。先用 `-v`，只在追 bug 时再上 `-d`。

## 4. 完整 flag 一览

`Lumice -h` 打印的完整 flag 列表（事实锚：`./build/cmake_install/Lumice -h`）：

```text
Usage: ./build/cmake_install/Lumice -f <config_file> [options]

Options:
  -f <file>          Specify the configuration file (required)
  -o <dir>           Output directory for rendered images (default: current directory)
  --format <fmt>     Output image format: jpg or png (default: jpg)
  --quality <1-100>  JPEG quality (default: 95, ignored for PNG)
  --benchmark        Run dual-mode benchmark (single-worker + multi-worker)
                     and emit two [BENCHMARK] JSON lines
  -v                 Verbose output (trace level logging)
  -d                 Debug output (debug level logging)
  -h                 Show this help message and exit
```

要点：

- `-f` 是唯一必需 flag。不带它会以非零退出并打印 usage 提示。
- `--format png` 切到无损 PNG，此时 `--quality` 被忽略。
- `--benchmark` 用于性能回归测试 — 详见 [`../performance-testing_zh.md`](../performance-testing_zh.md)，**不是**普通模拟用法。

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
