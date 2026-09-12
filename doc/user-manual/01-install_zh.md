[English version](01-install.md)

# 安装与构建 Lumice

本章带你从一份干净的 clone，走到一个可用的 `Lumice` 二进制文件（需要的话再加上 `LumiceGUI`），并执行一次 smoke test 确认工具链就位。

## 前置依赖

Lumice 是一个使用 CMake 构建的 C++17 项目，环境需要：

| 工具 | 最低版本 | 说明 |
|------|----------|------|
| C++ 编译器 | C++17 | Apple Clang ≥ 14、GCC ≥ 11、MSVC 2022 |
| CMake | 3.14 | `cmake --version` 验证 |
| Ninja | 1.10 | 推荐使用的 generator（`ninja --version`）；`scripts/build.sh` 会传 `-G Ninja` |
| Python | 3.9 | **每一次**构建都需要：CMake 在构建期调用 `scripts/embed_binary.py` 把标注字体嵌进二进制。跑 `test/` 下的 E2E 测试也用它 |
| OpenGL 开发包 | — | 仅 GUI 构建（`-g`）需要。macOS 与 Windows 无需额外安装。Linux 上 GLFW 依赖 X11、文件对话框依赖 GTK 3：`sudo apt-get install libgl-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libgtk-3-dev` |

外部依赖（spdlog、nlohmann/json、stb、googletest，以及 GUI 用到的 GLFW、Dear ImGui、nativefiledialog-extended 等）在 configure 阶段由 [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake) 自动拉取，无需手动安装。下载的源码按机器缓存在 `$HOME/.cache/lumice-cpm`（未设 `HOME` 的 Windows shell 下为 `$USERPROFILE/.cache/lumice-cpm`），同一台机器上再 clone 一份或新开 worktree 不会重新下载；想换位置可设置 `CPM_SOURCE_CACHE`。

## 构建

推荐用脚本 `scripts/build.sh` 构建：

```bash
# CLI 的 Release 构建，并行，产物安装到 build/cmake_install/static/
./scripts/build.sh -j release
```

其他常用变体（完整列表见 `scripts/build.sh -h`）：

```bash
./scripts/build.sh -gj release    # 连 GUI 一起构建（Dear ImGui + GLFW + OpenGL）
./scripts/build.sh -tj release    # 构建 + 跑单元测试
./scripts/build.sh -gtj release   # 构建 GUI + 跑单元测试与 GUI 测试（需要显示服务器）
LUMICE_SKIP_GUI_TESTS=1 ./scripts/build.sh -gtj release   # 在无显示器的机器上跳过 GUI 测试
./scripts/build.sh -k release     # 清理本 flavor 的构建树后重新构建（保留 CPM 缓存）
```

`-t` / `-g` / `-b` 决定*构建什么*（测试 / GUI 应用 / 基准）；只有带过 `-g` 的构建才会产出 GUI 二进制。`-s` 把库的 flavor 切到 shared（省略即 static），可与上述任意选项组合。

Release 构建成功后，关键产物位置如下：

| 产物 | 路径 | 用途 |
|------|------|------|
| CLI 二进制 | `build/cmake_install/static/Lumice` | 命令行执行 JSON 配置 |
| GUI 二进制 | `build/cmake_install/static/LumiceGUI` | 交互式 GUI（仅 `-g` 构建产出；除系统 OpenGL 驱动外无其他运行时依赖）|

> 两棵树都按 flavor 分开：`-s`（shared）构建与静态构建永不共用目录。CMake 构建树是
> `build/cmake_build/<flavor>/`。本手册其余章节默认使用 `build/cmake_install/static/` 下的 Release 产物。

![Lumice CLI 启动横幅](../figs/cli_screenshot_01.jpg)

## Smoke test

跑一遍内置示例配置，确认端到端可用。在项目根目录执行（CLI 不会替你创建输出目录）：

```bash
mkdir -p /tmp/lumice-smoke
./build/cmake_install/static/Lumice -f examples/config_example.json -o /tmp/lumice-smoke
```

你应该看到：

1. 启动时几行 `[I]` 日志（`CommitConfig`、`GenerateScene`、`ConsumeData`），标识本次运行；
2. 大约每秒一次，每张输出图一行 `Saved: ...`，后面跟一行 `Stats: sim_rays=..., crystals=..., orientations=...`——CLI 会随模拟累积不断重存中间渲染结果；
3. 所有光线追踪完毕后最后一组 `Saved:` / `Stats:`；
4. `/tmp/lumice-smoke/` 下生成 4 个 `.jpg` 文件，文件名为 `img_01.jpg` … `img_04.jpg`（配置里每个 `render` 条目一张）。

![Lumice CLI 完成输出](../figs/cli_screenshot_02.jpg)

只要 4 张图都生成出来，就说明工具链没问题。

> 示例配置把 `scene.ray_num` 设为 `4.5e8`——这是 9 段离散波长加起来的总数。在较新的多核笔记本上约 2 分钟。想跑得更快？把示例 copy 一份，把 `scene.ray_num` 改到 `1e6` 即可。

## 常见构建问题

| 现象 | 可能原因 | 处理 |
|------|----------|------|
| `cmake: command not found` | 未安装 CMake 或不在 `PATH` | Homebrew / apt / 官方安装包安装 |
| 拉依赖失败 | 首次构建无 CPM 缓存且无网络 | 切到有网环境重试，CPM 缓存默认在 `$HOME/.cache/lumice-cpm`（或 `CPM_SOURCE_CACHE` 指向的位置）|
| configure 在 `Python3` 处失败 | `PATH` 上没有 Python 3 解释器 | 安装 Python 3——每次构建都靠它嵌入字体 |
| GUI configure 在 GLFW / nfd 处失败（Linux） | 缺 X11 / OpenGL / GTK 3 开发头文件 | 按前置依赖表安装，或去掉 `-g` 不构建 GUI |
| GUI 启动时报 GLFW / OpenGL 错误退出 | 没有显示器，或 loader 路径上没有 OpenGL 3.3 驱动 | 在有桌面会话的机器上运行；无显示器的机器改用 CLI |
| `LUMICE_SKIP_GUI_TESTS` 在 CI 上不生效 | 配置缓存陈旧 | `./scripts/build.sh -k release` 清理重建 |

## 延伸阅读

- 在 GUI 里跑第一个配置 → [`02-gui-quickstart_zh.md`](02-gui-quickstart_zh.md)
- 或者在 CLI 里跑 → [`03-cli-quickstart_zh.md`](03-cli-quickstart_zh.md)
- 构建 flag 参考 → [`../developer-guide_zh.md`](../developer-guide_zh.md)
