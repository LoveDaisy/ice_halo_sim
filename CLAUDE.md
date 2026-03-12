# Lumice - Project Guide for Claude

Lumice 是一个冰晕 (Ice Halo) 光线追踪模拟程序，C++17 实现。

## 常用命令

```bash
# 构建
./build.sh -j release             # 并行构建 release
./build.sh -tj release            # 构建 + 单元测试
./build.sh -gtj release           # 构建 GUI + 全部测试（含 GUI 测试）
./build.sh -k release             # 清理构建产物后重新构建（保留依赖缓存）

# 运行
./build/cmake_install/Lumice -f config_example.json
./build/cmake_install/Lumice -f config.json -v    # verbose

# 测试
./build.sh -tj release            # 单元测试（GoogleTest via CTest）
./build.sh -gtj release           # GUI 测试（ImGui Test Engine，需 display server）
pytest test/e2e/ -v               # E2E 测试（Python，需 Pillow）

# 格式化
./format.sh                       # clang-format src/ 和 test/
```

构建产物: Release → `build/cmake_install/`, Debug → `build/cmake_build/`

## 项目结构

```
src/
├── config/    # 配置解析：config_manager, crystal/light/filter/render/proj_config, sim_data
├── core/      # 核心：crystal, geo3d, math, optics, simulator, filter, raypath, buffer, def
├── server/    # 服务层：server, consumer, render, stats, show_rays, c_api
├── util/      # 工具：logger, threading_pool, arg_parser, queue, illuminant, color_data
└── include/   # 公共 C API 头文件（lumice.h）
test/          # test_*.cpp（单元测试）+ e2e/（端到端测试）
```

- 命名空间: `lumice`
- 公共 API 仅通过 C 接口（`lumice.h`）暴露
- 头文件 `.hpp`，源文件 `.cpp`

## 代码风格

基于 Google C++ Style，由 `.clang-format` + `.clang-tidy` 强制执行。

| 类别 | 风格 | 示例 |
|------|------|------|
| 类/结构体/枚举 | CamelCase | `class CrystalModel` |
| 函数 | CamelCase | `void TraceRay()` |
| 变量 | lower_case | `int ray_count` |
| 私有成员 | lower_case + `_` | `int count_` |
| 常量 | k + CamelCase | `constexpr int kMaxRays` |
| 命名空间 | lower_case | `namespace lumice` |

格式化规则: 2 空格缩进，120 字符行宽，K&R 花括号，指针左对齐 (`int* ptr`)

## 关键约定

- C++17，使用 `auto`, `nullptr`, `override`, 智能指针
- 优先 `const` / `constexpr`
- 禁止裸 `new`/`delete`，禁止 `using namespace`
- 函数体不超过 200 条语句
- Release 启用 IPO

## 技术栈

- **构建**: CMake 3.14+ / Ninja
- **依赖管理**: CPM.cmake（依赖缓存在 `build/cpm_cache/`）
- **依赖**: nlohmann/json v3.10.5, spdlog v1.15.0, tl-expected v1.1.0, stb (图像读写), GoogleTest v1.15.2
- **GUI 依赖**: Dear ImGui v1.91.8-docking, GLFW 3.4, nfd v1.2.1, imgui_test_engine v1.91.8

## 文档语言策略

- 英文版为默认，中文版加 `_zh` 后缀（如 `configuration.md` / `configuration_zh.md`）
- 代码注释使用英文
- 详细配置说明见 `doc/configuration.md`

## 注意事项

- `config.json`, `test.json` 已在 `.gitignore`，用 `config_example.json` 作模板
- `*.jpg` 已在 `.gitignore`，`test/e2e/references/*.jpg` 通过 `!` 规则排除
- `scratchpad/` 已在 `.gitignore`，仅本地任务管理用
- **禁止使用 `git add -f` 强制追踪被 `.gitignore` 忽略的文件**。`.gitignore` 中忽略的文件都是有理由的, 遇到不确定的情况, 先询问用户
