[English version](00-overview.md)

# Lumice 用户手册 — 总览

欢迎使用 Lumice 用户手册。Lumice 是一个 C++17 编写的冰晶光晕（ice halo）光线追踪模拟器：通过追踪光线穿过冰晶的过程，复现你在寒冷的天气下可能在太阳周围看到的光晕图样。

本手册定位为**新手入口**，按用户旅程组织，而不是按参考主题。如果你需要完整的配置 schema、内部架构或开发者文档，请通过每章末尾的"延伸阅读"链接，跳转到 `doc/` 下既有的技术参考文档。

> **目标**：新用户从干净的 checkout 出发，按照下方任意一条路径，30 分钟内完成首次模拟并看到渲染出的光晕图像。

## 用户旅程

按照你的目标选择路径。每条路径都是自包含的，不必先看其他路径。

```
   ┌────────────┐     ┌──────────────────┐     ┌─────────────────┐
   │ 01-install │ ──▶ │ 02-gui-quickstart│ ──▶ │   04-recipes    │
   └────────────┘     │ （交互式上手）   │     │ （复现经典光晕）│
        │             └──────────────────┘     └─────────────────┘
        │                                                ▲
        │             ┌──────────────────┐               │
        └───────────▶ │ 03-cli-quickstart│ ──────────────┘
                      │ （批处理 / JSON）│
                      └──────────────────┘
                              │
                              ▼
                      ┌──────────────────┐
                      │ 05-faq           │
                      │ （默认值、GUI 与 │
                      │  JSON 差异、边界）│
                      └──────────────────┘
```

- **GUI 优先**（首次用户推荐）：`01-install` → `02-gui-quickstart` → `04-recipes`。
- **CLI 优先**（批处理 / 脚本化场景推荐）：`01-install` → `03-cli-quickstart` → `04-recipes`。
- **遇到问题？** 直接看 `05-faq`：默认值、"GUI 与 JSON 能力差异"、已知限制。

## 本手册不涵盖什么

本手册**不重复** `doc/` 下既有的技术参考文档。深入主题请查阅：

| 主题 | 参考文档 |
|------|----------|
| 完整 JSON 配置 schema | [`configuration_zh.md`](../configuration_zh.md) |
| 坐标系约定 | [`coordinate-convention_zh.md`](../coordinate-convention_zh.md) |
| GUI 内部结构与面板参考 | [`gui-guide_zh.md`](../gui-guide_zh.md) |
| 嵌入用 C API | [`c_api_zh.md`](../c_api_zh.md) |
| 性能调优与 benchmark | [`performance-testing_zh.md`](../performance-testing_zh.md) |
| 系统架构 | [`architecture_zh.md`](../architecture_zh.md) |

## 本手册的约定

- 所有 shell 命令默认在项目根目录（含 `CMakeLists.txt` 的目录）执行。
- 手册内部路径用 `../` 引用 `doc/` 同级（如 `../figs/<image>.jpg`）。
- 代码块以 `./scripts/build.sh -j release` 构建产出的 `build/cmake_install/Lumice` 为锚验证。
- 涉及代码事实的描述都会注明源文件行号（例如 `src/core/simulator.cpp:482-498`）。

## 延伸阅读

- 接下来：安装 Lumice → [`01-install_zh.md`](01-install_zh.md)
- 全部技术参考索引 → [`../README_zh.md`](../README_zh.md)
