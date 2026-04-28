[English version](gui-guide.md)

# GUI 使用指南

本文档介绍 Lumice GUI 应用 — 一个用于交互式配置和预览冰晕模拟的图形界面。

## 构建和运行

```bash
# 构建 GUI 应用
./scripts/build.sh -gj release

# 运行
./build/cmake_install/LumiceGUI
```

GUI 需要 display server 和支持 OpenGL 3.2 Core Profile 的 GPU。

## 界面布局

主窗口分为 6 个区域。下方截图与本文档其余部分使用同一套编号。

![LumiceGUI 默认界面布局](figs/gui_screenshot_default.jpg)

| # | 区域 | 用途 |
|---|------|------|
| 1 | Top Bar（顶栏） | 文件操作（New / Open / Save）、模拟控制（Run / Stop / Revert）、面板折叠按钮 |
| 2 | Left Panel — Crystal Parameters（左侧面板，晶体参数） | 散射层及每层中的晶体卡片（几何 / 取向 / 过滤器 / 占比） |
| 3 | Right Panel — View Parameters（右侧面板，视觉参数） | Scene（光源 + 模拟）、View（投影 + 相机）、Display（分辨率 / EV / 长宽比 / 背景）、Overlay（辅助线） |
| 4 | Render Preview（渲染预览） | 光线累计形成的、按 lens 投影的冰晕图像 |
| 5 | Status Bar（状态栏） | 模拟状态徽标、光线总数、当前分辨率 / lens / FOV、文件名、Log 切换 |
| 6 | Popup Editor（编辑弹窗） | 从晶体卡片打开的模态编辑器；Crystal / Axis / Filter 三个 tab |

左侧栏与右侧栏都可独立折叠 — 模拟过程中想让 Render Preview 占据更宽空间时很有用。

## Top Bar（顶栏）

从左到右：

- **左面板折叠按钮**：`<` 收起左面板，`>` 展开（也可用快捷键 `[`）。
- **Run / Stop**：固定宽度的按钮，根据模拟状态在绿色 **Run** 与红色 **Stop** 之间切换。模拟期间禁用的控件在运行结束后会自动恢复可用。
- **Revert**：仅当模拟结束后又改了参数（状态 `Modified`）时出现；用于把配置还原到产生上次结果时的状态。
- **New / Open**：项目生命周期操作（模拟期间禁用）。
- **Save**：点击后弹出子菜单，包含：
  - `Save` / `Save Copy` —— 写入 `.lmc` 文件
  - `Screenshot...` —— 把当前 Render Preview 导出为 PNG
  - `Dual Fisheye Equal Area...` / `Equirectangular...` —— 服务端离屏渲染导出（需要已完成的模拟）
  - `Config JSON...` —— 把配置导出为 JSON
  - `Include Texture in .lmc` / `Include Overlay in Screenshot` —— 控制下次保存 / 截图的 toggle
- **右面板折叠按钮**：`<` / `>` 与左侧对称（也可用快捷键 `]`）。

## Left Panel — Crystal Parameters（左侧面板）

### 散射层与晶体卡片（Layer & Crystal Cards）

左面板包含一个或多个**散射层（Layer）**，每层包含一个或多个**晶体卡片（Crystal Card）**。Layer 自上而下堆叠；离开上一层的光线按该层的 `Prob.` 概率进入下一层（仅有一层时该滑块禁用）。

![单次散射 vs 多次散射示例](figs/gui_scattering_combined.jpg)

每张晶体卡片对应散射层中的一项条目：

- **缩略图**（左）：晶体几何的小尺寸 3D 预览；几何或取向变化时缓存自动重画。
- **Crystal 行**：类型（`Prism` / `Pyramid`）+ `Edit` 按钮，打开 Popup Editor 的 Crystal tab。
- **Axis 行**：取向分布的命名预设（`Parry` / `Column` / `Lowitz` / `Plate` / `Random` / `Custom`）+ `Edit` 打开 Axis tab。
- **Filter 行**：光路过滤器的单行摘要 + `Edit` 打开 Filter tab。
- **占比滑块**（`prop.`）：本条目在所属 Layer 中的权重（0 – 100）。

缩略图支持 4 种渲染风格。风格选择器位于 Popup Editor 左侧的常驻预览面板（在 Crystal / Axis / Filter 三 tab 之间共享） —— 详见下文 [Popup Editor](#popup-editor编辑弹窗) 节。

![晶体预览 4 种风格：线框 / 隐藏线 / 透视 / 着色](figs/gui_crystal_styles_combined.jpg)

每个 Layer 底部有 `+ Crystal` 按钮（添加同层条目），Layer 头部右对齐 `x` 用于删除整个 Layer（仅有一层时禁用）。整个面板底部有 `+ Layer` 按钮新增散射层。

### 卡片悬停操作（Card Hover Actions）

把鼠标悬停在某张卡片上时，右上角会出现两个小按钮：

- `D` —— 在同层复制该条目（深拷贝 crystal / axis / filter / proportion 全部字段）。
- `×` —— 删除该条目。红色作为破坏性操作提示；当该层只剩这一条时禁用。

按钮通过 alpha 渐变显示 / 隐藏，所以卡片布局保持稳定；点击在首次悬停帧即生效。

## Right Panel — View Parameters（右侧面板）

右面板汇集了所有**影响光线如何被渲染**的参数。4 个折叠节默认全部展开。

### Scene（场景）

- **Sun**：`Altitude`（-90° ~ 90°）、`Diameter`（视角直径，0.1° ~ 5°）、`Spectrum`（标准光源 `D50` / `D55` / `D65` / `D75` / `A` / `E` 之一）。
- **Simulation**：`Infinite rays` 复选框（让模拟器持续累计直到手动停止）、有限模式下的 `Rays(M)`（单位：百万）、`Max hits`（每条光线最多碰撞次数，1 – 20）。

### View（视图）

- **Projection**：`Lens Type` 提供 10 种 lens 投影 —— Linear、Rectangular、Fisheye（Equidistant / Equal Area / Stereographic / Orthographic）以及 Dual Fisheye（Equidistant / Equal Area / Stereographic / Orthographic）。Combo 中按分组排序，使 Orthographic 变体紧邻其 fisheye / dual-fisheye 同族。`FOV` 由具体 lens 限定上限；`Visible`（front / back / all）控制只渲染哪一半球的光线。
- **Camera**：`Elevation`、`Azimuth`、`Roll`。Full-sky lens（dual 系列与 Equirectangular 导出格式）下三者均禁用并强制为 0。

不同 lens 的成像几何差异显著：

![五种 lens 投影对比](figs/gui_lens_projections_combined.jpg)

### Display（显示）

- **Rendering**：`Resolution`（512 / 1024 / 2048 / 4096；以棕色高亮提示更改会触发重新模拟）；`EV`（曝光偏移，-6 ~ +6 stops）。
- **Aspect Ratio**：`Preset` 下拉框（Free、16:9、3:2、4:3、1:1、2:1、Match Background）+ `Portrait` ↔ `Landscape` 切换按钮。请求的长宽比无法满足（窗口太小）时会出现提示行，显示实际可用比与请求比。
- **Background**：`Load Bg...`、`Clear`、`Show` 复选框、以及 `Alpha` 滑块（控制背景在渲染图下的合成透明度）。

### Overlay（叠加）

Render Preview 上的三组互相独立的辅助线：`Horizon`（地平线）、`Grid`（网格）、`Sun Circles`（太阳圈）。每组都有复选框、颜色块和 alpha 滑块。Sun Circles 还提供 `Edit Circles...` 弹窗，支持 9° / 22° / 28° / 46° 预设角度以及任意自定义角度。

## Render Preview（渲染预览）

中央区域显示实时的、按 lens 投影的冰晕图像。空闲时显示禁用态的 `Render Preview` 占位文本；模拟一开始累积光线，纹理就会逐帧更新。Horizon / Grid / Sun Circles / Compass 等 overlay 标签会按当前 lens 同步投影叠加在纹理之上。

![Render Preview 渲染示例](figs/gui_screenshot_example_04.jpg)

## Popup Editor（编辑弹窗）

晶体卡片上的 `Edit` 按钮会打开**单一**的模态编辑器，含 3 个 tab（Crystal / Axis / Filter），左侧带常驻的晶体预览面板。预览每帧重画，所以几何或取向的修改可立刻看见。自 v15 起，借助 ImGui multi-viewport，可以把弹窗拖出主窗口、独立成一个 OS 窗口。

![Popup Editor — Crystal / Axis / Filter 三 tab](figs/gui_edit_modal_combined.jpg)

### Crystal Tab

晶体几何参数：

- **Type**：`Prism`（六棱柱）或 `Pyramid`（带上下截顶楔形的六棱锥）。
- **形状参数**：Prism 用 `height`；Pyramid 用 `prism_h`、`upper_h`、`lower_h` 以及楔角 `upper_alpha` / `lower_alpha`（默认值对应 Miller 指数 `{1, 0, -1, 1}`）。
- **Face distance**：六个值，分别对应六个棱面，用于支持非规则六棱截面。

### Axis Tab

3 组互相独立的角度分布，控制晶体取向：`zenith`、`azimuth`、`roll`。每组分布包含：

- 类型 RadioButton：`Gauss`、`Uniform`、`Zigzag`、`Laplacian`，以及保留兼容的 `GaussLegacy`。
- `mean`（中心角）和 `std`（含义随类型而变 —— Gauss 时是标准差、Uniform 时是全范围、Zigzag 时是振幅、Laplacian 时是 scale）。

### Filter Tab

光路过滤器配置。Filter 既可指定特定面序列（含对称选项 `P` / `B` / `D`），也可只限定入射 / 出射面。最终结果会在对应的晶体卡片 Filter 行以单行摘要展示。

## Status Bar（状态栏）

从左到右：

- **模拟状态徽标**：`Ready`（绿）、`Simulating...`（黄）、`Done`（蓝）、`Modified`（橙）。
- **累计光线数**（仅在非 0 时显示）：按数量级缩放为 `x10^3 / x10^6 / x10^9`。
- **分辨率 / lens / FOV**：例如 `1024x512 Fisheye Equal Area FOV:180`。
- **文件名**，未保存的修改用 `*` 标记。
- **Log 切换按钮**（右对齐）：`Log [>]` 打开 / 收起底部日志面板。

## 文件操作

### 键盘快捷键

| 快捷键 | 操作 |
|--------|------|
| Ctrl+S | 保存项目 |
| Ctrl+Shift+S | 另存为副本 |
| `[` | 切换左面板折叠 |
| `]` | 切换右面板折叠 |

Run / Stop / New / Open 没有绑定快捷键 —— 通过 Top Bar 按钮触达（导出类操作走 Save 子菜单）。

### 项目文件格式（`.lmc`）

Lumice 使用二进制项目文件格式（`.lmc`），存储：

- **配置**：所有 crystal、scene、render、filter 设置，以语义化 JSON 形式保存
- **预览纹理**：可选的内嵌 PNG，保存最近一次渲染结果

格式头部为 44 字节，包含魔数 `LMC\0`、版本字段，以及 JSON / 纹理两段载荷的偏移量与大小。值以人类可读的语义类型存储（如 `"prism"` 而非枚举索引），保证前向兼容性。

### 未保存的更改

当存在未保存的修改时（状态栏 `*` 提示），应用会在以下操作前弹出提示：

- 新建项目
- 打开另一个项目
- 关闭应用

提示弹窗提供 `Save`、`Don't Save`、`Cancel` 三个按钮。

## 模拟工作流

1. **配置**：在左面板设置散射层与晶体卡片，再到右面板设置 scene / view / display / overlay。
2. **运行**：点击 Top Bar 的 `Run`。当前配置被序列化并提交给模拟核心；模拟期间参数控件被禁用。
3. **监控**：状态栏显示运行状态和累计光线数；Render Preview 持续刷新。
4. **查看**：模拟结束后（状态 `Done`）结果继续保留在 Render Preview 中。
5. **停止**：点击 `Stop` 提前终止；停止时已累积的光线仍然可见。
6. **恢复**：模拟完成后又改了参数（状态 `Modified`）时，`Revert` 把配置还原到产生上次结果时的状态。

模拟状态 —— `Ready` / `Simulating` / `Done` / `Modified` —— 由状态栏显示，并控制 Top Bar 上各动作的可用性。

## 相关文档

- [配置文档](configuration_zh.md) —— 详细的配置参考（JSON 格式）
- [系统架构文档](architecture_zh.md) —— 系统架构和 GUI 模块设计
- [开发指南](developer-guide_zh.md) —— GUI 测试与开发
- [用户手册 — GUI 快速上手](user-manual/02-gui-quickstart_zh.md) —— 面向新用户的逐步教程
