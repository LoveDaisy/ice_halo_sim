[English version](02-gui-quickstart.md)

# GUI 快速上手

本章带你在 Lumice GUI 中完成首次交互式模拟：启动应用、加载内置示例、跑一次模拟、读懂预览。看完你会知道点哪些面板、各浮窗是什么含义。

> **前置**：构建产物含 GUI（`./scripts/build.sh -j release` 会同时产出 `build/cmake_install/LumiceGUI`）。

## 1. 启动

```bash
./build/cmake_install/LumiceGUI
```

首次启动后主窗口为空载状态：

![LumiceGUI 默认空白状态](../figs/gui_screenshot_default.jpg)

## 2. 主窗口结构

下图为主界面 6 区域的标注图，编号在本手册中保持一致。

![GUI 布局标注图（中文）](../figs/gui_screenshot_example_07.jpg)

| 编号 | 区域 | 用途 |
|------|------|------|
| 1 | 顶部栏（Top Bar） | 打开/保存项目、Run/Stop 模拟、状态徽标 |
| 2 | 左侧面板（Left Panel） | 晶体列表、光源、散射、渲染配置卡片 |
| 3 | 晶体预览（Crystal Preview） | 当前晶体的 3D 预览（线框 / 隐藏线 / 透视 / 着色）|
| 4 | 渲染预览（Render Preview） | 边跑边累积的光晕图像 |
| 5 | 浮动镜头条（Floating Lens Bar） | 在多种镜头投影间快速切换（线性、等距鱼眼、等积鱼眼……）|
| 6 | 状态栏（Status Bar） | 当前光线数、耗时、日志严重度计数 |

> 区域名按"功能"取，不按视觉位置 — UI 颜色 / 像素位置可能在版本间漂移，但这 6 个功能稳定。

## 3. 加载内置示例

`File ▶ Open` 选择 `examples/config_example.json`。左侧面板会被填充示例的 4 个晶体、光源、散射层和 1 个渲染条目：

![加载示例](../figs/gui_screenshot_example_01.jpg)

晶体预览展示当前选中的晶体；点击列表里其他晶体即可切换预览对象。

## 4. 跑模拟、读预览

点击顶部栏的 **Run**。渲染预览开始实时累积光线：

![带网格 overlay 的渲染预览](../figs/gui_screenshot_example_02.jpg)

模拟运行中：

- 状态栏显示当前光线数与已耗时；
- 浮动镜头条可以**不停机**切换镜头投影 — 同一份光线数据被实时重投影；
- 上图的网格 overlay **仅 GUI** 使用，**不会**写入 JSON 配置 — 只是辅助你在预览里读角度。

提前停止？点顶部栏的 **Stop**，部分结果会保留在屏幕上。

## 5. 从零搭一个新 entry

不想直接打开示例，想自己搭一个光晕 recipe？最短路径：

1. **File ▶ New** 新建空项目。
2. 左侧面板 **Crystal** 卡片点 **Edit**（或 **+ Add**）打开晶体编辑器：

   ![晶体编辑对话框](../figs/gui_edit_crystal.jpg)

   选个预设（例如 **Hexagonal Prism**），如有需要可调 `height`，确认。（其它形状字段如 `face_distance` schema 中接受但当前引擎尚未实现 — 详见 [`05-faq_zh.md`](05-faq_zh.md) §4。）
3. 打开 **Light Source**，设 `azimuth`（太阳方位角）和 `altitude`（太阳高度角）。
4. 打开 **Render**，挑镜头、分辨率、视场角 — 浮动镜头条之后还能切换，所以默认值就行。
5. 点 **Run**。

> 📷 待补：Crystal Tab 整体截图（已登记到 `progress.md` 占位锚清单；closeout 阶段会汇总进 SUMMARY.md "待补充清单"）。

## 6. 保存与重载

`File ▶ Save As` 写出 `.lmc` 文件（其实是 JSON，CLI 也能直接吃）。在 GUI 里重新打开它会还原**晶体 / 光源 / 渲染**数据，但**不**还原 GUI-only 状态（镜头投影选择、网格 overlay、晶体预览样式）。差异完整列表见 [`05-faq_zh.md`](05-faq_zh.md) "GUI 与 JSON 能力差异"。

## 延伸阅读

- 同一份 `.lmc` 用 CLI 跑一遍 → [`03-cli-quickstart_zh.md`](03-cli-quickstart_zh.md)
- 用现成 recipe 复现经典光晕 → [`04-recipes_zh.md`](04-recipes_zh.md)
- 完整 GUI 面板参考 → [`../gui-guide_zh.md`](../gui-guide_zh.md)
- 全部字段名与类型 → [`../configuration_zh.md`](../configuration_zh.md)
