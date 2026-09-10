# 设计：print 显示模式（减色 / 密度墨）

> 状态：blueprint（2026-09-09 收敛，含一次离线标定得出的定案 `γ`）。**本文是 print 模式的单一设计权威**，
> 覆盖五块下游实现：① `render.tone` / `render.paper` 字段链；② 减色算子落地（core + preview shader）；
> ③ "ink 接管色彩通道"的四项互斥与告警；④ 零能量色余量谓词与两侧提示；⑤ 验收闸与 as-built 回写。
> 改 print 模式相关的任何一块前先读本文，⛔ 不要各自重新推导法则。
>
> 关联：`doc/ev-pipeline-architecture.md`（曝光锚点与 `ExposureScale()`；本设计的前提是**一行不改**它）、
> `doc/gui-custom-spectrum-and-raypath-color.md` §4.8（染色合成路的 z-order 定案；本设计 ⛔ 不碰它）、
> `doc/adaptive-brightness.md`（`e` 的来源侧）。

## 0. 为什么要这份文档

自定义背景色功能上线后收到一条用户反馈：**选白色背景则光晕完全看不见**。追下去发现这不是配色问题，
而是显示算子的结构性后果——用户真实想要的"浅底 + 深色光点"（印刷式的白底黑点）在当前的**加性**算子下
不可表达，任何前景色都救不了。

本文记录该诊断的机制层结论、由此定下的第二条显示法则（减色 / 密度墨）、以及围绕它的五个设计取舍。
写成一份 `doc/` 蓝图而不是散落在各实现任务里，是因为下游是五个彼此独立的实现单元：没有共同蓝图，
每一个都会把同一套法则重新推导一遍，而重新推导出来的版本必然彼此不一致。

## 1. 诊断（机制层，白盒确认）

**加性算子对辐亮度单调不减，因此白底恒等于白。**

三条路同形，都是"辐亮度 + 背景，然后 clamp + gamma"：

| 路 | 位置 | 代码 |
|---|---|---|
| CLI / 导出 | `src/server/render.cpp:899` | `rgb[j] += config_.background_[j];` |
| GUI 预览 shader | `src/gui/preview_renderer.cpp:709` | `tex_color = clampAndGamma(radiance_linear + u_background);` |
| 染色合成路 | `src/server/component_compositor.cpp:325`（在 `:303` `ApplyCompositeBackground` 内） | `linear_rgb[i * 3 + j] += background_linear[j];` |

即

```
out = clamp(L·c + bg, 0, 1)        // linear RGB，其后 sRGB gamma
```

`∂out/∂L ≥ 0` 恒成立。`bg` 取 linear 1.0（sRGB 白）时，**每个像素在加入任何非负能量之前就已经到顶**，
clamp 之后整幅恒等于白。浅灰是同一件事的连续版本：sRGB 200 → linear ≈ 0.58，只剩 0.42 的动态范围。

**推论（这条直接决定了方案的形状）**：`ray_color_`（`src/config/render_config.hpp:172`，
sentinel `-1` 表示"用真实光谱色"）在加性算子下取黑，得到的是**"不发光"而不是"黑点"**——
`rgb[j] = v * config_.ray_color_[j]`（`render.cpp:843` 起的 `use_real_color == false` 分支）
把 `ray_color` 当成一个乘性 tint 作用在**被加的那一项**上，乘 0 只是让这条光线不贡献能量。
所以**"放开前景色自定义"是一个无效方案**，问题不在配色维度上。

## 2. 辅助线不是加性的 ⇒ 真实症状是"线在、晕没了"

annotation 层（网格线、太阳圈、地平线、天顶/天底标记）走的是 **lerp**，不是加法：

- `src/server/render.cpp:922`（及 `:927`）：`rgb[j] = rgb[j] * (1.0f - layer.alpha) + layer.rgb[j] * layer.alpha;`
- 地平线同形：`render.cpp:931`，用文件作用域的 `kOutlineSrgb` / `kOutlineAlpha`（`render.cpp:99-100`）

lerp 对底色不单调，所以**白底下线条照常可见**。用户看到的不是"一整片白"，而是
**"网格清清楚楚地画着，晕整个没了"**——这个形态比全白更像软件坏了，用户不会把它读成
"我选了一个不合适的背景色"。

**这条决定了告警不是可选项**（见 §7 的余量谓词）：一个让人以为软件坏了的退化态，必须在进入之前
就被指出来，而不是等用户自己反推。

## 3. 两条法则

### 3.1 `tone: screen`（默认，行为完全不变）

```
out = clamp(L·c + bg, 0, 1)
```

其中 `L·c` 是经 `ExposureScale()` 定标后的辐亮度项（真实光谱色走 gamut clip → XYZ→RGB；
`ray_color` 非 sentinel 时走 D65 灰 → XYZ→RGB → tint），`bg` = `background`（linear RGB）。

### 3.2 `tone: print`（新增）

```
e     = Y · ExposureScale()          // 单个标量，CIE Y
D     = γ · log10(1 + e)             // 光学密度
out_j = paper_j · 10^(-D)            // linear RGB，其后与 screen 共用同一段 clamp + sRGB gamma
γ     = 11                           // 定案值，当前不暴露为用户旋钮
```

- `e` 是**曝光标量**，与既有灰度分支消费的量完全相同：`render.cpp:838-840` 把
  `snapshot_xyz_[i*3+1] * scale` 写进 `xyz[1]`，而 `scale = ExposureScale()`（`render.cpp:680`）。
  GUI 侧的同一量是 `tex_color.y * rel_illum * u_intensity_scale`
  （`preview_renderer.cpp:151` 的 `xyz *= u_intensity_scale;` 与 `:672` 的 `rel_illum` 相乘）。
- `paper` 是新字段（见 §5），linear RGB，JSON 键为 sRGB 并在解析时转换——与 `background_` 同约定
  （`render_config.hpp:171` 的注释：`// Linear RGB. The JSON "background" key is sRGB`）。
- 密度作用在 **linear** 域，然后与 screen 走同一段 clamp + `LinearToSrgb`。反射率 `10^(-D)` 本身就是
  一个线性量，放在 gamma 之后是错的域。

**`γ = 11` 是定案值**，来自一次对已捕获 raw XYZ 的离线标定，方法是数据驱动的百分位锚点：
取每个场景中**非零 `e`** 的 P95 百分位，反解使其落到目标密度 `D = 0.5`（约 32% 反射率，一个可读的深灰），
在 4 个物理差异较大的场景（宽视场全天 × 两种 `ev_mode` / 窄视场 / 高饱和 CZA）上分别求得
13.76 / 21.91 / 10.42 / 4.68，取几何均值 ≈ 11。用共享的 `γ = 11` 统一渲染这 4 个场景，halo 环、
窄视场 halo 盘边界、CZA 弧均清晰可读，背景空白区仍接近纸白，无一场景过曝到实心黑或欠曝到不可读。
成品形态已由 owner 眼判接受。

⚠️ **不要凭直觉挑 γ**：真实曝光标量 `e` 在这套场景下典型只有 1e-3 ~ 1e-1 量级，不是量纲上贴近 1 的
"标准化"值。`γ ∈ [0.3, 2.0]`（"温和调子曲线"的直觉区间）在**全部**场景上输出接近纯白，会让人得到
"这个公式不 work"的假结论。合理量级是 O(10)。这是本设计最先踩过的坑。

⛔ **实现途中不得自行重标定、改档，或把 `γ` 写成"待定"**。它的可调性是一条独立的、有意推迟的条目，
见 §9.2。

## 4. 为什么必须是两个算子，而不是一个旋钮

`10^(-D) ≤ 1` 对任何 `D ≥ 0` 恒成立，而 `e ≥ 0 ⇒ D ≥ 0`。所以

> **减色路的输出永远不可能亮过纸色。**

加性路则相反：输出对辐亮度单调不减，起点是 `bg` 而向上无界（直到 clamp）。两者是**结构性的非包含关系**——
加性不是减色的某个参数区间，减色也不是加性的。没有一组参数能让一条法则退化成另一条。

**结论（owner 裁决 D1）**：模式枚举躲不掉，`tone: screen | print` 是一个**显式开关**；
也正因如此，⛔ **不做"按背景色自动切换"**——那会让"我把背景调亮了一点"这个连续动作在某个阈值上
突然换掉整条显示法则，是一次隐式的、用户无法预测的行为跳变。

## 5. 为什么是灰度，以及放弃了什么

**逐像素色度会把 `κ`（墨的吸收色）变成一个需要归一化的逐像素量，并带一个直觉陷阱。**

最自然的直觉模型是"墨吸收互补色"：`κ = 1 - ĉ`，其中 `ĉ` 是该像素归一化后的光线颜色。
这个模型在彩色弧上说得通，但对**中性色**是灾难：白光 `ĉ ≈ (1,1,1) ⇒ κ ≈ 0`，即"不吸收任何东西" ⇒
**幻日环、日柱、22° 晕外侧这些中性特征在白纸上完全消失**。而这些恰恰是印刷插图里最常要展示的结构。

**所选方案：把 `κ` 降为常量**（灰度墨），陷阱随之消失——`D` 是一个标量，`out_j = paper_j · 10^(-D)`
对三个通道同一比例，纸色是什么色就往什么色的暗端走。

**代价（owner 裁决 D2，主动放弃）**：色相维度没有了。印刷图里**"哪条弧"只能靠位置和形状读**，
不能靠颜色读。CZA / CHA 这类高能量高饱和弧在减色下本来就难看，这个维度是被有意放弃的，不是遗漏。

**同族的一个已接受取舍**：`γ = 11` 下 CZA 场景的近日点核心会趋近实心黑（其强反射核心比其余场景亮
5–20×）。owner 已裁定这是可接受代价，⛔ 本设计范围内**不引入局部密度压缩一类专门机制**。
背后的物理是"强反射核心 + 远处暗弧共存"——同一场景若改用 P50(非零) → `D = 0.3` 锚定会解出 `γ ≈ 187`，
与 P95 锚定的 4.68 相差近 40×。这个跨度是该类场景的物理特性，不是标定方法的缺陷。

## 6. 两个字段：`background`（天空）与 `paper`（纸）

`render.background` 默认黑，`render.paper` 默认白，**两个独立字段**（owner 裁决 D5）。

**为什么不能共用一个字段**：共用时 `tone: print` + 默认黑背景 ⇒ `paper` 为黑 ⇒ 整页纯黑，
即一个新用户切到 print 的第一眼就是全黑。所有补救路径都不可接受：

| 补救 | 为什么不行 |
|---|---|
| 切换 mode 时自动改写该字段 | 静默覆盖用户已经设过的值；且不可逆（切回来时该还原成什么？） |
| 默认值随 mode 变 | "字段的默认值取决于另一个字段"是隐式规则，且对已显式设过值的文档不生效，等于只在一半情况下工作 |
| 事后告警 | 把一个结构上可以不存在的退化态变成一条要用户读的提示 |

**拆成两个字段让这个退化态结构上不存在**：切到 print 时 `paper` 取它自己的默认值（白），
`background` 保持用户设的值原封不动（在 print 下不参与法则，见 §7），切回 screen 时它还在。

字段链落点（下游实现须逐个穿过）：`RenderConfig`（含 `render_config.hpp:296`
`RenderConfigFieldSetGuard` 的结构化绑定字段集哨兵与两处 `sizeof(RenderConfig)` pin，
见 `render_config.hpp:280` 起的注释）、JSON schema、`LUMICE_RenderParam`、GUI state、
`file_io` 导入导出。`tone` 与 `paper` 都是 **appearance 字段**，不改累积布局 ⇒
`NeedsRebuild`（`render_config.hpp:325`）必须**不**因它们返回 true。
C API struct 追加成员 ⇒ 版本号 bump。

## 7. 一条规则：**print 模式下，ink 接管色彩通道**

print 把输出的色度自由度整个消耗掉了：每个像素的颜色由 `paper` 与一个标量密度决定，没有第二个色度输入
可以叠上去而不破坏"密度即能量"的语义。这不是四个平行的特例，是**一条规则的四个实例**：

| # | 实例 | 位置 | print 下的处置 |
|---|---|---|---|
| 1 | 背景照片 overlay | `preview_renderer.cpp:716-722`（`u_bg_enabled` / `u_overlay_alpha`） | **互斥**。叠实拍照片的语义前提是"往照片上加光"，与减色方向相反 |
| 2 | raypath 染色（合成路） | `component_compositor.cpp` | **互斥**。色相就是这条功能的载荷，灰度墨下它无处可去 |
| 3 | `ray_color` | `render_config.hpp:172` | **互斥**。见 §1 的推论：它在减色路里既不是 tint 也不是墨色 |
| 4 | annotation 颜色字段 | `render.cpp:99-100` `kOutlineSrgb`；`GridLineParam::color_` / `ZenithNadirParam::color_`（`render_config.hpp:39` / `:74`） | **改走密度**：annotation 在 print 下按同一条密度法则落墨，`out_j = paper_j · 10^(-D_line)` |

第 4 项的形态是关键，它决定了 print **不需要 per-mode 调色板**：如果 annotation 保留自己的颜色字段，
就得为 print 准备第二套线条颜色（"Print 预设"），而预设一旦要写入用户字段就带回覆盖用户值 / 影子状态
的老问题。让 annotation 走密度，这一整类问题不存在。

前三项的处置是**互斥 + 告警**，不是静默忽略：用户开着染色切到 print，画面会失去颜色载荷，
必须被告知。

## 8. 余量谓词：一个纯函数，两侧共用

两个模式各有一个退化方向，但它们是**同一个思路**（owner 裁决 D6），必须实现为**同一个谓词**：

| 模式 | 零能量色 | 余量 | 退化态 |
|---|---|---|---|
| screen | `background` | `1 - background` | 白底 ⇒ 余量 → 0 ⇒ 晕不可见（§1） |
| print | `paper` | `paper` 本身 | 黑纸 ⇒ 余量 → 0 ⇒ 全页黑，落墨不可见 |

"零能量色"= 一个不接收任何光线能量的像素在该法则下的输出。余量 = 从零能量色到法则另一端的可用距离。
两侧都是**逐通道取值、跨通道取最小**（一个通道见底就已经开始丢结构）。

谓词是 `src/util/` 的一个**纯、无状态、不携带 simulation/config 语义**的函数，CLI 与 GUI 双侧消费——
这正好落在 `AGENTS.md` 写明的 API 边界具名豁免形状内（"一条 BOTH renderers 都必须遵守的规则，
可以不引用 core/config 类型表达"）。⛔ 不要在两侧各写一份。

告警形态（非阻塞提示 + 一键修 / CLI 侧告警）属下游实现范围，本文不定死交互细节；本文定死的是
**谓词只有一个，两个方向共用**。

## 9. 已识别、有意推迟的选项

以下两条是**独立条目**，各自有明确的重新评估触发条件。它们不是本设计没做完的部分。

### 9.1 灰度标量取 Y vs 等权 / 辐射量

当前取 **Y**（CIE 亮度，与 `anchor_l99_sky` 的锚一致，owner 裁决 D4）。备选是等权 RGB 标量
（`GamutClipXyz → XyzToLinearRgb` 后三通道均值）或纯辐射量。

⚠️ **证据边界，必须如实读**：离线对照在一个暖色光源场景上做过——该场景亮部像素 R:G:B 中位数
≈ 9914:4602:1183（红比蓝亮约 8.4×），Y 标量与等权标量的全帧百分位几乎重合、渲染图肉眼不可分辨。
**这只说明该场景没有触发这个问题，⛔ 不得被写成"已验证 Y 不压制蓝弧"。** 真正的验证需要一个
蓝紫主导光源 + 高色散弧的场景，目前不存在这样的夹具。

- 推迟理由：等真实用户信号，而不是等更多离线标定。
- 重新评估触发条件：**有用户反映特定颜色的弧线在 print 模式下印不出来**（典型是蓝端）。
- 届时的形态：可能作为一个 GUI 开关暴露。

### 9.2 `γ` 的可调 / 暴露为用户旋钮

`γ = 11` 今天是钉死的常量，不暴露（owner 裁决 D3）。

- 推迟理由：同上——先要真实用户信号，才知道该往哪个方向调、以及是否值得多一个旋钮。
- 重新评估触发条件：**内测用户反馈**。
- 复用资产：§3.2 的数据驱动锚点方法（取非零 `e` 的某百分位，反解使其落到目标密度）本身是可复用的
  标定手段，届时不必重新摸索——它比拍脑袋挑值可核验，也更容易在场景分布变化时重新标定。

## 10. 诚实边界（三条硬约束）

1. ⛔ **这是一个显示传递函数，不是印刷仿真。** 不碰 CMYK、不碰 ICC、不碰网点扩大、不碰纸白点。
   产物仍然是一张 RGB 图。命名同理：config 键名叫的是法则（`tone: print`），"Print" ⛔ 不作为
   一个写入其它字段的预设存在。

2. ⛔ **不改染色 composite 路的算子。** 减色会让密度可加、合成顺序无关，看起来"顺便把 painter 也
   改了更整体"——不要。`doc/gui-custom-spectrum-and-raypath-color.md` §4.8 是**有意**把 z-order
   抬为一等视觉控制的（painter = 亮度即 alpha 的 over 合成），减色会让 z-order 重新失去意义，
   等于反转一次已经定案的设计。本设计范围内 composite 路**只做互斥与告警**（§7 实例 2），不改算子。

3. ⛔ **曝光链一行不改。** `e = Y · ExposureScale()` 照旧，`anchor_l99_sky` / `ev_mode` /
   `emitted_energy` 全部不动，ink 只替换**末端的传递曲线**。`doc/ev-pipeline-architecture.md`
   是雷区，动它之前先读那一篇。**本设计的前提就是不动它**——任何实现若发现"必须动曝光链"，
   属于前提被证伪 ⇒ 上抛，⛔ 不得自行扩范围。

## 11. 决策归属

### 11.1 owner 已裁决（⛔ 不得自行推翻，如有异议上抛）

> **as-built（2026-09-10，全部五块下游已落地）**：下表第三列是每条裁决在代码里的落点。
> ⚠️ 行号会漂，符号名不会——找不到行就按符号名 grep。

| # | 决策 | 落地位置（as-built） |
|---|------|---------------------|
| D1 | print 是一个**显式开关**，不是按背景色自动切换；两条法则不可统一为一个滑杆 | `src/config/render_config.hpp` 的 `enum Tone { kScreen, kPrint }`（:164，注释写明"structurally non-overlapping, not two parameter ranges"）。全树唯一会自己改写 `tone` 的代码路径是 `ApplyHeadroomFix`（`src/gui/gui_state.hpp`:516），而它挂在告警旁边那个**要用户点的按钮**上——没有任何路径读背景色就切模式 |
| D2 | print 模式**只做灰度**，不做逐像素色度（CZA/CHA 这类高能量高饱和弧在减色下会难看，主动放弃该维度） | `src/server/render.cpp` 的 `if (print_mode)` 分支（:905–916）：跳过 gamut clip、XYZ→RGB 矩阵与 `ray_color_`，只取标量 `xyz[1]`。GUI 侧对偶为 `subtractiveInk()`（`src/gui/preview_renderer.cpp`:197 起） |
| D3 | `γ`（密度斜率）与 `s`（色相倾斜）**先钉默认值**，不暴露为用户旋钮 | `γ` = `inline constexpr float kInkGamma = 11.0f`（`src/util/ink_transfer.hpp`:41），**不是** config 字段、不是函数参数，也没有对应 GUI 控件；`test/unit-correctness/util/test_ink_transfer.cpp` 把它钉住，所以改它只能是有意的。`s` 按本节脚注的结论在代码里不存在（被 D2 架空） |
| D4 | 灰度标量**先用 Y**；等权/辐射量作为已识别的推迟选项记入本文，将来可能作为 GUI 开关 | `src/server/render.cpp`:912 `const float e = paint_bg ? xyz[1] : 0.0f;`（`xyz[1]` = CIE Y，已乘过 `scale`）。等权/辐射量没有实现，仍只是 §9.1 的推迟选项 |
| D5 | `background`（天空）与 `paper`（纸）**拆成两个字段**，而非共用一个 | `src/config/render_config.hpp`:183 `float background_[3]{}`（默认黑）与 :193 `float paper_[3]{1,1,1}`（默认白）两个独立字段，默认值刻意相反 |
| D6 | screen 白底告警与 print 黑纸告警是**同一个思路**，应实现为同一个谓词 | `src/util/contrast_headroom.hpp` 的 `ContrastHeadroomMargin` / `ContrastHeadroomIsLow`（:92/:108）是唯一谓词；CLI 侧 `src/server/server.cpp`:741、GUI 侧 `src/gui/gui_state.hpp`:491–499 的 `ContrastHeadroomMarginFor`/`ContrastHeadroomIsLowFor` 各自只是把"哪一边是零能量色"喂进去（print→`paper`，screen→`background`），判据本体一份 |

> D3 的 `s`（色相倾斜）是 owner 原始裁决表的逐字措辞（本文按 §11 开篇的复用纪律原样保留），
> 但**已被 D2 架空**：D2 裁定 print 模式只做灰度、主动放弃色相维度（`κ` 降为常量，见 §5），
> 灰度方案下没有色相可倾斜。`s` 在本设计范围内不存在对应参数，⛔ 不要在下游任务里去别处
> （代码库或本 scrum 的其他制品）寻找它的定义或实现落点——它是早期裁决的历史遗留符号，仅
> 因忠实转录 owner 原文而保留在表中，本文档不为它另行定义语义或触发条件。

附带的排期裁决（对下游有约束力）：`γ = 11` 是**定案值**，按它实现；"调整 `γ` / 把 `γ` 暴露为旋钮"
是一条触发条件明确的将来任务（触发 = 内测用户反馈），⛔ 不得作为本设计的遗留项挂着（见 §9.2）。

### 11.2 AI 推导（⚠️ 可质疑；被实测证伪时不要迁就本文，按证据上抛并说明）

> **as-built（2026-09-10）**：五条推断已逐条落地核对。被证伪的两条**保留原推断文字不动**，在"实际结论"
> 里写明实际形态——本仓的既有做法是保留自我纠正链，而不是把猜错的那一版抹掉改写成"本来就是这样"。

| # | 推断 | 若被证伪的影响面 | 验证状态 |
|---|------|------------------|----------|
| A1 | 传递曲线取 `D = γ·log10(1+e)`，`out_j = paper_j · 10^(-D)` | 换曲线不改本设计的结构 | ✅ 已由离线标定在 4 场景上验证并经 owner 眼判接受（`γ = 11`，§3.2）。**as-built**：`src/util/ink_transfer.hpp` 的 `InkOpticalDensity` / `InkTransmittance` 是 C++ 侧单一 owner，`render.cpp`:913 调用它；GLSL 第三份实现是手抄（`preview_renderer.cpp`:197 起，注释指回权威） |
| A2 | 灰度标量可直接复用 `render.cpp:843` 起 `use_real_color == false` 分支的前半段（取 `xyz[1]` → D65 灰） | 若复用不成立，算子落地的规模上升 | ⚠️ **部分证伪，但代价反向**：实际没有复用那个分支的任何代码——print 分支（`render.cpp`:905）自己取 `xyz[1]` 直接送进传递曲线，**跳过了 D65 白点矩阵乘法**（那个矩阵只在需要 RGB 色度时才必要，print 只需要标量）。A2 预判的"若复用不成立，规模上升"**没有发生**：实际分支比它要复用的那半段更短 |
| A3 | annotation 层在 print 下改走**密度乘法**，因而**不需要** per-mode 调色板；"ink 接管色彩通道"这条规则覆盖四个实例 | 若不成立，需回退到"Print 预设写入线条颜色"，那会带回覆盖用户值/影子状态问题 | ✅ **成立**。`BlendAnnotation`（`render.cpp`:123–125）：`print_mode ? base*(1-alpha) : base*(1-alpha) + line_rgb*alpha`，CLI 侧 7 个调用点（grid / angular_dist / horizon / markers / `PaintLabels`）全部收敛于它；GUI 侧同形的 `blendAnnotationColor()`（`preview_renderer.cpp`:215）有 8 个调用点。全程未新增任何 print 专用调色板字段 |
| A4 | 余量谓词可做成 `src/util/` 纯函数、GUI 与 CLI 共用 | 影响谓词的落点，不影响它的存在性 | ✅ **成立**。`src/util/contrast_headroom.hpp`，无 core/config 类型依赖，`server.cpp` 与 `gui_state.hpp` 双侧调用同一实现（落点见 §11.1 的 D6 行） |
| A5 | 被 mask 区域可统一表述为"零能量"，`render.cpp:912` 的 `rgb[j] = 0.0f;` 特例随之消失 | 若不成立，print 下 mask 外的颜色需单独裁决 | ⚠️ **机制被证伪，效果达成**：那行 `rgb[j] = 0.0f;` **没有消失**（现 `render.cpp`:992），而是被 `if (!print_mode)`（:977）卫护起来、只在 screen 下执行；print 走的是另一条路——`paint_bg == false` 时 `e = 0.0f`（:912），经 `InkTransmittance(InkOpticalDensity(0)) == 1` 落地为**纸的原色**。所以 A5 的主张（print 下 mask 区域是零能量色 = 纸白，不是黑）成立，但实现形态是"新增一条独立分支"而不是"消除旧特例"，与 A5 字面预测的机制不同 |

关于 A5 的一个已知后果，实现时须一并核对：统一为"零能量"之后，print 下被 mask 的区域
（镜头成像圆之外、`visible` 舍弃的半天、`front` 之后）渲染为 `D = 0 ⇒ out = paper`，
即**纸白**，而不是 screen 下的黑。对印刷产物这多半是想要的（页面上不该有一大块黑矩形），
但它同时意味着成像圆边界在 print 下失去了视觉分界，除非画出镜头边框。这是 A5 的验证负担之一，
不是可以默认成立的推论。

A5 那条已知后果（成像圆边界在 print 下失去视觉分界，除非画镜头边框）**仍然成立且仍未处理**——
后续四个子任务都没有碰它，保留为已知边界，见上一段。

**跨切验收闸**（只有整个模式建成之后才可能存在的那一类）落在两处：
`test/unit-correctness/server/test_print_mode_dynamic_range_gate.cpp`——四个数量级动态范围在 8-bit
输出里的可分辨性，print 全分开、screen 白底全塌成同一个白，即 §1 的诊断被操作化成一条可执行断言；
以及 `test/gui/parity/test_gui_cli_export_parity.cpp` 的 `full_sky_dual_fisheye_print` 场景——GLSL 那份
手抄实现与 C++ 权威之间的漂移，只能靠同一份文档经 CLI 与 GUI 预览两条生产路各渲一次再互比来兜住。
⚠️ 后者所属的 `parity` tag **没有任何 CI job 在跑**（`doc/testing-architecture.md` §7.5），只在有 GL
context 的开发机上经 `./scripts/test.sh {quick,full,pr}` 求值——不要因为流水线绿就以为它跑过。
