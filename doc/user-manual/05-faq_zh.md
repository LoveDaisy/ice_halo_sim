[English version](05-faq.md)

# FAQ、默认值与已知限制

一些"新手常被绊住的点"汇总。扫一眼标题，挑你关心的看。

## 1. 配置默认值速查

Lumice 的 JSON schema 只要求少量字段，其他字段都有默认值。下表列出最常被问的几项（完整 schema 见 [`../configuration_zh.md`](../configuration_zh.md)）。

| 字段 | 是否必需 | 默认值 | 说明 |
|------|----------|--------|------|
| `scene.ray_num` | 是 | — | 每个波长追踪的光线数；可用 `"infinite"` 跑无限模拟 |
| `scene.max_hits` | 是 | — | 单条光线允许的最大内部反射次数 |
| `scene.light_source.altitude` | 是 | — | 太阳高度角（度） |
| `scene.light_source.azimuth` | 否 | `0.0` | 太阳方位角（度） |
| `scene.light_source.diameter` | 否 | `0.5` | 太阳角直径（度），`0.5` ≈ 真实太阳 |
| `scene.light_source.spectrum` | 是 | — | 取 CIE 标准光源名（`"D65"` 等）或 `[{wavelength, weight}, …]` 数组 |
| `crystal[].axis` | 否 | `{zenith: 90, azimuth: 0, roll: 0}` | 默认"水平柱状"取向 |
| `render[].lens.type` | 否 | `"linear"` | 其他选项：`fisheye_*`、`dual_fisheye_*` 系列投影 |
| `render[].lens.fov` | 否 | `90.0` | 对角视场角（度）；`rectangular` / `dual_*` 镜头忽略此字段 |
| `render[].view.{azimuth,elevation,roll}` | 否 | 各 `0.0` | 相机相对世界系的指向 |
| `render[].visible` | 否 | `"upper"` | 半球遮罩：`"upper"` / `"lower"` / `"full"` |
| `render[].background` | 否 | `[0, 0, 0]` | RGB 背景 |
| `render[].opacity` | 否 | `1.0` | 渲染不透明度 |

> 字段名为 JSON parser 实际接受的名称。C API 镜像了其中一部分 — 见 [`../c_api_zh.md`](../c_api_zh.md)。

## 2. `ray_num` × 波长语义 — "光线数"到底是什么意思？

**简短回答**：在离散 spectrum 模式（典型场景）下，`ray_num` 是**每个波长**追踪的光线数。总工作量 = `ray_num × N(wavelengths)`。

**为什么重要**：你把单波长换成 9 段 spectrum 后没改 `ray_num`，模拟会慢约 9 倍。新手常误以为是 config bug — 不是 bug，是积分方案就这样。

**源代码事实**（`src/core/simulator.cpp:482-498`）：

```cpp
const auto& spectrum = config.light_source_.spectrum_;
if (auto* illuminant = std::get_if<IlluminantType>(&spectrum)) {
  // 标准光源（如 D65）：每个 batch 随机选一个波长
  float wl = 380.0f + rng_.GetUniform() * 400.0f;
  float weight = GetIlluminantSpd(*illuminant, wl);
  SimulateOneWavelength(config, WlParam{ wl, weight }, batch.ray_num_, ...);
} else {
  // 离散波长：每个波长各跑 ray_num 条光线
  const auto& wl_params = std::get<std::vector<WlParam>>(spectrum);
  for (const auto& wl_param : wl_params) {
    SimulateOneWavelength(config, wl_param, batch.ray_num_, ...);
  }
}
```

两种 spectrum 模式的开销曲线不同：

- **离散波长**（`spectrum: [{wavelength, weight}, ...]`）：开销 ≈ `ray_num × N(wavelengths)`。内置示例 `examples/config_example.json` 用 9 段 × `ray_num=5e7` ⇒ 4.5 × 10⁸ 条光线。
- **标准光源**（`spectrum: "D65"`、`"D50"` 等）：开销 ≈ `ray_num`。每个 batch 在 `[380, 780]` 内按 SPD 加权随机选 1 个波长。

**`batch.ray_num_` 的语义桥接**：服务端 `GenerateScene` 将用户的 `scene.ray_num` 切分为多个 batch（`batch_ray_num = min(kDefaultRayNum, remaining)`），逐批投递给 simulator。每批对每个波长各跑 `batch.ray_num_` 条光线，所有 batch 累加后等于用户配置的总 `ray_num`。代码注释 `src/config/proj_config.hpp:28` 直接写道：`// For every single wavelength.`

**实操建议**：

- 首跑？`ray_num=1e6` + 单波长（`[{"wavelength": 550, "weight": 1.0}]`），几秒出图。
- 想要低噪最终图？`ray_num=5e7` + 完整离散 spectrum 是常见配方。
- 想 GUI 内连续累积预览？`ray_num: "infinite"`，看够了手动停。

## 3. GUI 与 JSON 能力差异

GUI 包了同一个引擎，但**并非** JSON config 的完全超集 — 一些能力只在 JSON 里有，一些状态只在 GUI 里有但**不会**被持久化。

| 能力 | GUI | JSON / CLI | 说明 |
|------|-----|------------|------|
| 多 `render` 条目 | ❌（只一个预览）| ✅ | JSON 可声明任意多个 `render: [...]`；CLI 每个 entry 出一张图 |
| 多层 scattering（≥ 2 个 entry） | ❌ | ✅ | 配方 3 在 [`04-recipes_zh.md`](04-recipes_zh.md) 中要求 JSON |
| 镜头投影切换 | ✅（浮动镜头条）| ✅（`render[].lens.type`） | GUI 实时重投影；JSON 一次设定每个 render 的镜头 |
| Overlay 网格 | ✅（实时，仅 GUI） | 部分通过 `render[].grid` | GUI overlay 是观察辅助，**不**写进 `.lmc` |
| 晶体预览样式（线框 / 隐藏线 / 透视 / 着色）| ✅ | ❌ | 纯 GUI 状态，与模拟无关 |
| `.lmc` 保存/加载 | ✅ | ✅ | `.lmc` 即 JSON，两边读同一个文件 |

经验法则：**交互式探索**走 GUI，**批处理 / 可复现的跑**走 JSON + CLI。

## 4. 字段命名疑问

少数 JSON 字段在 schema 里存在但当前实现未生效（前向兼容）。parser 不会报错，引擎也不会使用 — 它们就是没效果而已：

- `render[].grid` — 引擎只渲染模拟数据；网格 overlay 是 GUI 在渲染图上后叠加的。无头 CLI 渲染路径不消费 JSON 里的 `grid` 块。
- 个别 `crystal[].shape` 分布字段在异常组合下不生效 — 详见与文档任务并行维护的 `known_mismatch.md` 登记。

如果你"在 JSON 里设了 X 但没效果"，先查这一节，再查 [`../configuration_zh.md`](../configuration_zh.md)，最后再开 issue。

## 5. 输出文件去哪儿了？

- CLI：写到 `-o <dir>` 指定的目录（默认当前目录）。每个 `render` entry 一张图，命名 `<render_id>.<format>`（默认 `jpg`，可用 `--format png` 切到 PNG）。
- GUI：渲染图存内存里，`File ▶ Export` 才落盘。GUI 不会自动写文件。

## 6. "我的光晕看上去淡 / 噪点多 / 不对" — 排查清单

1. **太淡？** 升 `scene.ray_num`（光线数 ×4 ≈ 噪声 ÷2）。
2. **噪点多？** 升 `scene.ray_num`，或者把 spectrum 收窄一点（如果色散噪声占主导）。
3. **完全没光晕？** 确认相机指向太阳：`render[].view.elevation` ≈ `scene.light_source.altitude`，`view.azimuth` ≈ `light_source.azimuth`。
4. **形状不对？** 检查 `crystal[].axis` — 随机取向出圆环，定向片晶出弧线和亮斑。
5. **还是不对？** 用 `-v` 重跑，看每个 batch 是否有零命中。

## 延伸阅读

- 完整 schema → [`../configuration_zh.md`](../configuration_zh.md)
- 坐标约定（高度角 / 天顶角 / 方位角的符号） → [`../coordinate-convention_zh.md`](../coordinate-convention_zh.md)
- 性能调优 → [`../performance-testing_zh.md`](../performance-testing_zh.md)
- GUI 面板参考 → [`../gui-guide_zh.md`](../gui-guide_zh.md)
- 嵌入用 C API → [`../c_api_zh.md`](../c_api_zh.md)
