# 设计：GUI 自定义光谱 + per-raypath 颜色标记

状态：
- **功能 1（自定义离散光谱）已由 task-323 落地**（2026-07-03，见 `scratchpad/task-gui-custom-spectrum/`；同批完成 `ray_num` 语义统一为总数）。
- **功能 2（per-raypath 颜色标记）仍在 backlog**，架构留白，未立项。

本文档保留原设计，作为功能 2 的接手蓝图；功能 1 部分见 `doc/configuration.md` 的 `ray_num` 语义章节与 `src/gui/edit_modals.cpp::RenderSpectrumModal` 的 as-built 实现。

---

## 0. 背景与两个功能

owner 计划扩展两个相关但不同的功能：

1. **自定义光谱**：让用户在 GUI 里配置离散波长 + 权重（本质是把 config JSON 早已支持的能力带回 GUI）。
2. **per-raypath 颜色标记**：对不同 filter/raypath 结果用不同颜色标记（如 3-5 红、1-3 蓝）。启用颜色标记时**覆盖初始光谱颜色，仅取强度信号（Y 分量）来染色**。

两者在"强度依赖光谱"这一点耦合，但工作量差一个数量级：功能1 是补齐一条已存在的能力管道；功能2 是新增一个横切 core + GPU seam + GUI 合成器的可视化子系统。

---

## 1. 总纲：两种角色，别混（本设计的地基）

从用户视角只有一个"filter"概念，但实现上必须区分两种角色：

| | **Design A filter**（保留不动） | **Color Classifier 颜色分类器**（新增） |
|---|---|---|
| 语义 | 物理门，决定光线**是否传播** | 可视化分区，把**同一物理种群**打标签 |
| 对光线 | **破坏性丢弃**（`w_ = -1`） | 非破坏，只 tag |
| 绑定 | per-crystal，1:1（`crystal_id ↔ filter_id`） | **全局**，N 个并存 |
| 位置 | 仿真器 emit gate（`CollectData`） | 同一 gate 评估谓词 → consumer 路由 → display 合成 |
| 改动触发重仿真 | 改→重仿真 | **改谓词→重仿真；改颜色/开关→纯显示，免重仿真** |

**为什么必须分开**：Design A filter 是物理门，会改变传播到下一 MS 层的种群。用户要的"把打到屏幕的光按 raypath 家族染色"，前提是**一次物理仿真、固定种群、事后分区**。若用"每色跑一次 Design-A filter trace"来实现（下节路线A），在多散射下 N 次是 N 个**不同的物理仿真**而非一个种群的分区，会重复计数/错分。混用还会重蹈 task-200（把物理门挪去 consumer 侧 → 三 bug + 丢多层剪枝 → revert）的覆辙。

---

## 2. 当前架构现状（三个 explore 实测，2026-07-03）

### 2.1 光谱 / 颜色管道
- core + config-JSON 层**完全支持**自定义离散光谱：`SpectrumConfig = std::variant<std::vector<WlParam>, IlluminantType>`（`src/config/light_config.hpp:24`），JSON 字段 `wavelength`/`weight`，`examples/config_example.json` 就用离散数组。
- 波长→XYZ 是线性叠加 `CMF(λ)×weight`（`src/core/color_util.hpp:29-60`，`SpectrumToXyz` / `SpectrumToXyzPerRay`）。
- 波长粒度依后端：离散列表 → per-(batch×wl) 遍历（CPU）；illuminant → per-batch 单随机波长（CPU）或 per-ray wl pool（Metal/CUDA，`wl_idx` 是 uint8，上限 255，默认 64，`src/core/backend/wl_pool.hpp`）。
- **缺口**：flat `LUMICE_Config.spectrum` 是 `const char*`，只接受 `"D65"/"D50"/"A"/"E"`，传数组直接 `LUMICE_ERR_INVALID_VALUE`（`src/server/c_api.cpp:523-531`）；core 支持的 D55/D75 都没进这个 map。离散数组能力**只**存在于全 JSON 路径（`LUMICE_CommitConfig`/`FromFile`），绕过 `LUMICE_Config` struct。
- **副产品发现**：当前**无色适应/参考白解耦**，白点钉死 D65 sRGB 矩阵（`render.cpp` + `color_data.hpp:6-12`）。选 illuminant A(2856K)/自定义暖谱会渲染成偏橙而非归一化中性白。→ 独立议题，本设计不含。

### 2.2 filter / accumulator / consumer
- filter = per-crystal 物理门。config 类型 `FilterConfig`（`src/config/filter_config.hpp:46-61`），谓词有 raypath / entry-exit / direction / crystal / complex（sum-of-products，但**塌缩成单一布尔**）。
- gate 在 `CollectData()`（`src/core/simulator.cpp:449-493`，check 在 :466）：pass+prob-pass→continue；pass+prob-fail→emit；**fail→丢弃**（复用 TIR sentinel `w_ = -1`）。这一步**破坏性抹掉分类信息**。
- consumer（`RenderConsumer`）buffer 模型 = **单条 XYZ lane**（`internal_xyz_` W×H×3 float，`src/server/render.hpp:41-45`），eager 分配正好一条。**无 anchor lane、无 per-filter channel**。
- 多图输出的 fan-out 轴是 **`RenderConfig`（view/lens），不是 filter**：一次 run 出 N 图 = N 个 RenderConfig，**所有 consumer 拿同一份已过滤光线流**（`server.cpp:840-842`）。
- **结论**：今天一次 trace **无法**喂 N 个 per-filter buffer——分类在 gate（`simulator.cpp:466-477`）就被销毁。要 N 桶必须把 filter 从"破坏性丢弃"改成"非破坏 per-ray 分类"，随出射流携带。

### 2.3 GUI 现状
- 光源面板 `RenderSceneControls`（`src/gui/panels.cpp:890`）：只有 altitude/diameter slider + 6 预设 spectrum combo（`spectrum_index` int，`gui_state.hpp:76-80,138-139`）。**无光谱编辑、无色温控件**。
- filter 编辑 `RenderFilterModal`（`src/gui/edit_modals.cpp:900`）：per-entry 卡片里编辑，谓词 = raypath / entry-exit + action(In/Out) + 对称 P/B/D。**`FilterConfig` 无任何颜色字段**。多 filter 已通过 pool + entry 引用 **end-to-end round-trip**（`file_io.cpp` load/save）。
- 提交路径：主走 struct（`FillLumiceConfig` + `LUMICE_CommitConfigStruct`，`app.cpp:755`），另有 JSON 路径（`LUMICE_CommitConfig`，`app.cpp:749`）。
- 预览 = **单纹理单 RenderConfig**：poller 写死 `LUMICE_GetRawXyzResults(..., 1)` 只读 `xyz_results[0]`（`server_poller.cpp:150-166`），GUI 显式忽略多余 renderer（`file_io.cpp:1398-1401`）。shader 有 overlay 机制但只是背景图 blend + 线/网格/label，**不是多色合成器**。
- 自定义光谱在 GUI/C API/load/save **任何一层都不 round-trip**（load 只认 string，未识别 fallback D65）。

---

## 3. 功能1 设计：自定义光谱（小）

core 就绪，缺口全在上层三处：GUI state、C API、file_io 两条 load/save 路径。

**做法**：
1. `SunConfig::spectrum_index`（`gui_state.hpp:79`）升级成"预设名 **或** 离散 (wl,weight) 列表"的 variant，镜像 core 的 `SpectrumConfig`。
2. C API `LUMICE_Config` 加离散数组载体（**决策见 §5 #7：扩 struct，不走 JSON 侧信道**）。
3. `RenderSceneControls` 加编辑器：一张 wl/weight 表格（增删行）+ 从预设导入按钮。
4. `file_io.cpp` 两条路径补数组序列化。

**注意点**：
- **成本维度**：离散波长数直接乘仿真量（CPU per-(batch×wl)；GPU wl pool 上限 255/默认 64）。UI 需给波长数合理上界/提示，避免用户填几百条自伤。
- 顺带可修 C API 的 D55/D75 缺失。

---

## 4. 功能2 设计：per-raypath 颜色标记（大，横切子系统）

### 4.1 唯一正解的数据流（路线B）

```
emit gate (CollectData, simulator.cpp:466 —— raypath rp_ 此刻在手,
           正是 Design-A filter check 和 §5.1 device filter-match 的同一点)
   │ 评估 N 个 classifier 谓词
   ▼ 产出 per-ray bitmask，随 outgoing 光线发出
     （出射 SimData 新增一个 per-ray 字段，类比已有的 wl_idx）
consumer: 不碰谓词，只按 bitmask 把 Y 路由进 N 条标量-Y lane
   │ （views × classifiers 两个正交轴；bitmask 随光线走，各 view 各自路由）
   ▼
display 合成: color(px) = Σ_bucket  bucket_color × toneMap(Y_bucket[px])
EV anchor  = Σ_bucket Y  = 未分类图的 Y  → 开/关颜色标记 EV 都稳
```

**被否的路线A**（每色跑一次 Design-A filter trace 再合成）：多散射下语义错误（见 §1）。单散射下虽是合法分区但 N× 浪费。故一律用路线B。

### 4.2 关键性质（必然结论，非岔路）

- **谓词 = trace-time**（要上传 device 当分类表，改谓词→重仿真，与现有 filter dirty 语义 §6.3 一致）；**颜色 + 开关 = display-time**（改 shader uniform，即时，不重仿真）。
- **"轻量"的边界（owner 关注点，务必如实）**：
  - 调颜色 / 开关某家族 = 真·轻量（display-time）✅。
  - 新增 / 改一条 raypath 谓词 = **需重仿真**（bitmask 在 gate 算出，谓词变了要重算）。与"改 filter 就重仿真"一致，不是新负担。
  - 想连谓词都免重仿真，唯一办法是 consumer 保留每条光线完整 raypath 到显示时再分类——要放弃"累加进像素即丢弃"的流式模型、改成"留住全部光线"，**内存不可行，已否**。
  - 故模型是：**"哪些家族存在" = 较重偶发动作（重仿真）；"给已有家族调色" = 高频轻量动作（免重仿真）**。
- **"只取 Y" 确实省钱**：每桶是 W×H 标量 float，不是全 XYZ，N 条不心疼。但 **Y 仍依赖光谱**（亮度 = CMF-Y 加权），故功能1 的光谱仍喂功能2 的强度——两功能在此耦合。

### 4.3 真正的成本在哪（诚实评估）

不是 N 个 buffer（便宜），是：
1. **per-ray bitmask 字段穿过 outgoing SimData，在 CPU / Metal / CUDA 三后端的 emit gate 各自产出并保持一致**（类比已有 `wl_idx`，有界但跨 seam，是主要成本）。谓词 eval 复用 §5.1 already-validated 的 device filter-match 机制。
2. GUI：poller 的 `max_count=1` 放开读 N 条；preview 加**多色合成 shader pass**（今天只有单纹理 + 背景/网格 overlay，无合成器）。

### 4.4 GUI 数据模型

- filter pool 已 round-trip 是好地基，但**别把颜色分类器塞进 per-crystal 的 Design-A filter**（会把物理门与可视化搅在一起）。
- 新开**全局 "Color Layers" 面板**，复用现成 raypath/entry-exit 谓词编辑控件，独立全局列表，每项 = `{谓词, 颜色, 开关}`。

---

## 5. 决策台账（owner 2026-07-03 已同意整体设计）

| # | 岔路 | 决策 | 备注 |
|---|---|---|---|
| 1 | 两角色区分是否成立 | ✅ **成立** | 功能2 = 新增可视化分类层，不动 Design A |
| 2 | 未匹配光线怎么显示 | 倾向：真实光谱色 + 降亮 + "隐藏未匹配"开关 | **纯 UX，实现时再由 owner 最终确认** |
| 3 | 桶重叠（一线匹配多桶）| 倾向：叠加混色；优先级模式作后续 | **纯 UX，实现时再由 owner 最终确认** |
| 4 | 分类器谓词范围 | ✅ 复用 raypath/entry-exit（与 Design-A 谓词同源，借编辑控件） | |
| 5 | 分类器全局 vs per-crystal | ✅ **全局**可视化层 | |
| 6 | N 上限 | ✅ MVP **8**（bitmask 一字节装下） | |
| 7 | 光谱 C API | ✅ **扩 struct** 加离散数组载体 | 比走 JSON 侧信道干净、长期一致 |
| 8 | 参考白 / 色适应 | ✅ **本轮不做**，单独排 | 见 §2.1 副产品发现 |
| 9 | 分类 eval 落哪后端 | ✅ 三后端 gate 都要 | bitmask 必须跨 seam，不做 CPU-only 半成品 |

---

## 6. 推进节奏（建议，未立项）

两功能**先解耦、串行**，别捆一起：

1. **功能1 先做**（小、低风险，且其产出 Y 是功能2 的输入）——task 量级。
2. **功能2 拆两阶段**：
   - 阶段一：**core + CPU** 打通 bitmask → N-lane → consumer 路由 + 最小合成（可先 CLI 出 N 图验证语义正确），用 CPU 白盒把"分区语义对不对"这个最大正确性风险钉死。
   - 阶段二：上 Metal/CUDA gate + GUI 合成器，parity 锚定。

---

## 7. 顺带要修的债

- `doc/accumulator-consumer-architecture.md` 的 anchor-lane 章节（§6.1/§7.2 及 ON/OFF anchor 引用）是 **stale**——该 lane 早被 scrum-237 删除，`render.hpp` 无 anchor 成员。碰这块时顺手修，免误导后来人。

---

## 8. 关联

- explore 实测三份（2026-07-03，本 session）：光谱管道 / filter+accumulator / GUI 面板。
- doc：`filter-architecture.md`（Design A、§5.1 device filter-match、§6 task-200 revert 史）、`ev-pipeline-architecture.md`（单 lane、EV anchor）、`coordinate-convention.md`。
- 相关 memory：filter 物理门 vs 渲染分类的区分即本设计核心；参考白缺解耦为独立议题。
