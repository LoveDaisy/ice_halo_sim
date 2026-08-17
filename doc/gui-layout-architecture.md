# 设计：GUI 布局架构（面板组织形态）

> 状态：blueprint，已由可交互 HTML 原型验证并经 owner 上手定案（2026-08-14）。
> 形态层已定，**实现细节明确保留为落地自由度**（见 §6）——C++/ImGui 与 HTML 有真实差异，细节定死反而有害。
>
> 取证锚点：分支 `spike/gui-layout-prototype`（`65a41e19` 初版 → `ed46c1c1` 检视器并入文档列 → `64aa89ea` 双面板定案），原型本体在该分支 `spike-mockup/`（自包含单文件，浏览器直接打开）。
>
> 关联：`doc/gui-visual-language.md`（排版/色彩/尺寸节奏——那份管「长什么样」，本文管「放在哪」，两者正交且可分别落地）、`doc/gui-state-governance.md`（其 field→tier 分类器是本文组织原则的机制层依据）、`doc/gui-preview-lifecycle-architecture.md`（dirty 芯片所依赖的生命周期语义）。

## 0. 为什么重排面板

现状是左右两条侧栏：左栏以晶体卡片为主体（约 70% 空置），右栏一条长滚动列混装所有控件。这个形态的病根不是「侧栏」本身，而是**分割轴任意**：

- 晶体在左、太阳在右——两者都是场景定义（文档本体），却被拆在两侧；
- 右栏内部，Simulation（执行配置）与 Display（显示分级）挤在同一条滚动列里，属两种生命周期；
- 信息密度最高的编辑（晶体/轴/滤镜）住在最差的容器里——阻塞式 modal，编辑时看不到预览。

而正确的分割轴其实项目里早已形式化：`doc/gui-state-governance.md` 的 field→tier 分类器把每个字段按「改动是否触发 re-sim」分为 sim 通道与 display 通道。**面板组织应当追随状态架构**，这不是品味，是机制层事实。

## 1. 定案形态：「文档 | 图像 | 运行」三区

按字段生命周期聚类，控件只有三类，各归各区：

```
┌─────────────────────────────────────────────────────────────┐
│ 文件簇        [▶ Run] [dirty芯片] Rays ∞ Max-hits GPU  窗口簇 │ ← 运行（顶栏执行簇）
├──────────┬──────────────────────────────────────────────────┤
│ SCENE    │                                                  │
│  树      │                                                  │
│ （master）│                视口（图像）                        │
│ ├─拖拽───┤                                                  │
│ 检视器    │                                                  │
│ （detail）├──────────────────────────────────────────────────┤
│          │ Grade | Overlays | Components(预留)               │ ← 显示条（挂在图像上）
├──────────┴──────────────────────────────────────────────────┤
│ ● Ready   1024×512 · Linear · FOV 90        N rays          │
└─────────────────────────────────────────────────────────────┘
```

- **文档**（会被保存的东西）：太阳、相机、层与晶体，在左侧一列以树列全。
- **执行**（这一次跑多狠）：rays / max hits / 后端选择与 Run 按钮同住顶栏执行簇。
- **显示**（对已有结果怎么看）：EV / 分辨率 / 宽高比 / 背景 / overlay，以标签条挂在视口下缘——它们是图像的属性，不是「右栏杂项」。

## 2. 文档列：双面板 master–detail，编辑模态退役

- 树（master）在上、检视器（detail）在下，**同柱堆叠**：选中与编辑相距一次短竖移。选中晶体，检视器显示其全部属性（形状 / 朝向 / 滤镜 / 权重，线框缩略图作身份锚）；选中太阳/相机/层同理；crystal / axis / filter 三个编辑 modal 由此退役——非阻塞、就近这两个 modal 的优点都保留，另获两样 modal 给不了的：**持久**（不遮预览、不会点掉，可以一边拖参数一边盯图）与**零窗口管理成本**。
- 两半是**两个真面板**（VS Code 侧栏形态，owner 压测后定案）：各自内容超高各自滚动，互不挤压；分隔条可拖（拖过即以手动为准，双击回到内容自适应，树的自适应上限约 45%）；**点击节头整节折叠**，把全高临时让给另一半——这是「深编辑偶尔要全高」的一键逃生口。
- ImGui docking 里「同列上下两个 docked 窗口」原生就是这一整组行为（独立滚动 + 共享分隔条 + 可折叠），落地零额外成本。docking 下检视器天然可撕出浮动，供偏好浮窗手感者使用，但**默认形态是同柱**。

## 3. 执行簇与 dirty 芯片

- Run 按钮 + rays + max hits + 后端开关 + 进度条构成顶栏执行簇。rays 滑条拖满进入 ∞ 档位（"until stopped"）——∞ 改变的是终止语义而非「一个很大的数」，故为独立档位且精确最大有限值仍可键入（这条约束沿自 `doc/gui-visual-language.md` §4.5，只是落点从面板行改为顶栏）。
- **dirty 芯片**：文档（sim-tier）字段在有结果之后被改动 → 执行簇亮琥珀色 "Changed · re-run" 芯片，点击即重跑；display-tier 字段永不触发它。判据直接取自 tier 分类器，不另起炉灶。
- 原型里浮现的一个免费收益值得保留：文档改动后**坐标系 overlay 跟随新文档值，而陈旧结果图不动**——两者的错位本身就把「结果已过期」画在屏幕上，与 dirty 芯片互为印证。

## 4. 视口与显示条

- 显示条以标签页组织：**Grade**（EV / Resolution / Aspect / Background）、**Overlays**（辅助线表格——`doc/gui-visual-language.md` §4.4 的表格形态在此落地，表头声明一次记录结构、空格子即信息、行末折叠承载不等宽字段）、**Components**（光路成分分析的预留位，让布局按未来住户验收，参见 `doc/gui-custom-spectrum-and-raypath-color.md`）。
- Resolution 属 sim-tier（改动触发重跑），其警示形态用语义色细左缘条，不用整条填充——语义色定义归 `doc/gui-visual-language.md` 的语义色三档。
- **空态天空坐标系**：无结果时视口不是死黑——按当前文档画淡色地平线、角距参考圆与太阳标记（约半强度），配一行指令性提示。空屏由此从「什么都没有」变成「已取景、待曝光」，同时是功能预告（未跑先知道 22° 晕在哪）。
- 长期方向（不在本轮范围）：光路成分分析落地、图像上的成分可点选后，**画布成为第二个 master**（点图上的幻日 → 检视器跳到贡献它的晶体/滤镜）。同柱布局与此兼容。

## 5. 被推翻的形态（勿重提）

- **检视器放屏幕右缘**（树左、检视器右的 Figma 式对称）——owner 实测推翻：构建场景时每次「选中→调参」都要横跨整个视口。Figma 式布局成立的前提（画布本身是选择面，鼠标不必去左栏）在本应用不存在。教训：组织原则是「按生命周期聚类」，树与检视器同属文档，本就该同侧。
- **树与检视器单列共享高度**（一列内自然分配、只靠分隔条缓解）——owner 压测推翻：多层多晶体 + 最深编辑同时发生时竖直预算不够。定案见 §2 的双面板形态。
- **树内联展开编辑**（选中项在树内膨胀成编辑器）——选中项膨胀会把兄弟行挤跳，列宽内塞不下深编辑器。
- **锚定 popover 编辑**（在树条目旁弹非阻塞小窗）——瞬态的，切换选中即丢编辑现场；作为默认形态被否，docking 撕出浮动已覆盖该需求。
- **workspace 模式页**（Blender 式 Compose/Iterate 切页）——模式是用户必须管理的额外隐藏状态；同柱 master–detail + 可折叠面板已覆盖双峰需求。若 Components 真的撑破单一布局再以证据重提。
- **节点图**表达层×晶体×滤镜——结构是固定 schema（深度 2–3），不是任意 DAG；为浅结构付深结构的复杂度税。

## 6. 落地自由度（有意不定死的细节）

以下由实施时按 ImGui 实际行为裁量，蓝图不约束：列宽具体值与自适应上限、热参数是否需要 pin/收藏机制（若迭代实测「先选中太阳再调高度」构成真实摩擦再议）、显示条挂下缘还是右缘（届时按 Components 图例的宽高需求定）、窄窗口下执行簇的降级排布、树行的图标/缩略图具体形式、分隔条与折叠的具体手感。原型（HTML）与实现（C++/ImGui）的差异处一律以实现侧为准，但**§1–§5 的形态与被推翻清单不属于自由度**。

## 7. 与 docking 迁移的关系及顺序

- **本文就是 docking 迁移的设计输入**：迁移不做「现状面板的机械搬运然后再重组」——那是同一个 shell 改两遍。基底迁移（面板进 dock 节点、测试保绿）之后，各区按本文形态重建。
- **基底已落地（as-built）**，三条边界值得后来者直接接手，不必重新查证：
  - **顶栏与状态栏不是 dock 节点**，是夹住 DockSpace 的固定几何 chrome（`SetNextPanelGeometry`）。理由是 §1 里两者的定位——执行簇与状态展示——本就是固定 chrome，不是可被用户拖散的「文档」；真做成可拖拽节点，用户能把 Run 按钮拖到画面中间甚至拖没。
  - **中心节点永久保持为空**（`PassthruCentralNode | NoDockingOverCentralNode`），视口窗口钉在它的矩形上而不是 dock 进去。这不是风格选择：ImGui 只在中心节点为空时才在 dockspace 背景上开洞，一旦有窗口 dock 进中心节点，`ImGuiCol_WindowBg` 会填满整个 dockspace 把 GL 预览盖住。§1 的「图像区 = 视口」因此在机制上也成立——视口不是一个可以被拖走或被别的面板顶掉的工具窗。
  - **`DockBuilder*` 只允许出现在 `src/gui/dock_layout.cpp`**：docking 配置与默认布局若在 app 与 gui_test 各写一遍，参考图截出来的布局与真 app 就只是碰巧一致。与视觉语言收敛到 `theme.cpp` 同一条纪律。
- 纯视觉语言（字体/调色板/节奏/语义色）与本文正交，按 `doc/gui-visual-language.md` §6 先行落地。
### 7.1 基底 as-built（迁移第一步已落地，后续各区在此之上重建）

docking 基底已经就位，形态**刻意未变**（面板组织、编辑 modal 全部原样），以便把「docking 引入的回归」
与「形态重组引入的回归」分开归因。接手后续各区前需要知道的三件事：

- **单一 owner**：`src/gui/dock_layout.{hpp,cpp}` 是 docking 配置标志、DockSpace host、默认布局与
  **每一次对 dock 节点几何的写入**的唯一持有者。`src/gui/main.cpp` 与 `test/gui/test_gui_main.cpp`
  只调它暴露的那组函数、不自行触碰 `DockBuilder*`——理由与 `theme.cpp` 持有视觉语言完全相同：两套
  各自维护的 docking 初始化会让 gui_test 的截图变成「关于测试夹具的证据」而不是关于真 app 的证据。
  这条可机械核验：`grep -rn "DockBuilder\|DockSpace(" src/` 只应命中 `dock_layout.{hpp,cpp}`。
- **⭐ 中心节点永久为空，预览不是 dock 节点**。这条最容易被下一个人"顺手修正"回去，所以写明机制：
  `ImGuiDockNodeFlags_PassthruCentralNode` 只在中心节点**为空**时保持透明；一旦有窗口真的 dock 进
  中心节点，该窗口的 `ImGuiCol_WindowBg` 会整块盖住底下的 OpenGL 画面，「预览 docked」与「预览可见」
  不能同时成立。因此中心节点带 `NoDockingOverCentralNode` 永久留空，`##PreviewPanel` 是带
  `ImGuiWindowFlags_NoDocking` 的普通窗口，几何取自 `dock_layout` 的 `GetCentralNodeRect()`
  ——中心节点没有窗口入驻，没有任何窗口能被问出它的矩形，这正是该接口存在的原因。
  §2 所说「检视器天然可撕出浮动」不受影响：那是侧节点的原生能力，与中心节点留空无关。
- **布局持久化**：交互式 app 把用户拖出来的布局持久化到用户配置目录（**不是** ImGui 默认的
  `imgui.ini`，那会落在进程 cwd 随启动目录漂移），配 View → Reset Layout 复位；`gui_test` 侧
  `io.IniFilename` 恒为 `nullptr`。这条分叉是刻意的，与 gui_test 默认禁用个人默认值同一条隔离纪律：
  参考图绝不能依赖跑测机器上碰巧存在的布局文件。**新增视觉参考场景时不要"顺手打开"测试侧的持久化。**

- 测试影响：`modal_layout` 参考组随编辑模态退役而废弃（其覆盖的控件布局命题由检视器侧的新参考组接手）；`defaults_panel_layout` 的 Settings 面板不在本文范围、暂不受影响；`lens_proj` 走离屏 FBO，与 shell 无耦合；`capture_harness` 整帧参考随每步 shell 改动失效，重拍税一次性付清于 shell 收尾（重拍纪律见 `doc/testing-architecture.md` §4.6 与 `doc/gui-visual-language.md` §8）。

### 7.2 执行簇 as-built（顶栏，§3 已落地）

§3 的执行簇已建成，三处细节属 §6 的落地自由度、由实现侧定案，写在这里免得下一个人重新推一遍：

- **顶栏是两行，不是一行**。§1 的 ASCII 图把文件簇、执行簇、窗口簇画在一行，那是形态示意；一行装不下
  是实测而非估计——chrome（面板折叠 / New / Open / Save / Colors / Colored / Settings / View）本身约
  700 px，执行簇再加约 800 px，连 1600 px 的默认窗口都溢出，更不用说 `kMinWindowWidth`。定案：
  **第一行 chrome，第二行执行簇**（Run/Stop · dirty 芯片 + Revert · Rays · Max hits · Use GPU · 进度），
  按生命周期分行，与三区划分同一条组织原则。`kTopBarHeight` 因此从 40 变 64
  （= `WindowPadding.y*2 + FrameHeight*2 + ItemSpacing.y` = 57 取整留余）。执行簇整行约 985 px，
  在 1024 px 下**仍完整装下**（实测，非估算）；再窄则裁掉行尾，不重排——Run、dirty 芯片、光线预算这三个
  最常用的留在最左。
- **⭐ Rays ∞ 档位：档位边界闭在 ∞ 一侧，所以最大有限值靠拖拽结构上不可达**。轨道 `[0, 0.88)` 映射有限域
  `[min, max)`，`[0.88, 1]` 是显示 "until stopped" 的档位。这不是「像素精度不够所以很难拖到」，而是映射本身
  就取不到 `max`——**这正是「输入框必须始终可编辑」的对偶**，两条一起才让 ∞ 保持为终止语义而不是
  「一个很大的数」（约束出处 `doc/gui-visual-language.md` §4.5）。任何让轨道顶端吸附到 `max` 的「顺手改进」
  都会同时废掉这两条。
  另一条同样容易被改掉的性质：**拖进档位时要还原拖拽前的有限值**。ImGui 滑条每帧从指针绝对位置反算值，
  所以「从 5 M 拖到最右」这条路径本身会把 5 一路改写成接近 100；不还原，用户就带着一个自己没输过的数字
  进入 ∞。这条性质原本属于被退役的 `Infinite rays` 复选框，它不因为复选框没了而失效。
- **dirty 芯片没有自己的字段清单**。判据是 `IsModified(sim_state)` ← `dirty` ←
  `DiffAgainstCommitBaseline`，全链单一。值得知道的边界：`gui_state_tiers.hpp` 的 tier 表以**整个 struct**
  为一行，而提交基线只捕获 `renderer` 的 re-sim 投影（`RenderConfigResimFields`：分辨率 / 背景 / 光线色 /
  不透明度）——所以改 `renderer.fov` **不会**点亮芯片，改 `renderer.sim_resolution_index` 会。芯片跟随的是
  更细、也更权威的那个投影；tier 表在 struct 粒度上是治理文档，不是运行判据。
- **芯片与 Revert 是两个动作，不合并**：芯片带着新配置重跑，Revert 把新配置扔掉；合并等于删掉后者。
  芯片在运行中天然不可见——`ReconcileSimState` 只从 `kDone` 产生 `kModified`，`kSimulating` / `kStopping`
  不被 dirty 降级，运行中的编辑走 `main.cpp` 的节流自动提交。
- **进度槽**：有限预算画 `ImGui::ProgressBar(已追迹 / 预算)`；∞ 档位**整个槽连同前面的分隔条一起不画**。
  两条理由，第二条更容易被写错：没有分母的条只能说谎（满条读作「已完成」、空条读作「卡住」），而且光线预算
  控件已经在同一行写着 "until stopped"，进度槽再写一遍不增加任何信息、只会被读成渲染故障。它是行尾最后一项，
  所以省略不会让任何东西移位。状态指示文字（Ready / Simulating… / Done / Modified）**留在状态栏**，
  与 §1 的 ASCII 图一致。
- 测试影响：执行簇的用例住 `test/gui/functional/test_execution_cluster.cpp`（按字段用途切分：这一次跑多狠
  归执行簇，会被保存的文档字段留 `test_scene_controls.cpp`）。`kTopBarHeight` 变化会改变 `visual` 组
  `left_panel` 参考图的捕获高度（`test_gui_main.cpp` 的回读按 `fb_h - (kTopBarHeight + kStatusBarHeight)*sy`
  算），该参考图与 `capture_harness` 整帧一并进入 shell 收尾的一次性重拍。
