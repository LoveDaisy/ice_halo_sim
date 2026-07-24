# AGENTS.md

## Project Overview

Lumice is a C++17 ice halo ray-tracing simulator. It reproduces halo patterns by tracing light through ice crystals and supports CLI, GUI, unit tests, E2E tests, and performance-oriented workflows.

Core conventions:

- namespace: `lumice`
- public API boundary: `src/include/lumice.h`
- source layout: `.hpp` headers and `.cpp` implementations
- build system: CMake + Ninja
- dependency management: CPM.cmake

## Common Commands

```bash
# Build
./scripts/build.sh -j release
./scripts/build.sh -tj release
./scripts/build.sh -gtj release
./scripts/build.sh -k release

# Run
./build/cmake_install/Lumice -f examples/config_example.json
./build/cmake_install/Lumice -f config.json -v

# Tests
./scripts/build.sh -tj release
./scripts/build.sh -gtj release
LUMICE_SKIP_GUI_TESTS=1 ./scripts/build.sh -gtj release
pytest -v                                 # fast e2e only (matches CI; testpaths in pyproject.toml)
./scripts/build.sh -sj release && \
  pytest -v -m slow                       # slow e2e (needs shared lib; run before PR)

# Format
./scripts/format.sh
./scripts/format.sh --check          # read-only: exit 1 if anything needs formatting (used by the pre-commit hook; CI runs the format-check job)

# Engineering-policy gate (env-knob centralization/registration, GUI API boundary, using-namespace)
python3 scripts/check_policies.py    # whole-tree: what state is the repo in?
python3 scripts/check_new_refs.py --staged        # diff-scoped: what did you just write?
python3 scripts/check_new_refs.py --range A..B    # same, over a commit range (what CI runs)
./scripts/install-hooks.sh           # install the non-interactive pre-commit hook (one-time)
```

Release artifacts land in `build/cmake_install/`. Debug builds stay in `build/cmake_build/`.

## Code Structure

- `src/config/`: configuration parsing and simulation config data
- `src/core/`: math, optics, simulator, filters, buffers, ray paths
- `src/gui/`: GUI app, panels, preview, file IO, poller
- `src/server/`: server-side render, consumer, stats, C API bridge
- `src/util/`: logger, threading, queue, arguments, color data
- `src/include/`: public C API header
- `test/`: unit tests, GUI tests, and E2E tests

## Style and Engineering Rules

- Follow the repository `.clang-format` and `.clang-tidy` rules.
- Naming:
  - types: `CamelCase`
  - functions: `CamelCase`
  - variables: `lower_case`
  - private members: `lower_case_`
  - constants: `kCamelCase`
- Prefer `const`, `constexpr`, smart pointers, `nullptr`, and `override`.
- Do not use raw `new` / `delete`.
- Do not use `using namespace`.
- Keep code comments in English.
- Public API boundary: `src/gui/` code must only access core/config functionality
  through the C API (`src/include/lumice.h`). Direct `#include` of `core/` or `config/`
  headers from `src/gui/` is prohibited.
- Environment variables: before adding any new `std::getenv`, apply the decision gate in
  `doc/env-var-policy.md`. User-facing behavior switches must go through CLI/config/API,
  not env vars (env causes silent per-machine drift). Env is fine for dev/experiment knobs
  (centralized + logged on startup) and test/build infra. All `LUMICE_*` env reads live in
  `src/util/env_knobs.cpp` and are enforced by `scripts/check_policies.py` (CI `policy` job +
  local pre-commit hook, installed via `./scripts/install-hooks.sh`).
- Working-note references: prose in a tracked file must not cite `scratchpad/` artifacts —
  task ids (`task-<name>`, `explore-<name>`, `scrum-<name>`, `chore-<name>`), per-task note
  filenames (`plan.md`, `progress.md`, `SUMMARY.md`), `code-review-0N`, `round-N`, or
  `scratchpad/` paths. That tree is git-ignored and archived per task, so the reference
  dangles for anyone reading later; state the mechanism itself instead. Permanent anchors
  are fine and should stay: PR numbers, commit hashes, `file:line`, code symbol names.
  This applies to any long-lived prose read apart from the context it was written in —
  production comments, test docstrings, CI config comments, and `doc/` alike.
  Enforced by `scripts/check_new_refs.py`, a **second, diff-scoped entry point** next to
  `check_policies.py` — not a check inside it. It reads only the lines a change *adds*
  (CI `new-refs` job on PRs vs merge-base; pre-commit hook on the staged diff), so the
  large body of pre-existing references never blocks a commit. Only natural-language
  regions are read: comments, Python docstrings, and Markdown — never string literals.
  There is no inline exemption, by design: if some prose genuinely needs such a reference,
  the rule is wrong and the rule should change. Files exempt from it are limited by one
  criterion — the file's *subject* is this working-notes system itself, i.e. it must name
  the system to do its job (this file, `CLAUDE.md`, and the root `.gitignore`). Citing the
  system is not grounds for exemption; that is the thing being checked.
  **The checker is the rule, not an approximation of it.** If `check_new_refs.py` passes, the
  change is compliant — full stop. Do not flag a reference in review that the checker accepts,
  and do not appeal to the rule's "spirit" beyond what it matches: the checker deliberately
  misses some citation forms (the trade-offs are argued at each pattern in the script), and
  those misses are accepted, not oversights awaiting discovery. Rationale: this rule guards
  against a harm measured to be *low* — 41% of tracked files carried such references for years
  with no observed cost — so an unbounded review-time judgement about it is worth less than it
  costs. Tightening is legitimate only by changing a pattern in the script (and accepting the
  false positives that buys), never by case-by-case escalation in review.

## Testing and Platform Notes

- CLI, core, and unit-test flows should remain cross-platform.
- GUI tests require a display server unless explicitly skipped with `LUMICE_SKIP_GUI_TESTS=1`.
- E2E test layout (purpose-primary; see `doc/testing-architecture.md` §6):
  - `test/e2e-correctness/` — full-stack correctness via CLI/PSNR (smoke, CLI behavior, raypath equivalence) + `references/*.jpg`
  - `test/parity-cross-backend/backend/` — backend-equivalence oracles (Metal exit-seam parity, device-gen default path, cpu_backend route, Metal batch invariance) + C++ siblings from 270.3
  - `test/performance/` — throughput gates (Metal throughput)
  - `test/gui/` — GUI acceptance (Metal GUI north-star) alongside the C++ GUI tests (`functional/`, `visual/`, `responsiveness/` subdirs; target `gui_test`)
  - `test/regression-sentinel/` — bug-resurfacing guards (errors, capi sentinel overflow, MS filter leak)
  - Shared fixtures stay under `test/e2e/` (`base.py`, `runner.py`, `capi_runner.py`, `image_utils.py`, `_parity_metrics.py`, `configs/`).
- E2E test split:
  - Default `pytest -v` runs the fast subset — matches CI behavior (CI uses `-m "not slow"`). Test paths come from `pyproject.toml` `testpaths`.
  - `@pytest.mark.slow` tests require the shared-lib build (`./scripts/build.sh -sj release`) and are excluded from CI to keep PR feedback fast. Run them locally with `pytest -v -m slow` before opening a PR that touches the simulator core, query filter, or C API surface.
    - `test/regression-sentinel/test_capi_sentinel_overflow.py` — sentinel-overflow regression: 3-config × 12 rounds = 36 server lifecycles via `LUMICE_GetRawXyzResults(max_count=1)`; guards against reintroduction of the c_api.cpp off-by-one sentinel write (fix: 5287efe)
    - `test/regression-sentinel/test_ms_filter_leak.py` — Design A filter-fail termination regression: confirms filter-fail rays do not propagate across MS layers
- GUI screenshot references live under `test/gui/references/`.
- Windows physical-desktop validation uses `scripts/win_remote_test.sh` together with `scripts/win_test_watcher.ps1`.
- Performance diagnostics and workflows are documented in `doc/performance-testing.md`.

### GUI Test Reference Regeneration (auto_ev tests)

The `auto_ev` reference images are pixel-averaged means of N=10 stochastic renders to suppress
per-run noise. Per-scene PSNR thresholds are `mean − 3σ` (floored to 0.5 dB precision).

Each scene has a single reference: the auto-EV-applied capture (`auto_ev_<scene>_on.jpg`). The
legacy `_off` (intensity_factor=1.0) mode was dropped in chore-auto-ev-regression-drop-off — the
GUI has no auto-EV toggle, so off was a degenerate non-default state exercising no unique code path.

**`--keep-export-png` flag** — When passed to `gui_test`, `CheckAgainstReference` skips
`std::remove` so the per-run export PNGs at `/tmp/lumice_auto_ev_*.png` are preserved for
collection by the driver script.

**Regeneration workflow:**
```bash
# Full regen (Phase A: generate mean-ref + Phase B: calibrate thresholds, ~20 min):
python scripts/regen_gui_test_refs.py

# Phase A only (generate mean-ref images, then manually update thresholds):
python scripts/regen_gui_test_refs.py --phase-a-only

# Phase B only (recalibrate thresholds against existing mean-refs):
python scripts/regen_gui_test_refs.py --phase-b-only

# Quick smoke test (2 runs each phase):
python scripts/regen_gui_test_refs.py --n 2 --n-calib 2
```

After Phase B, copy the `threshold` values from `test/gui/references/_thresholds.json` into
`kScenes[]` in `test/gui/visual/test_gui_auto_ev.cpp` (one `<scene>_on` threshold per scene).

## Logging and Troubleshooting

- `VERBOSE` is a project-defined log level between `DEBUG` and `INFO`.
- When debugging intermittent issues, first map the full data flow, then audit silent-return paths and add observability before changing behavior.
- If a known-good branch or prior fix exists, diff it first before starting fresh analysis.
- Do not split one decision across multiple threads or modules when a single owner can make it.

## Collaboration Constraints

- `config.json`, `test.json`, `scratchpad/`, remote test output files, and most generated artifacts are intentionally git-ignored.
- Do not use `git add -f` to force-track ignored files. If a file is ignored and you are unsure, stop and ask first.
- Reference images under `test/e2e-correctness/references/*.jpg` and `test/gui/references/*.jpg` are explicitly unignored and may be tracked normally.
- CI runs build and unit tests on branch pushes; E2E tests run on PRs and `main`.

## Documentation Index (`doc/`)

Valuable design/architecture docs live in `doc/` (tracked). Consult the relevant one
**before** redesigning a subsystem — many decisions are already reasoned out here.
(Most have a `_zh` Chinese sibling.)

- **Overview / guides**: `README.md`, `architecture.md`, `developer-guide.md`, `gui-guide.md`, `configuration.md`
- **Core / rendering architecture**: `accumulator-consumer-architecture.md`, `raypath-rayseg-architecture.md`, `raypath-symmetry.md`, `coordinate-convention.md`, `crystal-orientation-sampling.md`, `ev-pipeline-architecture.md`, `adaptive-brightness.md`, `filter-architecture.md`
  - `overlay-label-placement.md` — GUI overlay 文字 label 的 **curve-centric 放置设计**（blueprint；explore-288.5 收敛）：现 boundary-centric 5-source 的 4 缺口审计（globe/rectangular 缺经度、dual_fisheye 零 label、边缘成簇）+ curve-centric 统一模型（每曲线 walk→裁剪可见区域→边界/内部两模式）。改 overlay label 放置前先读。
  - `numerical-robustness.md` — geometric numerical-stability conventions (7 rules): avoid absolute-ε anti-pattern, prefer argmax / relative tolerances, double precision for geometry generation, single predicate owner; distilled from the extreme-wedge bug family (PR #132/#133/#135/#137). Read before adding or **modifying** any geometric predicate.
  - `crystal-geometry-representation.md` — **晶体几何生成 + 内存表达的机制层诊断与重构**（**重构已大半 as-built，PR #214**；§1 是 pre-#214 管线=诊断，§4.a/b/c 是落地状态）：缺陷族根因 = 管线把先验已知信息逐步丢弃再用数值方法重建（平面→顶点→面归组→三角化→反推面归属→反推面号），每个重建一个容差、每个容差一个真缺陷（PR #132–#209 对照表）；结构性错配 = 把参数化的 ≤20 面封闭小族当任意凸多面体处理（通用求解器仅 3 个调用点，全是六方晶体工厂）+ 三角网格/多边形面主次倒置。目标表达 = 平面 + 面存在掩码 + 面号常数表 + 闭式轮廓，扁平 POD 三后端同构。⭐**拓扑复用的失效判据难题在该表达下不存在**（判据问题只在拓扑靠数值发现时才有）。含诚实边界（凹锥/求交 ε/与「校验输出不校验输入」判据的张力）+ 动手前必答三问（消费者清点 / **等价性 gt 不能用旧求解器** / 面存在谓词形态）。**重设计几何前先读这篇；只是改谓词读 `numerical-robustness.md`**。
  - `near-pole-area-measure-sampling.md` — **近极朝向采样从"常数包络拒绝"换为"面积测度重要性采样"的设计蓝图**（explore-326 GO + scrum-328.1 收敛）：`θ=90−lat` 换元 + 严格上界 `sinθ≤θ` → 提案 `base_pdf(θ)·θ`（Gaussian=Rayleigh / Laplacian=Gamma(2,b) / uniform=常数）+ 接受 `sinθ/θ`（M=1）= **精确 + 99% 接受**（vs 当前 20-27%）。⭐AC 硬约束：parity 不可 match 旧采样器（proposed 有意更精确，修当前 Laplacian 尾部 clamp 偏差）→ 对解析目标验证。改近极采样 / Rayleigh 路 / `ComputeJacobianEnvelope` / GPU device gen sampler 前先读。
  - `gui-preview-lifecycle-architecture.md` — **后台 worker × 前台实时显示的时钟解耦设计**（blueprint；GUI 侧对偶于 `seam-design.md`）：诊断"跑完但 GUI 仍 Simulating"卡死的机制层根因 = 仿真生命周期被复制 + 边沿触发 + 撕裂读；四时钟解耦（显示/快照物化/batch 生产/生命周期心跳）+ epoch 统一世代键 + 电平触发 reconcile + CQS + 6 条不变量 I1–I6。改 GUI 预览生命周期 / poller-server 交接 / 完成判定前先读。
  - `gui-state-governance.md` — **GUI 状态治理设计蓝图**（explore-gui-state-governance 收敛，2026-07-11）：回答 owner 总纲"每个用户操作 → ①内部状态如何转换 ②所有相关显示如何更新"。诊断 = `GuiState` 数据集中但**状态转换散乱、无统一 owner**，且偏离几乎全部聚于 **display 通道 ↔ sim 通道的交界**（活 bug=display-time 操作借 EnsureRunning+PublishValidReset 污染 sim_state 闪 Simulating / commit↔display 字段割裂 / Revert 不重推 / 显示态与结构态挤同 struct）。目标模型三支柱 = field→tier 声明式分类器（拆 ColorClassConfig 结构态/显示态子结构）+ 每通道单一序列化器+重推纪律 + latch 派生态且 display-time 禁碰 re-sim 原语，外加单一 `ResetFrontendState(reason)` 文档重置 owner（= backlog #5）。含 5 条固化不变量 + T1–T6 scrum 拆解。改 GUI 状态转换 / 染色 display 通道 / 文档切换重置 / 仿真生命周期显示联动前先读。
  - `gui-custom-spectrum-and-raypath-color.md` — GUI 功能扩展设计（功能 1 自定义离散光谱已由 task-323 落地并将 `ray_num` 语义统一为总数）。**功能 2 per-raypath 颜色标记（2026-07-05 深化蓝图，未立项）**：一个物理机制三层角色 = 物理门 filter（1:1 不动）+ 色桶 = filter 的 summand（Fork C，不放宽绑定）+ 跨层 rule（带层键 component 的布尔组合，接跨层轨迹染色）。⭐地基窄而稳 = 产 per-ray component 掩码（复用 §5.1）+ 跨层前向累积（加宽 `is_prior_filter_failed_`）+ 交付 consumer；三硬承诺 = 携带原始掩码/带层键/uint64。⭐隔离契约 = binning + re-sim 边界 + rule + UI 全属地基之上、换之不动 seam。改光谱/光路染色相关前先读。**§4.8 合成算法重设计（2026-07-15 定案）**：dominant 噪声敏感根因 = argmax 不连续；定案 = dominant 保持 hard argmax（诊断视图）/ painter 改为亮度即 alpha 的 over 合成（`alpha=f(ey)=min(ey,1)`、纯色相、修「暗点压亮点变黑」黑洞 + 顺带连续）/ painter 设默认 / EV 解耦（alpha 用 self-anchor、display EV 后置乘）/ composite+EV 在线性 RGB。改 `component_compositor.cpp` 前先读 §4.8。
- **C API**: `c_api.md`, `capi-lifecycle-architecture.md`
- **GPU / Metal route** (read these before touching the GPU path):
  - **🔒 设计纪律（GPU 后端实现硬约束）**：按 `seam-design.md` 蓝图走，**不要自己重新发明**。几何遍历 / 出射 seam / per-ray 旋转上传 / 单引擎大 dispatch — **复用已验证的实现**：参考当前 Metal（`gpu-single-engine-implementation.md` as-built）+ legacy `PropagateSlab`（`optics.cpp` 的 polygon-slab 遍历）。**蓝图是最终判据**：Metal/legacy 与蓝图冲突处以蓝图为准（如历史 Metal 投影焊进 trace 已被 §4.1/scrum-258 纠正，别照搬旧形态）。教训：CUDA #295 自创 Möller-Trumbore 遍历复现了 task-275~278 已解决的绝对-ε 漏面 bug（energy 0.735）；详见 `scratchpad/backlog.md`「MVP 落地后的架构发现」。
  - `seam-design.md` — **the `TraceBackend` host/device seam redesign blueprint**; §5 = single-engine, three-clock-decoupled GPU simulator (the target architecture); §3.6 "原始之罪" = why GPU must not mirror the CPU pipeline.
  - `gpu-route-history.md` — systematic retrospective of the GPU migration (#250→268): decision evolution, accumulated data assets, leftover-item ledger; §9 = single-engine arc high-point (CLI 9.5×/GUI 2.07×).
  - `trace-backend-frame-lifecycle.md` — Metal frame lifecycle (as-built, multi-MS transit via device `transit_root_kernel`, parity harness methodology, §8 DR-3 per-ray wavelength).
  - `gpu-single-engine-implementation.md` — **§0 as-built 接手须知**（scrum-267+268 完成：CLI 9.5×/GUI 2.07×/dispatch 32768/concern #2 解/DR-3 波长/R1 occupancy 640 benign）+ §5 设计推理 + §8 DR-3 决策链 + §9 关键发现；接手 GPU 路线先读 §0。
- **Perf / testing**: `performance-testing.md`, `windows-remote-testing.md`, `xyz-stats-tool.md`
  - `geometry-randomization-value-and-measurement.md` — **几何随机化"值不值钱"的两轴框架 + 偏置轴度量方法论 + stage-1 发现**（新，explore 收敛 2026-07-23）：⭐**混轴是度量原罪**——方差/效率轴（K=D vs 64，收敛多快）vs 偏置/正确性轴（ON vs OFF，确定性收敛到物理错的过锐图）；**已有 moat 数字（28–40×/15×）是方差轴的**，卖点在没测过的偏置轴。偏置发现：主峰软化 −21% / 暗区填充 +50% / 对日弱峰抑制减半，**gauss≡uniform 同 σ ⇒ 干净多分散非退化背景**，效应集中 faint/away-from-peak（两轴同指向）；plate 幻日环更丰富 + 120° 反常变亮（未解）。含**变量定律 blueprint**（`Var(K)=(σ̄²_w+K·σ²_b)/N`，`K*=√((σ̄²_w/σ²_b)·(C/c))` + 三结构不变量 + 护栏红线）+ 度量方法论（散射角空间 / 天顶-rectangular / plate 幻日环 / uniform disentangle / 积分能量纪律 / C API no-filter 读 xyz_buffer）。设计几何随机化质量/吞吐 gate 前先读。
  - `geometry-randomization-perf.md` — **几何随机化性能的成本模型 + 前沿图**（**顶部有 post-闭式更正段**）：真正指标 = equal-error throughput（变量轴，非 raw）；⚠️「随机比确定性慢 15.7×」是冷 vs 暖 CUDA context-init 假象；⭐**闭式落地（PR #214）后 C（拓扑复用）是错杠杆**（只碰 92ns compute=构造 5%，「B+C 不可分」作废），真杠杆 = 砍 MakeCrystal 对象构造（多晶体 ROI 最大）；⭐**K=64 惩罚场景相关 0.25–0.43、多晶体更差**（非普适 0.37×），K=D 跨场景免费。含「压到极限没有」核对清单 + 测量纪律。优化几何随机化路径前先读，配 `geometry-randomization-value-and-measurement.md` 一起。
  - `gpu-remote-cuda-build-testing.md` — **dev49 + win-builder 现成 recipe**（CUDA build + parity/正确性
    验证的两机操作手册：源码同步、docker/BuildTools 工具链、`LUMICE_HAS_CUDA` un-skip 闸、parity battery
    三文件、PS-over-ssh 坑）。**任何触及 `cuda_trace_backend.*` / 三后端共享头 / SimData / simulator 的
    改动，先读这份**，别对两台机器从头摸索。与 `windows-remote-testing.md`（GUI VSync 物理桌面）场景正交。
  - `testing-architecture.md` — **authoritative test-organization spec**: verification-purpose primary axis × subsystem tag, seven layers (unit-correctness / golden-analytic / parity-cross-backend / e2e-correctness / performance / gui / regression-sentinel), the "how to add a test" decision tree, cross-cutting rules (perf denominator = legacy CPU; parity metric-masks-bugs battery; reference ownership), and the layer×subsystem physical-layout blueprint. Read before adding or reorganizing any test.
- **Engineering policy**: `env-var-policy.md` — **环境变量使用策略**: user-facing behavior switches must NOT live only in env vars (they cause silent per-machine drift / undebuggable bugs); use CLI/config/API instead. A-class runtime knobs (`LUMICE_TRACE_BACKEND` + 6 perf knobs, with file:line) vs B-class test/build infra (leave alone); three disposition rules; and the **decision gate to answer before adding any new `getenv`**. Read before introducing a new env knob.
- Example config: `examples/config_example.json`

## Knowledge Base & Working Discipline

This project carries a deliberate accumulated memory. Its value depends on it being
**retrieved**, not just stored (信息价值 = 内在价值 × 被检索到的概率). The recurring
failure mode is starting each session like a newcomer and re-deriving decisions the
owner already settled. Avoid it:

- **Retrieve before re-deriving.** Before starting work on a continuing/recurring topic
  (GPU/Metal perf, GUI perf, parity, batch/throughput, architecture decisions), FIRST:
  1. `grep` `scratchpad/backlog.md` for the topic — it holds owner concerns, start
     conditions, and dependencies (e.g. concern #2 commit↔batch decoupling).
  2. Read the relevant prior `scratchpad/explore-*/{SUMMARY,insights}.md` and
     `scratchpad/scrum-*/SUMMARY.md` — they hold verified conclusions and rejected
     directions. Check the `doc/` index above.
  3. **Explicitly continue prior threads** ("this refines concern #X") rather than
     starting fresh. If new evidence conflicts with a prior conclusion, connect them.
- **Don't re-measure what's already de-risked.** Before running ANY benchmark or
  experiment on a recurring topic, `grep` the relevant prior `experiments.md` for that
  exact measurement — if it's there, harvest the number, do NOT re-run it. Reading a
  SUMMARY is not enough: it gives conclusions, but the failure mode is re-running the
  experiment, so check `experiments.md` at row level. A *design* explore's job is to
  de-risk the unresolved frontier (the blueprint's open §-items), not to re-confirm the
  premise the blueprint already rests on. Self-check: if your "finding" restates a
  sentence already in some prior insights/experiments, you are re-deriving — stop.
- **The scratchpad system is the source of truth for in-flight reasoning.**
  `scratchpad/tasks.md` (the task ledger), `scratchpad/backlog.md` (deferred work +
  owner concerns), `scratchpad/explore-*/` & `scratchpad/scrum-*/` (per-effort
  hypotheses/experiments/insights/SUMMARY), `scratchpad/learnings/` (extracted lessons).
  These are git-ignored working memory — do not treat their absence from git as absence
  of knowledge.
- **Promote durable design docs out of `scratchpad/` into `doc/`.** scratchpad is
  git-ignored, so design docs left there are undiscoverable and get lost (this is how
  the `seam-design.md` blueprint sat unbuilt). When an explore/scrum produces a durable
  design or decision record, copy it to `doc/` and add it to the index above.
- **Think at the architecture level before decomposing into tasks.** The task/scrum
  machinery rewards fast decomposition and immediate action; resist acting on the first
  promising small direction before the architecture-level question is reasoned through.
