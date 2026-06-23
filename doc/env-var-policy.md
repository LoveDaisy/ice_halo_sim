# Environment Variable Policy（环境变量使用策略）

> 状态：策略文档（chore-286 立项）。本文确立 Lumice 中"环境变量作为调节旋钮"的
> 使用边界与处置规则，并定义新增 env 旋钮前的决策门。
>
> 相关公理：**a04**——软约束必失效，约束须固化为自动化门禁；信息价值 = 内在价值 ×
> 被检索到的概率。env 变量恰恰是最软、最不可检索、最逐机的配置形态。

## 1. 立场

环境变量本身不是问题。**问题是用 env 承载了"会改变用户机器上输出行为"的开关。**

这类开关的危害是结构性的：

- **逐机静默漂移**：用户 shell profile 里残留一个 `LUMICE_TRACE_BACKEND=metal`，之后每次
  运行都静默走另一条引擎路径——同一份配置、同一个二进制，不同机器不同结果。
- **不可检索**：env 不进配置文件、不进命令行历史、不进 bug 报告附件。复现一个"只在某台
  机器上出现"的 bug 时，env 是最后才会被想到的变量。
- **绕过 API 边界**：本项目刻意维护清晰的 GUI/CLI ↔ core 边界（C API）。env 旁路这道边界，
  让"行为由谁决定"变得不可追溯。

因此该治的是**这一类**（user-facing 行为开关），而不是所有 `getenv`。测试/构建基础设施用
env 是标准做法，不在治理范围内。

## 2. 现状清单

> 勘察方法：`grep -rn "getenv" src/`，剔除 C API 枚举常量（`LUMICE_OK` / `LUMICE_ERR_*` /
> `LUMICE_LOG_*` / `LUMICE_LENS_*` / `LUMICE_AXIS_DIST_*` / `LUMICE_MAX_CONFIG_*`）与头文件
> guard（`LUMICE_*_HPP`）噪声后，真·环境变量分两类。

### A 类 — 运行时行为旋钮（策略主要约束对象）

| 变量 | 位置 | 性质 | 已有替代路径？ |
|---|---|---|---|
| `LUMICE_TRACE_BACKEND` | `src/core/simulator.cpp:558`、`src/server/server.cpp:211` | 选后端 `legacy`/`cpu_backend`/`metal`——**最危险的 footgun** | GUI/API 有（见下）；**CLI 无 `--backend` flag，靠此 env** |
| `LUMICE_DISPATCH_RAY_NUM` | `src/server/server.cpp:890` | GPU dispatch 粒度（perf 旋钮） | 无（纯实验旋钮） |
| `LUMICE_COMMIT_RAY_NUM` | `src/server/server.cpp:685` | commit 粒度（perf 旋钮） | 无（纯实验旋钮） |
| `LUMICE_BATCH_RAY_NUM` | `src/server/server.cpp:691,702` | 已废弃别名（已有 deprecation WARN） | 由 `COMMIT_RAY_NUM` 取代 |
| `LUMICE_DISABLE_DEVICE_GEN` | `src/core/metal_trace_backend.mm:494` | 关 device-gen（实验/调试旋钮） | 无 |
| `LUMICE_DISABLE_METAL_SOURCE_COMPILE` | `src/core/metal_trace_backend.mm:353` | 关 metal 源码编译（也被回归 sentinel 当门禁用） | 无 |
| `LUMICE_WL_POOL_SIZE` | `src/core/metal_trace_backend.mm:192` | 波长池大小（实验旋钮） | 无 |

关于 `LUMICE_TRACE_BACKEND` 的关键事实（决定其处置方式）：

- 代码注释（`simulator.cpp:547`）已自述其为 **"debug/CI override; legacy semantics"**——
  设计意图上它本就是覆盖项，不是主路径。
- 主路径已存在：`Server::SetPreferredBackend`（默认 `kPreferCpu`），经 GUI checkbox 暴露，
  并由 C API 后端可用性 gate 把关（task-281 `LUMICE_IsBackendAvailable`）。所以 **GUI/API
  侧已有正规的后端选择路径**，env 只是其上的覆盖。
- 缺口在 **CLI**：CLI 没有一等公民的 `--backend` flag，目前后端选择依赖此 env
  （见 `server.cpp:207` 注释 "CLI / --benchmark"）。这正是需要补的显式路径。

### B 类 — 测试 / 构建基础设施（正当用途，不治理）

`LUMICE_SKIP_METAL_TESTS`、`LUMICE_SKIP_GUI_TESTS`、`LUMICE_PERF_CORELOG/METAL/CONFIG`、
`LUMICE_BIN`、`LUMICE_LIB`、`LUMICE_TEST_REF_DIR`、`LUMICE_E2E_CONFIG_DIR`、
`LUMICE_ICO_PATH`、`HOME`。

这些只在测试 harness（`test/`）和脚本（`scripts/`）里读，**不进产品运行路径**。env 恰恰是
它们正确的载体（CI/本地差异、临时路径注入是 env 的本职），硬搬成 CLI/API 反而是污染。**保持
现状。**

## 3. 三条处置规则

1. **用户可触发的输出行为**（A 类里 `TRACE_BACKEND` 是典型）→ **必须有 CLI / config / API
   路径**；env 要么删除，要么降级为"带响亮 WARN 的 debug-only override"（启动时若检测到该
   env 生效，打 WARN 级日志说明"正在被环境变量覆盖，非正规配置路径"）。

2. **纯开发 / 实验旋钮**（dispatch / commit / wl_pool / disable_*）→ **不必**搬成 CLI。最高
   杠杆、最便宜的一招是：**集中化 + 启动时把所有"非默认的 env 覆盖"打一行 log**，例如：

   ```
   [INFO] env override: LUMICE_DISPATCH_RAY_NUM=2048 (default 32768)
   ```

   这把不可见的逐机状态变成日志里可 `grep` 的一行——一个困惑的 bug 报告瞬间可解释。这正是
   a04 的做法：软约束失效，就把它固化成**可观测的门禁**，而非靠自律。即使一个变量都不迁移，
   仅此一条就消掉大半"难定位"的痛。

3. **测试 / 构建基础设施**（B 类）→ **不动。**

## 4. 决策门：新增一个 env 旋钮前必答

> 这是本文档的核心可执行产物。在引入任何新的 `std::getenv` 调用前，先回答：

**问：它会改变用户机器上的输出 / 行为吗？**

- **会** → 不要用 env 作为唯一入口。走 CLI flag / config 字段 / C API。若确实需要一个
  覆盖项用于 debug/CI，可保留 env，但必须：(a) 代码注释标明 "debug/CI override"；
  (b) 启动时生效则打 **WARN**；(c) 在本文档 A 类清单登记。
- **不会**（纯开发/实验/性能旋钮，不影响正确性结果）→ 可用 env，但必须：(a) 经过集中的
  env 读取入口（见规则 2 的注册表）；(b) 启动时若为非默认值则打 **INFO** 一行；(c) 在本文档
  A 类清单登记。
- **只在测试/脚本里读，不进产品路径** → B 类，自由使用，无需登记。

软约束会失效，所以这道门最终应固化为代码层的集中入口（见后续 task），而非仅靠本文档自律。

## 5. 需点名的后续 task（不在 chore-286 内实施）

本文档只立策略。以下为策略要求的代码落地，各自单独立 task + code-review：

- **task：env-knob 集中注册表 + 启动 log**——把 A 类 perf 旋钮（dispatch/commit/
  wl_pool/disable_*）的读取集中到一处入口，启动时对每个非默认值打一行 INFO。落地规则 2，
  最高杠杆。
- **task：`LUMICE_TRACE_BACKEND` footgun 处置**——给 CLI 补一等公民的 `--backend` flag
  （和/或 config 字段），覆盖目前依赖 env 的缺口；随后把 env 降级为带响亮 WARN 的 debug
  override（保留 CI/调试能力，但启动生效即告警）。改运行时行为，必须 code-review。结合当前
  内测/发版阶段，优先级较高。

## 6. 维护约定

- 新增 / 删除 / 改名任何 A 类 env 旋钮时，**同步更新本文 §2 清单**（含 `file:line`）。
- B 类不必逐一登记，但若某 env 从"仅测试"演变为"进产品路径"，须升级登记为 A 类并走决策门。
