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

> task-287 起，所有 A 类 env 的读取都集中在 **`src/util/env_knobs.cpp`**（仓库内唯一允许
> `getenv("LUMICE_*")` 的地方，由 `scripts/check_policies.py` 强制）。每个旋钮设为非默认值时
> 首次读取会打一行日志——perf/调试旋钮 INFO，`LUMICE_TRACE_BACKEND` WARN。下表"读取入口"
> 统一为 `env_knobs.cpp`，"消费方"列出实际使用点。

| 变量 | 读取入口 / 消费方 | 性质 | 显式替代路径 |
|---|---|---|---|
| `LUMICE_TRACE_BACKEND` | `env::TraceBackendOverride` ← `simulator.cpp` `CreateBackend` / `server.cpp` `ResolveMetalRoute` | 选后端 `legacy`/`cpu_backend`/`metal`——**曾经的 footgun，已降级** | **CLI `--backend auto\|cpu\|metal`**（`src/main.cpp`）+ C API `LUMICE_SetPreferredBackend` / GUI checkbox。env 仅作 debug/CI 覆盖，生效即 WARN |
| `LUMICE_DISPATCH_RAY_NUM` | `env::DispatchRayNum` ← `server.cpp` `GenerateScene` | GPU dispatch 粒度（perf 旋钮） | 无（纯实验旋钮，启动 INFO 可观测） |
| `LUMICE_GEOM_CLOCK` | `env::GeomClock` ← `simulator.cpp` `Run` | legacy CPU 几何时钟 K = 每个采样形状服务多少光线。1 = 每光线一个新形状；默认 32 = `kSmallBatchRayNum`（该值是光线分批 stride，几何重采样搭它顺风车，**并非为采样质量选定**）。⚠️ **安全域 [1, SimBatch 大小]，超出会破坏堆**——而 SimBatch = `LUMICE_DISPATCH_RAY_NUM`（legacy 默认 `server.cpp` `kDefaultRayNum=128`）→ **该上界是 dispatch 设置的推论，不是本旋钮的固有属性**：默认 dispatch 下实测 128→`139 134 139`；**dispatch=65536 下 128/1024/32768 全部干净通过**。机制（缓冲 `ray_num*2` vs 工作集 `geom_clock*2`）与实测退出码见 `env_knobs.hpp` `GeomClock()` | 无（纯实验旋钮，启动 INFO 可观测） |
| `LUMICE_COMMIT_RAY_NUM` | `env::CommitRayNum` ← `server.cpp` `ConsumeData` | commit 粒度（perf 旋钮） | 无（纯实验旋钮，启动 INFO 可观测） |
| `LUMICE_XYZ_DRAIN_BATCHES` | `env::XyzDrainBatches` ← `simulator.cpp` `Run` | scrum-312 第三时钟 drain cadence 上限（每 N 个 device-fused batch 强制回读 XYZ；perf/精度旋钮，CUDA 路径） | 无（纯实验旋钮，启动 INFO 可观测；GUI 显示节奏主要由 producer-pause flush 驱动，此为 burst 内上界） |
| `LUMICE_BATCH_RAY_NUM` | `env::CommitRayNum`（fallback） | 已废弃别名（deprecation WARN） | 由 `COMMIT_RAY_NUM` 取代 |
| `LUMICE_DISABLE_DEVICE_GEN` | `env::DisableDeviceGen` ← `metal_trace_backend.mm` Impl ctor | 关 device-gen（实验/调试旋钮） | 无（启动 INFO 可观测） |
| `LUMICE_DISABLE_METAL_SOURCE_COMPILE` | `env::DisableMetalSourceCompile` ← `metal_trace_backend.mm` `LoadMetalLibrary` | 关 metal 源码编译（也被回归 sentinel 当门禁用） | 无（启动 INFO 可观测） |
| `LUMICE_WL_POOL_SIZE` | `env::WlPoolSize` ← `metal_trace_backend.mm` `EnsureWlPoolBuffer` | 波长池大小（实验旋钮） | 无（启动 INFO 可观测） |

关于 `LUMICE_TRACE_BACKEND` 的处置（task-287 已落地）：

- 它本就是 debug/CI 覆盖（代码注释自述 "debug/CI override"）。task-287 给 CLI 补了一等公民
  `--backend` flag（之前 CLI 选后端只能靠此 env，是真正的缺口），并在 env 生效时打 WARN，
  正式把它降级为覆盖项而非主路径。
- 主路径：`--backend`（CLI）/ `LUMICE_SetPreferredBackend`（C API）/ GUI checkbox（经 task-281
  `LUMICE_IsBackendAvailable` gate）。env 覆盖于其上，保留是因为 CI/parity 测试依赖它强制 backend，
  删除会破坏测试。
- 注意：CLI `--backend auto` 与 `--backend cpu` 当前行为相同（都选 CPU 路由），且二者都会被
  `LUMICE_TRACE_BACKEND` 覆盖——`--backend cpu` 不是"硬钉 CPU 屏蔽 env"。这是有意设计（env 覆盖
  位于 `CreateBackend` 内部，对所有 preference 一视同仁），非缺陷。
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

软约束会失效，所以这道门固化为代码层的集中入口 + 自动门禁（§5），而非仅靠本文档自律。

## 5. 门禁（这道策略怎么"长出牙齿"）

策略不靠自觉，靠检查 artifact 的确定性门禁（a04：固化为自动化门禁；a01：验证 artifact 而非
自我声明）。task-287 落地：

- **`scripts/check_policies.py`**（确定性，无版本依赖）：
  - getenv 集中化——`getenv("LUMICE_*")` 只能出现在 `src/util/env_knobs.cpp`，其余一律 fail；
  - getenv 注册——`env_knobs.cpp` 引用的每个 `LUMICE_*` 名字必须出现在本文档，否则 fail；
  - GUI/core API 边界——`src/gui/**` 不得直接 `#include "core/.."`/`"config/.."`；
  - `using namespace` 禁用（src/）。
- **CI `policy` job**（`.github/workflows/ci.yml`，权威门禁，绕不过）：跑 `check_policies.py`。
  clang-format 由既有的独立 `format-check` job 负责（两者分开，互不耦合）。
- **本地 pre-commit hook**（`scripts/hooks/pre-commit`，装：`./scripts/install-hooks.sh`）：
  非交互镜像 CI 的 policy + format 检查（后者用 `scripts/format.sh --check` 对暂存文件），提交前
  快速反馈；`git commit --no-verify` 可单次绕过；不动 git-lfs 其他 hook。
- **判断类项**（机械化不了的，如"这个 env 是否改变用户输出"）放 `.github/PULL_REQUEST_TEMPLATE.md`，
  由 reviewer 用正交视角核对，而非作者在 hook 里自答。

## 6. 维护约定

- 新增 / 删除 / 改名任何 A 类 env 旋钮时：在 `src/util/env_knobs.cpp` 增删读取函数，并**同步
  更新本文 §2 清单**——getenv 注册检查会拿本文当白名单，名字对不上 CI 直接红。
- B 类不必逐一登记，但若某 env 从"仅测试"演变为"进产品路径"，须升级登记为 A 类并走决策门。
- 新增任何 `getenv("LUMICE_*")` 都必须走 `env_knobs.cpp`，否则 `check_policies.py` 拦截。
