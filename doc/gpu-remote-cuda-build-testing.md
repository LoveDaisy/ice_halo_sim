# 远程 CUDA 编译 / 测试流程

> Mac 本机编不了 CUDA，subprocess 自报不可信 → CUDA 改动必须在 **CUDA 参照机**上亲跑。
> 本文档是那套流程的现成 recipe，免每次重新折腾。
>
> ⭐ **本文档正文只写角色，不写主机名。** 「CUDA 参照机（Linux）」「CUDA 参照机（Windows）」
> 具体是哪台机器、仓库在哪、CUDA 装在哪，全部在
> [`machines.md`](machines.md) —— 那是主机绑定的**单一真源**。换机器只改那一份，本文档不动。
> 这个分层是有代价换来的：上一代验证机整体不可用时，正文里每一条写死的 ssh 别名与路径同时
> 失效，把本来仍然有效的协议部分一起拖成了可疑内容。
>
> **与 [`windows-remote-testing.md`](windows-remote-testing.md) 的分工**：那份讲的是 **GUI VSync
> 物理桌面性能测试**（需真实显示器 session）；**本文档**讲的是 **CUDA build + parity/正确性验证**
> （headless 即可）。二者场景正交，别混用。吞吐 bench 口径见
> [`performance-testing.md`](performance-testing.md)。

## 0. 何时需要

- 任何触及 `src/core/backend/cuda_trace_backend.*` 或三后端共享头（`trace_backend.hpp`、
  `pcg_shared.h`、`*_shared.h`）、`SimData`、simulator/server/stats 的改动。
- 验收口径：**CUDA parity battery 10/10**（exit-seam 2 + filter 4 + multi-MS 4）+（按需）CLI 冒烟。
- perf bench 才需要锁频 / idle 窗口；**纯正确性验证不需要等窗口**，随时可跑。

## 1. 通用约定（两个角色都适用）

- **un-skip 闸**：CUDA parity pytest 用 `pytest.mark.skipif(platform in (Linux,Windows) and
  os.environ["LUMICE_HAS_CUDA"]=="1")`。**必须设 `LUMICE_HAS_CUDA=1`** 才会跑（不是
  `LUMICE_CUDA_ENABLED`——那个是 runtime 路由用的，**两个都要设**）。
- **CApiRunner 找 lib**：`LUMICE_LIB` 指向 shared lib（`.so`/`.dll`），cudart 需与之**同目录**。
  ⚠️ **仅 Linux 上"或在 PATH（`LD_LIBRARY_PATH`）"成立——Windows 不成立**：Python 3.8+ 起
  `ctypes.CDLL` 在 Windows 上不再搜索 `PATH`（改为显式 `add_dll_directory`），把 CUDA `bin`
  塞进 `PATH` 并不能让 `cudart64_12.dll` 被加载；Windows 侧实测可行的做法见 §3。
- **parity battery 三文件**：`test/parity-cross-backend/backend/test_cuda_{exit_seam,filter,multi_ms}_parity.py`。
- **别信 subprocess 自报**：读 build EXIT、grep 告警、亲看 pytest 计数（`N passed`）。
  ⚠️ 管道会吃掉退出码——`cmd | tail` 的 `$?` 是 `tail` 的。要么读前台命令的 `$?`，
  要么 `set -o pipefail`。
- **arch 表**：本仓库默认 `CMAKE_CUDA_ARCHITECTURES` 以 `61-virtual` 为 PTX floor。
  ⚠️ **CUDA 13 已移除 `compute_61`**，用它构建会直接 `nvcc fatal`，与你改的东西无关。
  参照机须装 CUDA 12.x。做吞吐 bench 时还要追加与卡匹配的 `-real` arch，否则跑的是
  驱动 JIT 出来的 PTX 而非原生 SASS（详见 [`machines.md`](machines.md) 的「已知坑」）。

## 2. CUDA 参照机（Linux）

本节命令假设你已按 [`machines.md`](machines.md) 设好这几个绑定（值在那份文件里，不在这里）：

| 变量 | 含义 |
|---|---|
| `$HOSTALIAS` | 该角色的 ssh 别名 |
| `$REPO` | 机器上的仓库路径 |
| `$ENVSH` | 环境激活脚本（设 CUDA 版本 + venv + arch 表） |

- **同步改动**：仓库是 git 工作树，改动经局域网 rsync 推送即可：
  ```bash
  rsync -az --exclude '__pycache__' src/ $HOSTALIAS:$REPO/src/   # test/ 同理
  ```
  ⚠️ 首次同步整棵树时**别在机器上从 GitHub 拉**——见 `machines.md` 的网络实测，
  从开发机 rsync 过去快得多，`build/cpm_cache` 尤其（省掉全部依赖下载）。

- **build**（shared flavor，parity 需要 `.so`）：
  ```bash
  ssh $HOSTALIAS "bash -lc 'source $ENVSH && ~/lumice-build-cuda.sh'"
  ```
  判据 = 日志里 `CONFIGURE_EXIT` / `BUILD_EXIT` / `INSTALL_EXIT` **三个都是 0**。
  单 TU 强制重编：删掉对应 `.o` 再 `ninja` 那个 target。
  既有噪声告警（与改动无关，别追）：`nvcc warning : Support for offline compilation for
  architectures prior to '<compute/sm/lto>_75' will be removed`（这是 `61-virtual` floor 的
  必然产物）、spdlog/fmt 的 `#128-D loop is not reachable`、`math.hpp` 的 C4305 truncation。

- **parity（⭐首选 pytest）**：
  ```bash
  source $ENVSH
  export LUMICE_HAS_CUDA=1 LUMICE_CUDA_ENABLED=1
  export LUMICE_LIB=$REPO/build/Release/shared/lib/liblumice.so
  export LD_LIBRARY_PATH=$REPO/build/Release/shared/lib:$LD_LIBRARY_PATH
  python -m pytest -v -m slow \
    test/parity-cross-backend/backend/test_cuda_{exit_seam,filter,multi_ms}_parity.py
  ```
  判据 = 退出码 0 且看到 `N passed`。
  - 历史坑（**已修复**，留档以免误判为新问题）：这套 pytest 曾 `Fatal Python error: Aborted`
    （`free(): invalid next size`），根因是 `test/e2e/capi_runner.py` 的 ctypes 镜像结构
    （`LUMICE_RenderResult` / `LUMICE_ServerConfig`）比 C 侧头文件 sizeof 小 8/4 字节，
    每次 poll 堆越界写、污染相邻堆块。现已补齐 ctypes 字段 + C++ 侧
    `static_assert(sizeof(...))` 编译期护栏。**再遇到类似崩溃属新问题，别按旧结论绕过。**
  - ctest 的 `CudaMultiMsParity` 可能因 cmake `PYTEST_EXECUTABLE` cache 指向不存在的路径而
    Not Run；直接 `python -m pytest` 绕过。（在激活了 venv 的 shell 里 configure 可以让
    cmake 找到正确的 pytest，`find_program(... HINTS ENV PATH)`。）

- **gtest parity（快速冒烟补充）**：C++ `parity_test` 只要 ~1s，适合改完先跑一遍再上 pytest 全量：
  ```bash
  export LUMICE_HAS_CUDA=1 LUMICE_CUDA_ENABLED=1
  export LD_LIBRARY_PATH=$REPO/build/Release/shared/lib:$LD_LIBRARY_PATH
  $REPO/build/Release/shared/bin/parity_test --gtest_filter="Cuda*:*ComponentMask*"
  ```
  覆盖 `CudaRichExit`（含 2 个 in-test `GTEST_SKIP`）、`CudaBackendCrystalCount`、
  `CudaRngHiWiring`、`CudaComponentMaskParity`（染色 parity）、`CudaKShapeFilterParity`。
  **判据 = 进程退出码 0 且 0 failed**（skip 不算失败）。

- **染色密度验证**（任何 Y-lane composite 改动的功能门——parity 只测 mask，密度是它的盲区）：
  three_arcs 2M 跑 `--backend cuda` vs `--backend cpu`，比 `img_01_components.jpg` 的 lit-px
  密度（`test/e2e/image_utils.py::classify_pixels_by_color_direction`）。健康时两者接近。

- **CLI 冒烟**：
  ```bash
  $REPO/build/cmake_install/shared/Lumice -f examples/config_example.json --backend cuda -o /tmp
  ```
  （`-o` 是**目录**不是文件。）确认日志里有 `preferred_backend=cuda → routing via CudaTraceBackend`，
  再 grep `Stats: ... crystals=N`。
  ⚠️ **`examples/config_example.json` 有 4 个 renderer，而 TraceBackend 路径只支持单 renderer**
  ⇒ 它会打印 `falling back to legacy CPU` 然后用 CPU 跑完。这是配置属性不是故障，但**这条冒烟
  因此不能证明 CUDA 真的跑了仿真**——要证明路由，看上面那行 `routing via CudaTraceBackend`；
  要证明算得对，用 parity battery。

- **GUI 测试**：该机是 WSL，`gui_test` 需要 `xvfb-run`（WSLg 的 X server 在 ssh 会话里不可达）：
  ```bash
  xvfb-run -a build/Release/static/bin/gui_test \
    --fixed-dt --filter "modal_layout,defaults_panel_layout" --no-user-config
  ```
  这两组正是 CI 在软件光栅下运行的组。⚠️ **完整视觉池里的 `crystal_preview_*` 与
  `capture_harness` 场景在软件光栅下达不到 40 dB 阈值**，属已知边界，不是回归——
  判据与可运行组的界定见 `testing-architecture.md`。

## 3. CUDA 参照机（Windows）

绑定同样见 [`machines.md`](machines.md)（仓库路径、BuildTools 路径、CUDA 路径、构建脚本名）。

- **同步改动**：`scp` 到带盘符冒号的深层路径会解析失败 → **tarball 法**：
  ```bash
  git diff --name-only <base>..HEAD -- src/ test/ > files.txt
  tar czf changed.tgz -T files.txt
  scp changed.tgz $WINALIAS:C:/lumice-test/changed.tgz     # 浅路径 scp OK
  ```
  再在机器上解包到仓库根。（Windows 10+ 自带 `tar` = bsdtar。）
  验证：`Select-String -Path <file> -Pattern <新符号>`。

- **PS over ssh 通用坑**（⭐ 每次都会撞）：默认 shell 是 **PowerShell 5.1**——
  `&&` 不是有效语句分隔符；内联引号 + 中文 locale 极易乱码或解析错；本地 bash 还会先吃掉 `$`。
  ⇒ **一律写 `.ps1` / `.bat` 文件 scp 过去再执行**（`powershell -NoProfile -ExecutionPolicy
  Bypass -File` 或 `cmd /c`），或用 `powershell -EncodedCommand`（UTF-16LE + base64）传整段。

- **build**：调 `machines.md` 里记的构建脚本；它内部 `call vcvars64.bat`、把 BuildTools 自带的
  cmake/ninja 与 CUDA 的 `bin` 加进 PATH，然后 configure + build + install。
  判据 = `CONFIGURE_EXIT` / `BUILD_EXIT` / `INSTALL_EXIT` 三个都是 0。
  ⚠️ 长时间构建**别用 `Start-Process` 分离**（实测会静默失败、日志为空）；
  要么同步跑（我方 bash 用后台任务拿 notification），要么用 `schtasks` 建一次性计划任务。

- **⚠️ 这条路径至今没有任何 CI job 走过，所以它的红只会在这台机器上出现。**
  Windows 上「带 CUDA 编译测试」这个配置的覆盖缺口**仍然存在**，而原因不是漏了 Windows：
  主矩阵的 Windows job 开了 `BUILD_TEST` 但没开 CUDA、CUDA 编译 job 开了 CUDA 但没开
  `BUILD_TEST`——**两半都在，交集为空**。这个缺口一次攒下三处 MSVC 不兼容
  （两个 `test_cuda_*` 用 POSIX `setenv`/`unsetenv`、一个 bench 文件用 `M_PI`），
  三周无信号，直到这台机器第一次以 `BUILD_TEST=ON` + CUDA 构建才炸出来；它们已经修掉
  （`test/support/env_var.hpp` 是全树唯一持有该 `#ifdef` 的地方），但**攒下它们的缺口没有**。
  ⇒ 在缺口补上之前，动 `test/` 或 `bench/` 后请在这台机器上真跑一次本节的构建，
  别把「mac/Linux 绿」当成 Windows 也绿。

- **parity**：
  ```bat
  set LUMICE_HAS_CUDA=1
  set LUMICE_CUDA_ENABLED=1
  set LUMICE_LIB=<仓库>\build\Release\shared\bin\lumice.dll
  copy <CUDA>\bin\cudart64_12.dll <仓库>\build\Release\shared\bin\
  set PYTHONUTF8=1
  python.exe -m pytest -v -m slow test\parity-cross-backend\backend\test_cuda_exit_seam_parity.py ^
    test\parity-cross-backend\backend\test_cuda_filter_parity.py ^
    test\parity-cross-backend\backend\test_cuda_multi_ms_parity.py
  ```
  - ⚠️ **不要**把 CUDA `bin` 塞进 `PATH` 期待 `LUMICE_LIB` 能带出 `cudart64_12.dll`——
    Python 3.8+ 起 `ctypes.CDLL` 在 Windows 上不再搜索 `PATH`（见 §1）。实测可行做法是把
    `cudart64_12.dll` **复制到被加载的 dll 同目录**（即 `LUMICE_LIB` 所在目录）；照此做之后
    CUDA parity battery 在 Windows 上第一次真正跑通。
  - ⚠️ **`set PYTHONUTF8=1` 必带**：中文 Windows 控制台默认 GBK codec，parity 测试的
    docstring/traceback 若含非 ASCII 字符（数学符号、破折号），pytest 渲染输出时会
    `UnicodeEncodeError` 崩掉 —— parity 已算完但断言未执行，**假失败遮蔽真结果**。
    `PYTHONUTF8=1` 强制 Python UTF-8 I/O，与 locale 无关。测试消息串本身已改 ASCII（治本），
    此为第二道保险。

- **吞吐 bench**：⚠️ `scripts/bench_throughput.py` 的默认二进制探测**不带 `.exe`**
  ⇒ Windows 上必须显式 `set LUMICE_BENCH_BIN=<仓库>\build\cmake_install\static\Lumice.exe`，
  否则 preflight 直接报 `Binary missing or not executable`。

## 4. 一次完整改动的推荐顺序

1. 本机改代码 + Mac build/单测/Metal parity（`./scripts/build.sh -tj release`）。
2. 同步到 CUDA 参照机（Linux rsync / Windows tarball）。
3. 各自 build，**逐个查 EXIT 码 + grep 告警**。
4. Linux 参照机跑 CUDA parity battery（10/10）；Windows 侧同一 battery 已实测跑通
   （`pytest` 报 `11 passed`），前提是应用了 §3 的 DLL 同目录做法——不是只验编译。
5.（按需）CLI 冒烟核路由与 `Stats`、染色密度门。
6. commit + push + PR，CI 的 `windows-cuda-compile` job 再兜一层 Windows 编译。

> ⚠️ **两个 CUDA 参照机角色目前由同一台物理机的两个环境承担**（见 `machines.md`）。
> 因此第 3、4 步**不要并行跑**——共享 CPU 与 GPU 会让任何吞吐数字失真，
> 正确性验证也会被拖慢到看起来像挂起。
