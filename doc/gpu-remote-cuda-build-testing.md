# 远程 CUDA 编译 / 测试流程（dev49 + win-builder）

> Mac 本机编不了 CUDA，subprocess 自报不可信 → CUDA 改动必须在 dev49（Linux/NVIDIA）
> 和/或 win-builder（Windows/NVIDIA）亲跑。本文档是这两台机器的现成 recipe，免每次重新折腾。
> 首次落地：scrum-311（2026-07-01）；scrum-313 从 scratchpad 提升为 tracked 文档并加索引（避免每
> 次新 session 对两台机器从头摸索）。关联记忆 `reference_win_builder_cuda_build_recipe`、
> `reference_dev49_linux_bench`、`reference_win_test_machine`（三者均以本文档为单一真源）。
>
> **与 [`windows-remote-testing.md`](windows-remote-testing.md) 的分工**：那份讲的是 **GUI VSync
> 物理桌面性能测试**（`win_remote_test.sh` + `win_test_watcher.ps1`，需真实显示器 session）；**本文档**
> 讲的是 **CUDA build + parity/正确性验证**（dev49 docker + win-builder BuildTools，headless 即可）。
> 二者场景正交，别混用。吞吐 bench 口径见 [`performance-testing.md`](performance-testing.md)。

## 0. 何时需要

- 任何触及 `src/core/backend/cuda_trace_backend.*` 或三后端共享头（`trace_backend.hpp`、
  `pcg_shared.h`、`*_shared.h`）、`SimData`、simulator/server/stats 的改动。
- 验收口径：**CUDA parity battery 10/10**（exit-seam 2 + filter 4 + multi-MS 4）+（按需）CLI 冒烟。
- perf bench 才需要 dev49 idle-gate 锁频窗口；**纯正确性验证不需要等窗口**，随时可跑。

## 1. 通用约定（两台机器都适用）

- **un-skip 闸**：CUDA parity pytest 用 `pytest.mark.skipif(platform in (Linux,Windows) and
  os.environ["LUMICE_HAS_CUDA"]=="1")`。**必须设 `LUMICE_HAS_CUDA=1`** 才会跑（不是
  `LUMICE_CUDA_ENABLED`——那个是 runtime 路由用的，两个都要设）。
- **CApiRunner 找 lib**：`LUMICE_LIB` 指向 shared lib（`.so`/`.dll`），cudart 需与之同目录或在 PATH。
- **parity battery 三文件**：`test/parity-cross-backend/backend/test_cuda_{exit_seam,filter,multi_ms}_parity.py`。
- **别信 subprocess 自报**：读 build EXIT、grep 告警、亲看 pytest 计数（`N passed`）。

## 2. dev49（Linux / RTX 4060Ti / `ssh 49-GPU`，主机 wztest49）

- **源码树**：`/home/work/zjj/ice-halo-sim`（rsync 同步改动：
  `rsync -az --delete src/ 49-GPU:/home/work/zjj/ice-halo-sim/src/` + 同理 test/，
  排除 `__pycache__`/`references/`）。非 git 仓，rsync 覆盖即可。
- **CUDA docker 镜像**：
  `registry.cn-zhangjiakou.aliyuncs.com/weizhendev/wzffm-centos7-cuda11.6:20250811T094454Z-e7555768`
  （CUDA 11.6）。
- **build dir**：`build/cmake_build_cuda`（已配好，Ninja，`LUMICE_CUDA_ENABLED=ON`，86-virtual PTX）。
- **build**：
  ```bash
  ssh 49-GPU 'cd /home/work/zjj/ice-halo-sim && IMG=<镜像> && \
    docker run --rm --gpus all -v /home/work/zjj/ice-halo-sim:/work \
      -w /work/build/cmake_build_cuda $IMG bash -lc "ninja"'
  ```
  单 TU 强制重编：`rm -f CMakeFiles/lumice_obj.dir/src/core/backend/cuda_trace_backend.cu.o &&
  ninja <该 .o target>`；grep `177-D`/`declared but never referenced` 查死代码告警
  （既有噪声：spdlog/fmt 的 `#128-D loop is not reachable`、`trace_backend.hpp` multi-line comment，
  与改动无关）。
- **parity**：docker 默认 `/usr/local/bin/python3` **无 pytest/numpy**，需一次性装：
  ```bash
  docker run --rm --gpus all -v /home/work/zjj/ice-halo-sim:/work -w /work $IMG bash -lc '
    python3 -m pip install -q pytest numpy Pillow
    export LUMICE_HAS_CUDA=1 LUMICE_CUDA_ENABLED=1
    export LUMICE_LIB=/work/build/Release/lib/liblumice.so
    export LD_LIBRARY_PATH=/work/build/Release/lib:/usr/local/cuda/lib64:$LD_LIBRARY_PATH
    python3 -m pytest -v test/parity-cross-backend/backend/test_cuda_{exit_seam,filter,multi_ms}_parity.py'
  ```
  （ctest 的 `CudaMultiMsParity` 因 cmake `PYTEST_EXECUTABLE` cache 指向不存在路径会 Not Run，
  直接 `python3 -m pytest` 绕过。）耗时 ~7min。
- **CLI 冒烟**：`Lumice -f examples/config_example.json --backend cuda -o /tmp`
  （`-o` 是**目录**不是文件），grep `Stats: ... crystals=N`。
- **坑**：shared 机 CPU 争用使 host-bound 吞吐剧烈抖动（只信 interleaved，perf 才在意）；
  rsync 对 root 生成的历史文件报 Permission denied 是环境噪声，忽略。

## 3. win-builder（Windows / GTX 1070Ti sm_61 / `ssh win-builder`，主机 DESKTOP-4G9MT1O）

- **源码树**：`C:\lumice-src`。**同步坑**：`scp` 到 `C:/lumice-src/深层/路径` 会因盘符冒号
  解析失败 → **tarball 法**：
  ```bash
  git diff --name-only <base>..HEAD -- src/ test/ > files.txt
  tar czf changed.tgz -T files.txt
  scp changed.tgz win-builder:C:/lumice-test/changed.tgz   # 浅路径 scp OK
  ssh win-builder "powershell -NoProfile -Command \"cd C:/lumice-src; tar xzf C:/lumice-test/changed.tgz\""
  ```
  （Windows 10+ 自带 `tar`=bsdtar。）验证：`Select-String -Path <file> -Pattern <新符号>`。
- **工具链**：VS BuildTools `C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools`，
  MSVC 工具集 **14.39.33519**（scrum-309 定；14.51 太新 CUDA 11.7 收不了）。CUDA `v11.7`。
  cmake/ninja 不在 PATH，走 BuildTools 内置（build.ninja 已 baked 绝对路径，纯 ninja 增量即可）。
- **配好的 CUDA build dirs**（CMakeCache+build.ninja 齐全，`LUMICE_CUDA_ENABLED=ON`）：
  - `C:\lumice-src\build\win-cuda`（静态 CLI）
  - `C:\lumice-src\build\win-cuda-shared`（**shared→liblumice.dll，parity 用这个**）
  - `C:\lumice-src\build\win-gui-cuda`（GUI）
- **build**（写 .bat scp 过去，`cmd /c` 跑——**别用 PowerShell `&` 后台化，会 AmpersandNotAllowed**；
  同步跑，我方 bash `run_in_background` 拿 notification）：
  ```bat
  @echo off
  call "C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat" -vcvars_ver=14.39 >nul 2>&1
  set NINJA="C:\Program Files (x86)\Microsoft Visual Studio\18\BuildTools\Common7\IDE\CommonExtensions\Microsoft\CMake\Ninja\ninja.exe"
  cd /d C:\lumice-src\build\win-cuda-shared
  %NINJA%
  echo NINJA_EXIT=%ERRORLEVEL%
  ```
  产物 `C:\lumice-src\build\Release\bin\lumice.dll` + `Lumice.exe`（cudart64_110.dll 同目录）。
  既有噪声告警：fmt/spdlog `#27-D character out of range`、`math.hpp` C4305 truncation（无关）。
- **parity**：embeddable python `C:\lumice-test\py311\python.exe`（**已装 pytest+numpy**）：
  ```bat
  set LUMICE_HAS_CUDA=1
  set LUMICE_CUDA_ENABLED=1
  set LUMICE_LIB=C:\lumice-src\build\Release\bin\lumice.dll
  set PATH=C:\lumice-src\build\Release\bin;C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v11.7\bin;%PATH%
  cd /d C:\lumice-src
  C:\lumice-test\py311\python.exe -m pytest -v test\parity-cross-backend\backend\test_cuda_exit_seam_parity.py test\parity-cross-backend\backend\test_cuda_filter_parity.py test\parity-cross-backend\backend\test_cuda_multi_ms_parity.py
  ```
  耗时 ~7min，1070Ti/sm_61 走 compute_61 PTX JIT。
- **PS over ssh 通用坑**：默认 shell 是 PowerShell，内联引号 + 中文 locale 易乱码/解析错 →
  **一律写 .ps1/.bat 文件 scp 过去，再 `powershell -NoProfile -ExecutionPolicy Bypass -File`
  或 `cmd /c`**。用完清理 `C:\lumice-test` 下的临时脚本（共享机卫生）。

## 4. 一次完整改动的推荐顺序

1. 本机改代码 + Mac build/单测/Metal parity（`./scripts/build.sh -tj release`）。
2. rsync/tarball 同步到 dev49 + win-builder。
3. 两台各自 build（查 EXIT + 告警）。
4. 两台各自跑 CUDA parity battery（各 10/10）。
5.（按需）CLI 冒烟核 `Stats`/出图。
6. commit + push + PR，CI 的 `windows-cuda-compile` job 再兜一层 Windows 编译。
