# 验证机器清单（角色 → 主机绑定的单一真源）

本文件是**唯一**记录「哪个角色由哪台机器承担、路径与版本是什么」的地方。

## 为什么单独一个文件

远程验证的文档里混着两种寿命完全不同的东西：

| 层 | 判别法 | 换硬件后 | 写在哪 |
|---|---|---|---|
| **角色 / 协议** | 在讲**「验证什么、怎么判定通过」** | 仍然成立 | 各 recipe 正文，只写**角色名** |
| **主机绑定** | 在讲**「去哪台机、敲哪条命令、路径在哪」** | 全部作废 | **本文件**，且只有本文件 |

把这两层写在一起的代价是实测过的：上一代验证机随环境变更整体不可用时，两份 recipe 里
每一条 ssh 别名、docker 镜像 tag、build 目录、工具集版本同时失效，而正文里真正有效的
协议部分（un-skip 闸怎么设、parity battery 是哪三个文件、验收口径）被一起拖成了可疑内容。
根源是**把主机名当角色名用**——同一个词既指一台具体机器，又指「CUDA 参照机」这个职责。

⇒ **改机器只需要改本文件**；recipe 正文不含任何主机名，因此不必跟着改。

## 角色 → 主机绑定

| 角色 | 当前承担者 | 用途 |
|---|---|---|
| **CUDA 参照机（Linux）** | `home-wsl` | CUDA build、parity/正确性验证、CUDA 吞吐 bench |
| **CUDA 参照机（Windows）** | `home-win` | MSVC + CUDA 编译、Windows 产物验证、Windows 吞吐 bench |
| **GUI 物理桌面机** | `home-win` | 需要真实显示器 session 的 GUI VSync 性能测试 |
| **Metal 参照机** | 开发用 Mac | Metal 后端 build / parity / 吞吐 bench |

`home-win` 与 `home-wsl` 是 ssh(1) 别名，须在你的 `~/.ssh/config` 里配置。

> ⚠️ **`home-win` 与 `home-wsl` 是同一台物理机**的原生 Windows 与 WSL2 环境，
> 共享同一 CPU 与同一块 GPU。**在一侧跑 bench 时另一侧必须闲置**——这不是理论顾虑：
> 实测中一侧跑测试套件期间，另一侧的 legacy CPU 基线 CoV 冲到 44.1%，
> 触发了吞吐 harness 的重测逻辑。

## 硬件

| 项 | 值 |
|---|---|
| CPU | Ryzen 9 9950X（16C / 32T） |
| 内存 | 94 GB（WSL 侧可见） |
| GPU | **RTX 5090 D 32GB（Blackwell, sm_120）**，driver 591.86 |

## `home-wsl`（Ubuntu 24.04.4 / WSL2）

| 项 | 值 |
|---|---|
| 仓库 | `~/Codes/lumice` |
| 环境激活 | `source ~/lumice-env.sh`（设 CUDA 12.9 + venv + arch 表） |
| CUDA | **12.9**，装在 `/usr/local/cuda-12.9` |
| Python | `~/venvs/lumice`（Ubuntu 24.04 为 PEP668 托管，必须用 venv） |
| CUDA 构建脚本 | `~/lumice-build-cuda.sh`（shared flavor） |

- **CUDA 12.9 与该机上既有的 13.0 并存**，`/usr/local/cuda` 符号链接**仍指向 13.0**，
  本项目通过 `~/lumice-env.sh` 单独指向 12.9 ⇒ 该机上的其他项目不受影响。
  为什么必须是 12.x 见下方「已知坑 1」。
- GPU 直通经 `/dev/dxg` + `/usr/lib/wsl/lib/`。
  ⚠️ **`nvcc` / `nvidia-smi` 不在 PATH 里不等于没有 GPU**——WSL 的 NVIDIA 运行时装在
  `/usr/lib/wsl/lib/`，要按该路径查。

## `home-win`（Windows 11 Pro）

| 项 | 值 |
|---|---|
| 仓库 | `C:\Users\15093\Codes\lumice` |
| 工具链 | VS Build Tools 2022 17.14.37 / MSVC 14.44.35207 |
| cmake / ninja | **VS Build Tools 自带**，位于 `Common7\IDE\CommonExtensions\Microsoft\CMake\` 下 |
| CUDA | **12.9**，标准路径 `C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v12.9` |
| 构建脚本 | `C:\Users\15093\win_build.cmd`（产物-only 版 `win_build_product.cmd`） |
| 吞吐脚本 | `C:\Users\15093\win_bench.cmd` |

- ssh 默认 shell 是 **PowerShell 5.1**：`&&` 不是有效的语句分隔符。多条命令请分开发，
  或用 `-EncodedCommand` 传整段脚本（本地 shell 容易吃掉 `$`）。
- ssh 会话**自带管理员令牌**，需要提权的安装可以直接远程执行。
- ⚠️ `scripts/bench_throughput.py` 的默认二进制探测**不带 `.exe`** 后缀
  ⇒ 在 Windows 上必须显式设 `LUMICE_BENCH_BIN`。

## 已知坑（都由一手实测确认）

1. **CUDA 13 编译不了本仓库。** `nvcc -arch=compute_61` 报
   `nvcc fatal : Unsupported gpu architecture 'compute_61'`，而本仓库的 arch floor 就是
   `61-virtual`（见 `CMakeLists.txt` 的 `CMAKE_CUDA_ARCHITECTURES` 默认表）。
   ⇒ CUDA 参照机必须装 12.x（12.9 接受 `compute_61`，只报 deprecation warning）。
   **吞吐 bench 另需显式追加 `120-real`**，否则 sm_120 的卡跑的是驱动 JIT 出来的
   Pascal PTX 而不是原生 SASS，不够 bench 保真度：
   `-DCMAKE_CUDA_ARCHITECTURES="61-virtual;75-real;86-real;89-real;120-real"`
2. **WSL 上 GLFW 会选中 Wayland 后端然后段错误**（`_glfwInitWayland` →
   `wl_proxy_get_version` SIGSEGV）。成因是 WSLg 把 wayland socket 放进
   `XDG_RUNTIME_DIR`，于是 `wl_display_connect` 会成功但后续不兼容。
   ⇒ configure 时加 `-DGLFW_BUILD_WAYLAND=OFF -DGLFW_BUILD_X11=ON`。
   另外 **WSLg 的 X server 在 ssh 会话里不可达**（`X11: Failed to open display :0`），
   ⇒ 跑 GUI 测试用 `xvfb-run`，这也正是 CI 的 Linux leg 所用的方式。
3. **网络：GitHub 慢，Docker Hub 不通。** 实测 GitHub 直下约 69 KB/s、
   Docker Hub 连不上；而 apt（阿里云镜像）、pypi、NVIDIA 的
   `developer.download.nvidia.cn`（26–30 MB/s）都很快。
   ⇒ CPM 依赖不要在这两台机器上下载，从开发机经局域网 rsync `build/cpm_cache`
   （约 282 MB）过去。⚠️ Mac 上的缓存里**没有 `glad`**（它是 `WIN32` 才用的依赖，
   Mac 从不构建它），Windows 首次构建会自行拉取。

## 相关文档

- [`gpu-remote-cuda-build-testing.md`](gpu-remote-cuda-build-testing.md) — CUDA build +
  parity/正确性验证的协议与操作步骤（headless）
- [`windows-remote-testing.md`](windows-remote-testing.md) — GUI VSync 物理桌面性能测试
- [`performance-testing.md`](performance-testing.md) — 吞吐 bench 口径
