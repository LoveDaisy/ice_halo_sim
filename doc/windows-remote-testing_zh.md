# Windows 远程测试指南

本指南介绍如何通过远程方式在 Windows 机器上运行 Lumice GUI 测试，使用 watcher 工作流
在物理桌面会话中执行测试（真实显示器、VSync、DWM 合成器）。

## 为什么需要远程测试？

涉及显示交互的 GUI 测试（VSync、DWM 合成、窗口可见性）无法通过无头 SSH 会话可靠测试。
watcher 方式在物理交互会话中运行测试二进制文件，同时允许通过 SSH 远程触发和收集结果。

## 前置条件

1. **Windows 机器**，物理用户已登录（交互式桌面会话）
2. **SSH 访问**同一台机器（可以是不同账户，如 `builder`）
3. **共享目录** `C:\lumice-test`，物理用户和 SSH 账户均可读写
4. CI 产物已下载到本地（或本地构建）

### 一次性设置

在 Windows 机器上：

```powershell
# 创建共享目录
mkdir C:\lumice-test
icacls C:\lumice-test /grant Everyone:(OI)(CI)F
```

## 工作流

### 1. 启动 Watcher（物理会话）

在 Windows 物理桌面上打开 PowerShell 运行：

```powershell
powershell -ExecutionPolicy Bypass -File C:\path\to\win_test_watcher.ps1
```

watcher 监控 `C:\lumice-test` 中的触发文件，并在交互会话中执行指定命令。

### 2. 远程运行测试

在开发机（macOS/Linux）上：

```bash
# 下载 CI 产物
gh run download <RUN_ID> --name LumiceGUITests-windows-msvc --dir /tmp/ci-win

# 运行 VSync 下的性能测试（真实显示器）
./scripts/win_remote_test.sh /tmp/ci-win/bin/LumiceGUITests.exe \
  --filter perf_test --vsync --log-level verbose

# 运行 --perf-bench 吞吐量测量
./scripts/win_remote_test.sh /tmp/ci-win/bin/LumiceGUI.exe --perf-bench
```

### 3. 收集结果

脚本自动完成：
- 将 stdout/stderr 拷贝为 `win_stdout.txt` / `win_stderr.txt`
- 打印 stderr 最后 30 行（包含 `[PERF]` 输出）
- 显示退出码和耗时

## 脚本说明

### `scripts/win_remote_test.sh`

远程触发脚本。将二进制文件拷贝到 Windows 共享目录，写入触发文件，等待完成，收集结果。

环境变量：
- `WIN_SSH_HOST` — SSH 主机（默认：`win-builder`）
- `WIN_REMOTE_DIR` — 共享目录（默认：`C:/lumice-test`）

### `scripts/win_test_watcher.ps1`

PowerShell watcher，在 Windows 物理会话中运行。监控触发文件，执行命令并捕获 stdout/stderr。

参数：
- `-WatchDir` — 监控目录（默认：`C:\lumice-test`）

## 故障排查

- **300 秒超时**：确认 `win_test_watcher.ps1` 在物理会话中运行
- **完整测试套件 ACCESS_VIOLATION**：使用 `--filter perf_test` 仅运行性能测试
  （非性能 GUI 测试远程执行存在已知问题）
- **共享目录权限不足**：重新执行 `icacls` 授权命令

## 另见

- [性能测试指南](performance-testing_zh.md) — 完整的性能测试概述
