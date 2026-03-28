# Windows Remote Testing Guide

This guide explains how to run Lumice GUI tests on a Windows machine remotely,
using a watcher-based workflow that executes tests in the physical desktop session
(with real display, VSync, and DWM compositor).

## Why Remote Testing?

GUI tests that involve display interaction (VSync, DWM composition, window visibility)
cannot be reliably tested via headless SSH sessions. The watcher approach runs the test
binary in the physical interactive session while allowing remote triggering and result
collection via SSH.

## Prerequisites

1. **Windows machine** with a physical user logged in (interactive desktop session)
2. **SSH access** to the same machine (can be a different account, e.g., `builder`)
3. **Shared directory** `C:\lumice-test` accessible by both the physical user and SSH account
4. CI artifacts downloaded locally (or built locally)

### One-time Setup

On the Windows machine:

```powershell
# Create shared directory
mkdir C:\lumice-test
icacls C:\lumice-test /grant Everyone:(OI)(CI)F
```

## Workflow

### 1. Start the Watcher (Physical Session)

On the Windows physical desktop, open PowerShell and run:

```powershell
powershell -ExecutionPolicy Bypass -File C:\path\to\win_test_watcher.ps1
```

The watcher monitors `C:\lumice-test` for trigger files and executes the specified
command in the interactive session.

### 2. Run Tests Remotely

From your development machine (macOS/Linux):

```bash
# Download CI artifact
gh run download <RUN_ID> --name LumiceGUITests-windows --dir /tmp/ci-win

# Run perf test with VSync (real display)
./scripts/win_remote_test.sh /tmp/ci-win/bin/LumiceGUITests.exe \
  --filter perf_test --vsync --log-level verbose

# Run --perf-bench for throughput measurement
./scripts/win_remote_test.sh /tmp/ci-win/bin/LumiceGUI.exe --perf-bench
```

### 3. Collect Results

The script automatically:
- Copies stdout/stderr to `win_stdout.txt` / `win_stderr.txt`
- Prints the last 30 lines of stderr (contains `[PERF]` output)
- Shows exit code and elapsed time

## Scripts

### `scripts/win_remote_test.sh`

Remote trigger script. Copies the binary to the Windows shared directory, writes a
trigger file, waits for completion, and collects results.

Environment variables:
- `WIN_SSH_HOST` — SSH host (default: `win-builder`)
- `WIN_REMOTE_DIR` — Shared directory (default: `C:/lumice-test`)

### `scripts/win_test_watcher.ps1`

PowerShell watcher that runs in the Windows physical session. Monitors for trigger
files and executes commands with stdout/stderr capture.

Parameters:
- `-WatchDir` — Watch directory (default: `C:\lumice-test`)

## Troubleshooting

- **Timeout after 300s**: Ensure `win_test_watcher.ps1` is running in the physical session
- **ACCESS_VIOLATION on full test suite**: Use `--filter perf_test` to run only perf tests
  (known issue with non-perf GUI tests via remote execution)
- **Permission denied on shared directory**: Re-run `icacls` grant command

## See Also

- [Performance Testing Guide](performance-testing.md) — comprehensive perf testing overview
