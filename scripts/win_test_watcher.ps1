# win_test_watcher.ps1 — Run on the Windows PHYSICAL session (interactive desktop).
#
# Watches a shared directory for trigger files. When a trigger is detected,
# executes the specified command IN THIS SESSION (with real display/VSync/DWM),
# captures output, and signals completion.
#
# Usage (on Windows physical session):
#   powershell -ExecutionPolicy Bypass -File C:\path\to\win_test_watcher.ps1
#
# Shared directory: C:\lumice-test (must be readable/writable by both physical
# user and SSH builder account). Create once with:
#   mkdir C:\lumice-test
#   icacls C:\lumice-test /grant Everyone:(OI)(CI)F

param(
    [string]$WatchDir = "C:\lumice-test"
)

$triggerFile = Join-Path $WatchDir "trigger.txt"
$doneFile    = Join-Path $WatchDir "done.txt"
$stdoutFile  = Join-Path $WatchDir "stdout.txt"
$stderrFile  = Join-Path $WatchDir "stderr.txt"

# Ensure watch directory exists
if (-not (Test-Path $WatchDir)) {
    New-Item -ItemType Directory -Path $WatchDir -Force | Out-Null
    Write-Host "[watcher] Created $WatchDir"
}

Write-Host "[watcher] Watching $WatchDir for trigger.txt ..."
Write-Host "[watcher] Press Ctrl+C to stop."
Write-Host ""

while ($true) {
    if (Test-Path $triggerFile) {
        # Read command from trigger file
        $lines = Get-Content $triggerFile
        $cmd = ($lines | Where-Object { $_ -and $_.Trim() }) -join " "
        Remove-Item $triggerFile -Force

        # Clean up previous results
        foreach ($f in @($doneFile, $stdoutFile, $stderrFile)) {
            if (Test-Path $f) { Remove-Item $f -Force }
        }

        Write-Host "[watcher] $(Get-Date -Format 'HH:mm:ss') Trigger received: $cmd"

        # Split into executable and arguments
        # Supports: "path\to\exe.exe --arg1 --arg2" or just "exe.exe"
        $parts = $cmd -split ' ', 2
        $exe = $parts[0]
        $procArgs = if ($parts.Length -gt 1) { $parts[1] } else { "" }

        # Execute in this (interactive) session
        $sw = [System.Diagnostics.Stopwatch]::StartNew()
        try {
            $proc = Start-Process -FilePath $exe -ArgumentList $procArgs `
                -RedirectStandardOutput $stdoutFile `
                -RedirectStandardError $stderrFile `
                -PassThru -Wait -NoNewWindow
            $exitCode = $proc.ExitCode
        } catch {
            $exitCode = -1
            $_.Exception.Message | Out-File $stderrFile -Append
        }
        $sw.Stop()

        # Write completion signal
        @"
exit_code=$exitCode
elapsed_sec=$([math]::Round($sw.Elapsed.TotalSeconds, 1))
command=$cmd
"@ | Out-File $doneFile -Encoding utf8

        $elapsedSec = [math]::Round($sw.Elapsed.TotalSeconds, 1)
        Write-Host "[watcher] $(Get-Date -Format 'HH:mm:ss') Done (exit=$exitCode, ${elapsedSec}s)"
        Write-Host ""
    }
    Start-Sleep -Seconds 1
}
