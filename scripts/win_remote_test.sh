#!/usr/bin/env bash
# win_remote_test.sh — Run a GUI test on Windows with real display/VSync.
#
# Requires:
#   1. Physical user logged in on Windows, running win_test_watcher.ps1
#   2. SSH access to the same machine (builder account)
#   3. Shared directory C:\lumice-test accessible by both accounts
#
# Usage:
#   ./scripts/win_remote_test.sh <local-binary> [args...]
#
# Examples:
#   # Run perf test with VSync
#   ./scripts/win_remote_test.sh ./LumiceGUITests.exe --filter perf_test --vsync --log-level verbose
#
#   # Run full GUI test
#   ./scripts/win_remote_test.sh ./LumiceGUITests.exe
#
# The script will:
#   1. Copy the binary to the Windows shared directory
#   2. Write a trigger file for the watcher
#   3. Wait for completion
#   4. Print stdout/stderr and copy log files back

set -euo pipefail

# --- Configuration ---
SSH_HOST="${WIN_SSH_HOST:-win-builder}"
REMOTE_DIR="${WIN_REMOTE_DIR:-C:/lumice-test}"
POLL_INTERVAL=2    # seconds between done-checks
TIMEOUT=300        # seconds before giving up

# --- Parse arguments ---
if [ $# -lt 1 ]; then
    echo "Usage: $0 <local-binary> [args...]"
    echo ""
    echo "Environment variables:"
    echo "  WIN_SSH_HOST    SSH host (default: win-builder)"
    echo "  WIN_REMOTE_DIR  Shared directory (default: C:/lumice-test)"
    exit 1
fi

LOCAL_BINARY="$1"
shift
EXTRA_ARGS="$*"

BINARY_NAME=$(basename "$LOCAL_BINARY")
REMOTE_BINARY="${REMOTE_DIR}/${BINARY_NAME}"

# --- Helper ---
# SSH runs PowerShell on Windows; all remote commands must be valid PowerShell.
ssh_cmd() { ssh "$SSH_HOST" "$@"; }

echo "=== win_remote_test ==="
echo "Host:   $SSH_HOST"
echo "Binary: $LOCAL_BINARY -> $REMOTE_BINARY"
echo "Args:   $EXTRA_ARGS"
echo ""

# --- Step 1: Copy binary ---
echo "[1/4] Copying binary..."
scp -q "$LOCAL_BINARY" "${SSH_HOST}:${REMOTE_DIR}/"

# --- Step 2: Write trigger ---
echo "[2/4] Sending trigger..."
# Clean previous results (PowerShell; ignore errors if files don't exist)
ssh_cmd "Remove-Item '$REMOTE_DIR/done.txt','$REMOTE_DIR/trigger.txt' -ErrorAction SilentlyContinue; exit 0"

# Write trigger: the watcher reads this and executes the command
TRIGGER_CMD="${REMOTE_BINARY} ${EXTRA_ARGS}"
ssh_cmd "'${TRIGGER_CMD}' | Out-File '${REMOTE_DIR}/trigger.txt' -Encoding utf8"

echo "       Trigger sent. Waiting for watcher to pick up..."

# --- Step 3: Wait for completion ---
echo "[3/4] Waiting for completion (timeout ${TIMEOUT}s)..."
ELAPSED=0
while [ $ELAPSED -lt $TIMEOUT ]; do
    if ssh_cmd "Test-Path '${REMOTE_DIR}/done.txt'" 2>/dev/null | grep -q "True"; then
        break
    fi
    sleep $POLL_INTERVAL
    ELAPSED=$((ELAPSED + POLL_INTERVAL))
    # Progress indicator every 10s
    if [ $((ELAPSED % 10)) -eq 0 ]; then
        printf "       %ds...\n" "$ELAPSED"
    fi
done

if [ $ELAPSED -ge $TIMEOUT ]; then
    echo ""
    echo "ERROR: Timeout after ${TIMEOUT}s. Is win_test_watcher.ps1 running?"
    exit 1
fi

echo "       Completed."

# --- Step 4: Collect results ---
echo "[4/4] Collecting results..."
echo ""

# Print done.txt (exit code, elapsed time)
echo "--- done.txt ---"
ssh_cmd "Get-Content '${REMOTE_DIR}/done.txt'" 2>/dev/null || true
echo ""

# Print stderr (contains [PERF] output)
echo "--- stderr (last 30 lines) ---"
ssh_cmd "Get-Content '${REMOTE_DIR}/stderr.txt' -Tail 30" 2>/dev/null || true
echo ""

# Copy stderr/stdout locally
LOCAL_OUT_DIR="${LOCAL_OUT_DIR:-.}"
echo "Copying output files to ${LOCAL_OUT_DIR}/"
scp -q "${SSH_HOST}:${REMOTE_DIR}/stdout.txt" "${LOCAL_OUT_DIR}/win_stdout.txt" 2>/dev/null || true
scp -q "${SSH_HOST}:${REMOTE_DIR}/stderr.txt" "${LOCAL_OUT_DIR}/win_stderr.txt" 2>/dev/null || true

echo ""
echo "=== Done ==="
