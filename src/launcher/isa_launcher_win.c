// Windows ISA launcher for the release zip.
//
// The Windows x64 release ships every entry point twice — `<name>.baseline.exe` (x86-64-v1,
// MSVC cl.exe, runs on any x86_64 CPU) and `<name>.x86-64-v3.exe` (AVX2+FMA, clang-cl, ~2.3x
// faster on CPUs that have it) — and installs this program under the user-facing name
// (`Lumice.exe`, `LumiceGUI.exe`). It does exactly one thing: pick the sidecar this CPU can
// run and hand the process over to it. It is the Windows twin of src/launcher/isa_launcher.c;
// the same structure, with every step that was a POSIX primitive there replaced by the
// Win32/CRT one, and the same override token. Read that file's header for the decisions the
// two share (CPUID-instruction detection, no engine code, plain C, stderr on purpose).
//
// Three things are different here, and each is a measured fact, not a preference:
//
// - There is no exec() on Windows. The CRT's `_execv` is not process replacement: measured on
//   the Windows reference box, the parent exits 0 the moment the child starts (a child that
//   returned 3 read as ERRORLEVEL 0 in the calling shell), and `_execv` joins argv with spaces
//   and no quoting (an argument "a b" arrived as two). So this launcher spawns the sidecar with
//   CreateProcess from a command line it quotes itself (MSVC argv rules, below), waits, and
//   returns the child's exit code as its own — the exit-code and argv pass-through that execv
//   gives Linux for free are the two things a launcher must not get wrong.
//
// - Ctrl-C goes to every process attached to the console, so the sidecar sees it directly
//   with no forwarding. The launcher installs a handler that swallows its own copy — a handler
//   *function*, never `SetConsoleCtrlHandler(NULL, TRUE)`, whose ignore flag is inherited by
//   the child and would make the engine itself deaf to Ctrl-C — and keeps waiting, so the
//   shell sees the child's real termination status rather than the launcher dying first.
//
// - The launcher detaches from the console once the child is running. LumiceGUI is a
//   console-subsystem binary that calls FreeConsole() at startup so the window Explorer opens
//   for a double-click closes again; with the launcher still attached that window would stay
//   open for the whole GUI session. Detaching costs the CLI nothing: the child inherited the
//   stdout/stderr handles at creation, and the launcher writes nothing after this point.
//
// Detection covers the whole x86-64-v3 psABI level (every v2/v3 feature bit plus the XGETBV
// check that the OS saves YMM state), not just the four bits an AVX2 binary visibly needs:
// `-march=x86-64-v3` lets the compiler use all of them, and the Linux launcher's
// `__builtin_cpu_supports("x86-64-v4")` checks its whole level the same way. Order matters in
// one place: OSXSAVE (CPUID.1:ECX[27]) must be confirmed before `_xgetbv` is executed — with
// CR4.OSXSAVE clear (a machine booted with `bcdedit /set xsavedisable 1`) XGETBV is #UD, and
// the launcher exists precisely to not crash on the CPU/OS states it is protecting against.

#if !defined(_WIN32) || !defined(_M_X64)
#error "isa_launcher_win.c is Windows x64 only: it reads GetModuleFileName and dispatches on x86 CPUID"
#endif

#include <intrin.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <windows.h>

static const char kIsaPrefix[] = "--isa=";
static const char kBaseline[] = "baseline";
static const char kV3[] = "x86-64-v3";
static const char kExeSuffix[] = ".exe";

static void Usage(const char* self) {
  fprintf(stderr,
          "%s: ISA launcher. Runs %s.%s.exe or %s.%s.exe from the same directory, chosen by CPUID.\n"
          "  %s%s | %s%s   force one variant (the token is consumed, not forwarded)\n",
          self, self, kBaseline, self, kV3, kIsaPrefix, kBaseline, kIsaPrefix, kV3);
}

// One CPUID feature bit: which leaf/subleaf to query, which register (0=EAX..3=EDX) and bit.
typedef struct {
  int leaf;
  int subleaf;
  int reg;
  int bit;
} CpuidBit;

static int HasBit(const CpuidBit* b) {
  int info[4];
  __cpuidex(info, b->leaf, b->subleaf);
  return (info[b->reg] >> b->bit) & 1;
}

static const char* DetectVariant(void) {
  // Leaf availability first: CPUID.0:EAX is the highest basic leaf, and leaf 7 (AVX2/BMI) is
  // only meaningful if it is reported; the extended leaf 0x80000001 likewise.
  int info[4];
  __cpuid(info, 0);
  if ((unsigned)info[0] < 7u) {
    return kBaseline;
  }
  __cpuid(info, (int)0x80000000);
  if ((unsigned)info[0] < 0x80000001u) {
    return kBaseline;
  }

  // x86-64-v2: CMPXCHG16B, LAHF-SAHF, POPCNT, SSE3, SSE4.1, SSE4.2, SSSE3.
  // x86-64-v3: AVX, AVX2, BMI1, BMI2, F16C, FMA, LZCNT, MOVBE, plus OS-enabled YMM state.
  static const CpuidBit kV3Bits[] = {
    { 1, 0, 2, 0 },                // SSE3
    { 1, 0, 2, 9 },                // SSSE3
    { 1, 0, 2, 12 },               // FMA
    { 1, 0, 2, 13 },               // CMPXCHG16B
    { 1, 0, 2, 19 },               // SSE4.1
    { 1, 0, 2, 20 },               // SSE4.2
    { 1, 0, 2, 22 },               // MOVBE
    { 1, 0, 2, 23 },               // POPCNT
    { 1, 0, 2, 27 },               // OSXSAVE — must be checked before _xgetbv below (XGETBV is #UD otherwise)
    { 1, 0, 2, 28 },               // AVX
    { 1, 0, 2, 29 },               // F16C
    { 7, 0, 1, 3 },                // BMI1
    { 7, 0, 1, 5 },                // AVX2
    { 7, 0, 1, 8 },                // BMI2
    { (int)0x80000001, 0, 2, 0 },  // LAHF-SAHF
    { (int)0x80000001, 0, 2, 5 },  // LZCNT (ABM)
  };
  for (size_t i = 0; i < sizeof(kV3Bits) / sizeof(kV3Bits[0]); ++i) {
    if (!HasBit(&kV3Bits[i])) {
      return kBaseline;
    }
  }
  // Every bit above is set, OSXSAVE included, so XGETBV is legal here. XCR0[1] = SSE state,
  // XCR0[2] = AVX (YMM upper) state: the OS has to save both across context switches, or a
  // -march=x86-64-v3 binary corrupts its own registers.
  unsigned long long xcr0 = _xgetbv(0);
  if ((xcr0 & 0x6ull) != 0x6ull) {
    return kBaseline;
  }
  return kV3;
}

// Append one argument to a command line the way the MSVC CRT (and CommandLineToArgvW) will
// read it back: quote when it contains whitespace, a quote, or is empty; inside quotes,
// double every backslash run that precedes a quote (or the closing quote) and escape the
// quote itself. Returns the number of characters written, or -1 if `cap` is too small.
static int AppendQuoted(char* out, size_t cap, size_t at, const char* arg) {
  size_t n = at;
  int need_quotes = (*arg == '\0') || (strpbrk(arg, " \t\"") != NULL);
  if (!need_quotes) {
    size_t len = strlen(arg);
    if (n + len + 1 > cap)
      return -1;
    memcpy(out + n, arg, len);
    return (int)(n + len);
  }
  if (n + 1 > cap)
    return -1;
  out[n++] = '"';
  for (const char* p = arg; *p; ++p) {
    size_t backslashes = 0;
    while (*p == '\\') {
      ++backslashes;
      ++p;
    }
    if (*p == '\0') {
      // Trailing backslashes precede the closing quote: double them.
      if (n + 2 * backslashes + 1 > cap)
        return -1;
      memset(out + n, '\\', 2 * backslashes);
      n += 2 * backslashes;
      break;
    }
    if (*p == '"') {
      if (n + 2 * backslashes + 2 + 1 > cap)
        return -1;
      memset(out + n, '\\', 2 * backslashes + 1);
      n += 2 * backslashes + 1;
      out[n++] = '"';
    } else {
      if (n + backslashes + 1 + 1 > cap)
        return -1;
      memset(out + n, '\\', backslashes);
      n += backslashes;
      out[n++] = *p;
    }
  }
  out[n++] = '"';
  return (int)n;
}

static BOOL WINAPI SwallowCtrl(DWORD type) {
  (void)type;
  // Handled: the sidecar, attached to the same console, receives the same event and acts on
  // it; this process just keeps waiting for it to exit.
  return TRUE;
}

static void PrintLastError(const char* self, const char* what, const char* target) {
  DWORD err = GetLastError();
  char msg[512];
  DWORD n =
      FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, err, 0, msg, sizeof(msg), NULL);
  if (n == 0) {
    snprintf(msg, sizeof(msg), "error %lu", (unsigned long)err);
  }
  fprintf(stderr, "%s: %s %s: %s", self, what, target, msg);
  if (n == 0 || msg[n - 1] != '\n')
    fputc('\n', stderr);
}

int main(int argc, char** argv) {
  // Own path first: the directory decides where the sidecars are, the stem (file name minus
  // `.exe`) decides their names — the same launcher binary is installed as both `Lumice.exe`
  // and `LumiceGUI.exe`.
  char self_path[MAX_PATH];
  DWORD n = GetModuleFileNameA(NULL, self_path, (DWORD)sizeof(self_path));
  if (n == 0 || n >= sizeof(self_path)) {
    fprintf(stderr, "%s: cannot resolve own executable path\n", argv[0]);
    return 1;
  }
  char* slash = strrchr(self_path, '\\');
  const char* self_name = slash ? slash + 1 : self_path;
  size_t dir_len = slash ? (size_t)(slash - self_path) + 1 : 0;  // keeps the trailing '\'
  char stem[MAX_PATH];
  snprintf(stem, sizeof(stem), "%s", self_name);
  size_t stem_len = strlen(stem);
  if (stem_len > sizeof(kExeSuffix) - 1 && _stricmp(stem + stem_len - (sizeof(kExeSuffix) - 1), kExeSuffix) == 0) {
    stem[stem_len - (sizeof(kExeSuffix) - 1)] = '\0';
  }

  // Consume the override token; every other argument is forwarded verbatim and in order.
  const char* forced = NULL;
  size_t cmd_cap = 32768;  // CreateProcess's own command-line limit
  char* cmd = (char*)malloc(cmd_cap);
  if (!cmd) {
    fprintf(stderr, "%s: out of memory\n", self_name);
    return 1;
  }
  int cmd_len = AppendQuoted(cmd, cmd_cap, 0, self_name);  // argv[0] stays the user-facing name
  for (int i = 1; i < argc && cmd_len >= 0; ++i) {
    if (strncmp(argv[i], kIsaPrefix, sizeof(kIsaPrefix) - 1) != 0) {
      if ((size_t)cmd_len + 1 >= cmd_cap) {
        cmd_len = -1;
        break;
      }
      cmd[cmd_len++] = ' ';
      cmd_len = AppendQuoted(cmd, cmd_cap, (size_t)cmd_len, argv[i]);
      continue;
    }
    const char* value = argv[i] + sizeof(kIsaPrefix) - 1;
    if (forced) {
      fprintf(stderr, "%s: %s given more than once\n", self_name, kIsaPrefix);
      Usage(stem);
      return 1;
    }
    if (strcmp(value, kBaseline) == 0) {
      forced = kBaseline;
    } else if (strcmp(value, kV3) == 0) {
      forced = kV3;
    } else {
      fprintf(stderr, "%s: unknown ISA variant \"%s\"\n", self_name, value);
      Usage(stem);
      return 1;
    }
  }
  if (cmd_len < 0) {
    fprintf(stderr, "%s: command line too long\n", self_name);
    return 1;
  }
  cmd[cmd_len] = '\0';

  const char* variant = forced ? forced : DetectVariant();

  char target[MAX_PATH];
  int len = snprintf(target, sizeof(target), "%.*s%s.%s%s", (int)dir_len, self_path, stem, variant, kExeSuffix);
  if (len < 0 || (size_t)len >= sizeof(target)) {
    fprintf(stderr, "%s: sidecar path too long\n", self_name);
    return 1;
  }

  SetConsoleCtrlHandler(SwallowCtrl, TRUE);

  STARTUPINFOA si;
  PROCESS_INFORMATION pi;
  memset(&si, 0, sizeof(si));
  si.cb = sizeof(si);
  memset(&pi, 0, sizeof(pi));
  // lpApplicationName is the resolved sidecar path, so no PATH or current-directory search
  // takes place; lpCommandLine is what the child parses into its argv. Handles are inherited
  // so `> file` and pipes set up by the caller reach the child unchanged.
  if (!CreateProcessA(target, cmd, NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi)) {
    // 127 is the shell's "command not found" code, and a missing sidecar (a partially
    // unpacked zip) is the same failure — the same code the Linux launcher uses.
    PrintLastError(self_name, "cannot start", target);
    return 127;
  }
  CloseHandle(pi.hThread);
  FreeConsole();

  WaitForSingleObject(pi.hProcess, INFINITE);
  DWORD code = 1;
  if (!GetExitCodeProcess(pi.hProcess, &code)) {
    code = 1;
  }
  CloseHandle(pi.hProcess);
  return (int)code;
}
