// Linux ISA launcher for the release tarball.
//
// The Linux x64 release ships every entry point twice — `<name>.baseline` (x86-64-v1, runs
// on any x86_64 CPU) and `<name>.x86-64-v4` (AVX-512, ~2x faster on CPUs that have it) — and
// installs this program under the user-facing name (`Lumice`, `LumiceGUI`). It does exactly
// one thing: pick the sidecar this CPU can run and `execv` it. Process replacement, not
// fork+wait: the real binary takes over this PID, so exit codes, signals (Ctrl-C on a long
// trace) and the environment pass through with no forwarding code to get wrong.
//
// Two decisions worth knowing before changing anything here:
//
// - Detection is the CPUID instruction (`__builtin_cpu_supports`), never `/proc/cpuinfo`.
//   `__builtin_cpu_supports("x86-64-v4")` checks the whole psABI level — every v2/v3/v4
//   feature bit plus the XGETBV check that the OS actually saves ZMM state — which is what
//   a `-march=x86-64-v4` binary assumes; a hand-rolled AVX-512F test would miss the OS half.
//   It is also the only detection that can be *tested* on a machine that has AVX-512:
//   `qemu-x86_64 -cpu <pre-AVX-512 model>` emulates CPUID per its `-cpu` argument but hands
//   the guest the host's real `/proc/cpuinfo`, so a text-file detector would pick the v4
//   sidecar under QEMU and SIGILL, and the negative control this launcher must pass would be
//   defeated by the detector itself.
//
// - This file is plain C, is compiled with no -march flag and links nothing from the engine
//   (no lumice_obj, no ILOG_*). It must run on the very CPUs it exists to protect, so it
//   cannot itself be an ISA-gated binary; and because it has no logger, its two error
//   messages go to stderr directly — the one place in src/ besides main.cpp and fatal.hpp
//   where that is the design rather than a bypass (see the Logging rule in AGENTS.md).
//
// Bootstrap argument: a single `--isa=baseline` / `--isa=x86-64-v4` token anywhere in argv
// overrides detection and is removed before the sidecar sees its arguments. It is the
// negative-control hook and the user's escape hatch; it is deliberately not an environment
// variable (doc/env-var-policy.md).

#if !defined(__linux__) || !defined(__x86_64__)
#error "isa_launcher.c is Linux x86_64 only: it reads /proc/self/exe and dispatches on x86 CPUID"
#endif

// readlink() and PATH_MAX are POSIX, not ISO C: ask for them explicitly rather than depend on
// the build defaulting to gnu11 (a strict -std=c11 compile fails without this line).
#define _GNU_SOURCE

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static const char kIsaPrefix[] = "--isa=";
static const char kBaseline[] = "baseline";
static const char kV4[] = "x86-64-v4";

static void Usage(const char* self) {
  fprintf(stderr,
          "%s: ISA launcher. Runs %s.%s or %s.%s from the same directory, chosen by CPUID.\n"
          "  %s%s | %s%s   force one variant (the token is consumed, not forwarded)\n",
          self, self, kBaseline, self, kV4, kIsaPrefix, kBaseline, kIsaPrefix, kV4);
}

static const char* DetectVariant(void) {
  __builtin_cpu_init();
  return __builtin_cpu_supports("x86-64-v4") ? kV4 : kBaseline;
}

int main(int argc, char** argv) {
  // Own path first: dirname decides where the sidecars are, basename decides their stem
  // (the same launcher binary is installed as both `Lumice` and `LumiceGUI`).
  char self_path[PATH_MAX];
  ssize_t n = readlink("/proc/self/exe", self_path, sizeof(self_path) - 1);
  if (n < 0) {
    fprintf(stderr, "%s: cannot resolve /proc/self/exe: %s\n", argv[0], strerror(errno));
    return 1;
  }
  self_path[n] = '\0';
  char* slash = strrchr(self_path, '/');
  const char* self_name = slash ? slash + 1 : self_path;
  size_t dir_len = slash ? (size_t)(slash - self_path) + 1 : 0;  // keeps the trailing '/'

  // Consume the override token; every other argument is forwarded verbatim and in order.
  const char* forced = NULL;
  char** fwd = (char**)calloc((size_t)argc + 1, sizeof(char*));
  if (!fwd) {
    fprintf(stderr, "%s: out of memory\n", self_name);
    return 1;
  }
  int fwd_n = 0;
  fwd[fwd_n++] = (char*)self_name;  // argv[0] stays the user-facing name, not the sidecar's
  for (int i = 1; i < argc; ++i) {
    if (strncmp(argv[i], kIsaPrefix, sizeof(kIsaPrefix) - 1) != 0) {
      fwd[fwd_n++] = argv[i];
      continue;
    }
    const char* value = argv[i] + sizeof(kIsaPrefix) - 1;
    if (forced) {
      fprintf(stderr, "%s: %s given more than once\n", self_name, kIsaPrefix);
      Usage(self_name);
      return 1;
    }
    if (strcmp(value, kBaseline) == 0) {
      forced = kBaseline;
    } else if (strcmp(value, kV4) == 0) {
      forced = kV4;
    } else {
      fprintf(stderr, "%s: unknown ISA variant \"%s\"\n", self_name, value);
      Usage(self_name);
      return 1;
    }
  }
  fwd[fwd_n] = NULL;

  const char* variant = forced ? forced : DetectVariant();

  char target[PATH_MAX];
  int len = snprintf(target, sizeof(target), "%.*s%s.%s", (int)dir_len, self_path, self_name, variant);
  if (len < 0 || (size_t)len >= sizeof(target)) {
    fprintf(stderr, "%s: sidecar path too long\n", self_name);
    return 1;
  }

  execv(target, fwd);
  // execv only returns on failure. 127 is the shell's "command not found" code, and a
  // missing or non-executable sidecar (a partially unpacked tarball) is the same failure.
  fprintf(stderr, "%s: cannot exec %s: %s\n", self_name, target, strerror(errno));
  return 127;
}
