// Experiment: ~152KB of padding functions to shift code layout in .text section.
// The test binary's SimulateOneWavelength is at 0x180870, GUI's is at 0x15ac70,
// a difference of ~152KB. This padding aims to shift GUI's layout to match.
//
// If throughput changes significantly, it confirms the 3x Windows perf gap is
// caused by cache-unfriendly code placement from the MinGW linker.

// Each function: ~32 bytes of machine code (loop + volatile write)
// 5000 functions × 32 bytes ≈ 160KB of .text padding

#ifdef _MSC_VER
#define NOINLINE __declspec(noinline)
#else
#define NOINLINE __attribute__((noinline))
#endif

static volatile int g_padding_sink;

#define PAD(n)                     \
  NOINLINE static void pad_##n() { \
    volatile int x = 0;            \
    for (int i = 0; i < (n); ++i)  \
      x += i;                      \
    g_padding_sink = x;            \
  }

#define P10(b) PAD(b##0) PAD(b##1) PAD(b##2) PAD(b##3) PAD(b##4) PAD(b##5) PAD(b##6) PAD(b##7) PAD(b##8) PAD(b##9)
#define P100(b) P10(b##0) P10(b##1) P10(b##2) P10(b##3) P10(b##4) P10(b##5) P10(b##6) P10(b##7) P10(b##8) P10(b##9)
#define P1000(b) \
  P100(b##0) P100(b##1) P100(b##2) P100(b##3) P100(b##4) P100(b##5) P100(b##6) P100(b##7) P100(b##8) P100(b##9)

P1000(1)
P1000(2)
P1000(3)
P1000(4)
P1000(5)

// Reference all functions so they survive LTO/gc-sections
using PadFn = void (*)();
extern "C" {
NOINLINE PadFn g_pad_fns[] = {
#define REF(n) &pad_##n,
#define R10(b) REF(b##0) REF(b##1) REF(b##2) REF(b##3) REF(b##4) REF(b##5) REF(b##6) REF(b##7) REF(b##8) REF(b##9)
#define R100(b) R10(b##0) R10(b##1) R10(b##2) R10(b##3) R10(b##4) R10(b##5) R10(b##6) R10(b##7) R10(b##8) R10(b##9)
#define R1000(b) \
  R100(b##0) R100(b##1) R100(b##2) R100(b##3) R100(b##4) R100(b##5) R100(b##6) R100(b##7) R100(b##8) R100(b##9)
  R1000(1) R1000(2) R1000(3) R1000(4) R1000(5)
};
}
