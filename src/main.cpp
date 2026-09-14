#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>
// clang-format off
#ifdef _WIN32
#include <windows.h>   // Must come before shellapi.h (defines EXTERN_C etc.)
#include <shellapi.h>  // CommandLineToArgvW
#endif
// clang-format on

#include "lumice.h"
#include "util/cpu_info.hpp"
#include "util/logger.hpp"
#include "util/raypath_analysis_display.hpp"
#include "util/result_frame.hpp"
#include "util/sky_direction.hpp"

#ifdef _WIN32
#define STBIW_WINDOWS_UTF8
#endif
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace {

constexpr int kDefaultJpegQuality = 95;
// How often the render loop materializes a partial result to disk/stdout, NOT how
// often it checks whether the run has finished. The two used to be the same number,
// which put a hard 1s floor under every CLI render: the loop slept a whole interval
// before its first completion check, so a 20k-ray run that finished in ~20ms still
// cost 1.02s of wall time. Completion is now polled at kFinePollInterval; this
// constant only paces the expensive half (SaveRenderResults / SaveCompositeResults /
// PrintStats all go through LUMICE_AcquireResultFrame, i.e. a full render).
constexpr auto kSaveInterval = std::chrono::seconds(1);

// Owning wrapper for the short-lived LUMICE_Scene handles the CLI builds from JSON.
// LUMICE_CommitScene reads the scene as const and keeps no reference to it, so each handle is
// a local that must be destroyed on every exit path — including the commit-failure returns.
struct SceneDeleter {
  void operator()(LUMICE_Scene* scene) const { LUMICE_SceneDestroy(scene); }
};
using ScenePtr = std::unique_ptr<LUMICE_Scene, SceneDeleter>;

// Result frames get the same treatment, but from a shared header — util/result_frame.hpp,
// which the GUI's two call sites use as well. See there for why the wrapper exists at all.
using lumice::ResultFramePtr;

// Warn (once, on config load) if the last multi-scattering layer has prob > 0.
// core semantics: the last layer's prob-fail rays would have "continued to the
// next layer", but there is no next layer, so they are silently discarded (see
// simulator.cpp:1134). This is a footgun in hand-written configs; not a bug.
//
// Threshold is a strict `prob > 0.0`, intentionally different from the GUI's
// IsProbZero(epsilon) helper in gui_state.hpp (kProbZeroEps=0.005 half-step).
// The GUI epsilon absorbs slider-drag/text-input float noise — CLI values come
// straight from hand-written JSON where a "0" means literal zero. Keeping the
// CLI check strict avoids false-negatives on tiny hand-written probs (e.g.
// 1e-4) that would still leak rays. This intentional asymmetry does NOT
// violate "GUI ≡ CLI" (that rule is about identical rendering per file value,
// not about UI-noise-absorption epsilon).
//
// JSON keys mirror scattering entries as parsed in config_manager.cpp:94
// (`j_s.at("prob")`) — keep aligned if the schema evolves.
void WarnIfLastScatteringLayerProbNonzero(const nlohmann::json& j_cfg) {
  try {
    const auto& j_scene = j_cfg.at("scene");
    if (!j_scene.contains("scattering")) {
      return;
    }
    const auto& j_scat = j_scene.at("scattering");
    if (!j_scat.is_array() || j_scat.empty()) {
      return;
    }
    const auto& j_last = j_scat.back();
    if (!j_last.contains("prob")) {
      return;
    }
    double prob = j_last.at("prob").get<double>();
    if (prob > 0.0) {
      LOG_WARNING(
          "Last scattering layer has prob={:.4f} > 0: that fraction of filter-pass rays "
          "will be discarded (no next layer to receive them). Set the last layer's prob to 0 "
          "unless this is intentional.",
          prob);
    }
  } catch (const nlohmann::json::exception&) {
    // Malformed / unexpected shape: silently skip. The core parser will
    // surface any real schema errors when it consumes the config.
  }
}

void WarnIfLastScatteringLayerProbNonzero(const std::filesystem::path& config_path) {
  std::ifstream f(config_path);
  if (!f.is_open()) {
    return;  // let LUMICE_SceneFromJsonFile report the real "cannot open" error.
  }
  try {
    nlohmann::json j;
    f >> j;
    WarnIfLastScatteringLayerProbNonzero(j);
  } catch (const nlohmann::json::exception&) {
    // As above: let the core parser surface real errors.
  }
}

// task-metal-green-pixel-floor: emit a stdout line reporting per-color-class
// signal presence (0/1 per class) after the final composite fetch. Consumed by
// test/e2e-correctness/test_raypath_color.py to assert every class captured at
// least one non-zero pixel — a per-class lane-wiring smoke test that stays
// cross-backend stable (the retired dominant-argmax pixel-count floor was
// CPU-calibrated and unstable on Metal). No-op unless the config declares
// `raypath_color`.
void PrintColorClassSignal(LUMICE_Server* server, const nlohmann::json& j_cfg) {
  int class_count = 0;
  try {
    if (!j_cfg.contains("raypath_color")) {
      return;
    }
    const auto& j_rc = j_cfg.at("raypath_color");
    // Accept both wire forms core RaypathColorConfig::from_json accepts:
    //   - bare array [ ... ]  (default-mode short form)
    //   - object {"mode": ..., "classes": [ ... ]}  (non-default / explicit)
    const nlohmann::json* j_classes = nullptr;
    if (j_rc.is_array()) {
      j_classes = &j_rc;
    } else if (j_rc.is_object() && j_rc.contains("classes") && j_rc.at("classes").is_array()) {
      j_classes = &j_rc.at("classes");
    }
    if (j_classes == nullptr || j_classes->empty()) {
      return;
    }
    class_count = static_cast<int>(j_classes->size());
  } catch (const nlohmann::json::exception&) {
    return;
  }
  std::vector<int> flags(static_cast<std::size_t>(class_count), 0);
  auto rc = LUMICE_GetColorClassSignal(server, flags.data(), class_count);
  if (rc != LUMICE_OK) {
    std::cerr << "Warning: LUMICE_GetColorClassSignal returned " << rc << "; skipping ColorClassSignal line\n";
    return;
  }
  std::cout << "ColorClassSignal:";
  for (int f : flags) {
    std::cout << " " << f;
  }
  std::cout << "\n";
}

void PrintColorClassSignal(LUMICE_Server* server, const std::filesystem::path& config_path) {
  std::ifstream f(config_path);
  if (!f.is_open()) {
    return;
  }
  try {
    nlohmann::json j;
    f >> j;
    PrintColorClassSignal(server, j);
  } catch (const nlohmann::json::exception&) {
    // Malformed config: no-op, do not fail the CLI.
  }
}
// Fine poll granularity (was 100ms). At 100ms the IDLE-detection quantization
// alone could add up to a full poll interval to wall time; for a fast backend
// whose run completes in ~0.2s that deflated rays_per_sec by >30%. 5ms caps the
// trailing quantization at a few ms while staying sleep-based (negligible CPU
// steal from trace workers). See task-fix-throughput-bench-honesty.
//
// Shared by BOTH polling loops in this file — the benchmark pass and the render
// loop in main() — because the reason for 5ms is the same on both sides: the
// completion check itself is cheap (a mutex-guarded state read), and any coarser
// interval quantizes the run's wall time up to a multiple of itself. Keeping one
// constant is deliberate: two copies of the same "why 5ms" invite a one-sided
// retune that silently reintroduces the quantization on the other side.
// If the two loops ever need genuinely different granularities for genuinely
// different reasons (e.g. the render loop wanting a coarser interval to save
// power on long runs), split this back into two named constants — do NOT branch
// on the caller inside one shared constant.
constexpr auto kFinePollInterval = std::chrono::milliseconds(5);
constexpr int kBenchmarkSingleRays = 2'000'000;
// GPU-route warm-up pass ray count: sized to be the smallest value that still
// reliably touches every one-time init path (CUDA context / Metal PSO compile /
// first device dispatch). Context/PSO init cost is ray-count-independent — it
// fires on the first GPU call, not on ray N — so the choice is bounded below
// only by "enough rays for the pipeline to actually reach a GPU dispatch under
// the normal drain semantics". 100k is well above the ~64k-batch drain
// granularity of every current backend and keeps the added warm-up wall-time
// short — every GPU bench pass pays it, including the ones the e2e suites run
// under a wall-clock timeout, so the warm-up must not be a measurable share of
// a pass.
constexpr int kBenchmarkGpuWarmupRays = 100'000;
// Drain-count-driven measurement (task-gpu-bench-drain-aligned-rate): when the
// bench config asks for scene.ray_num="infinite", RunBenchmarkPass measures the
// window from drain #1 (warmup-end anchor) to drain #(N+1), then stops the
// server. N = 10 yields ~CUDA 168M / Metal 21M rays per bench pass — a
// predictable, cheap, backend-agnostic steady-state measurement.
// N was chosen empirically (N-sweep on Mac Metal, N∈{5,10,20,40}): CoV does NOT
// drop monotonically with N — beyond ~N=10 thermal drift over the longer
// measurement REGROWS variance (N=40 hit 23.6% CoV), so a bigger window is not
// "more stable". N=10 is the robust middle: on locked-clock desktops (CUDA
// dev49 / win-builder — the authoritative throughput machines) it is a larger,
// steadier window than N=5 at negligible cost and no thermal regrowth. On a Mac
// laptop, Metal throughput is environment-dominated (thermal / GPU boost swing
// it ~2x run-to-run, CoV 8-28%) and NO N stabilizes it — Mac Metal is treated
// as approximate (phase-1), not canonical. See doc/performance-testing.md.
constexpr int kBenchmarkDrainWindows = 10;

// --- Subcommands -------------------------------------------------------------
//
// argv[1] names the subcommand: `render`, `benchmark` or `analyze`. Anything else in
// that slot (an option, or nothing at all) means the implicit `render`, so the
// `Lumice -f config.json ...` form every README / quickstart / user script uses
// keeps working verbatim. Each subcommand accepts ONLY its own option set and
// reports everything else as an unknown option — there is no "accepted but
// ignored" flag anywhere, which is what keeps the "which flag means what in
// which mode" matrix from growing a dimension per mode. A subcommand is one
// Parse*/Run* pair below plus one branch in main(); nothing shared has to learn
// about it.

constexpr std::string_view kSubcommandRender = "render";
constexpr std::string_view kSubcommandBenchmark = "benchmark";
constexpr std::string_view kSubcommandAnalyze = "analyze";

// The options every subcommand shares, as help-text fragments defined once so the
// wording cannot drift between subcommands. Split in three because each subcommand
// interleaves its own options between them in the order a reader expects (-f first,
// the log/help switches last).
constexpr const char* kHelpConfigOption = "  -f <file>          Specify the configuration file (required)\n";
constexpr const char* kHelpBackendOption =
    "  --backend <name>   Trace backend: auto, cpu, metal, or cuda (default: auto).\n"
    "                     'auto' and 'cpu' both select the CPU route today; 'metal'\n"
    "                     falls back to CPU if unavailable. The LUMICE_TRACE_BACKEND\n"
    "                     env var, if set, still overrides this (debug/CI only).\n";
constexpr const char* kHelpLogAndHelpOptions =
    "  -v                 Verbose output (trace level logging)\n"
    "  -d                 Debug output (debug level logging)\n"
    "  -h, --help         Show this help message and exit\n";

// --workers is shared by `render` and `analyze` (both size a CPU worker pool) and rejected
// by `benchmark` (its worker counts are the methodology); the text is one fragment so the two
// pages that show it say the same thing.
constexpr const char* kHelpWorkersOption =
    "  --workers <N>      Number of CPU simulation worker threads (default: automatic —\n"
    "                     one per physical core, capped at a ceiling above which no\n"
    "                     machine measured ran faster; an explicit N is never capped).\n"
    "                     Machine-dependent, so it is a command-line switch rather than\n"
    "                     a config-file field: a config travels between machines and a\n"
    "                     worker count should not travel with it. Ignored on a GPU route\n"
    "                     (single engine).\n";

void PrintRenderOptions() {
  std::cout << kHelpConfigOption
            << "  -o <dir>           Output directory for rendered images (default: current directory)\n"
            << "  --format <fmt>     Output image format: jpg or png (default: jpg)\n"
            << "  --quality <1-100>  JPEG quality (default: 95, ignored for PNG)\n"
            << kHelpBackendOption << kHelpWorkersOption << kHelpLogAndHelpOptions;
}

void PrintRenderExamples(const char* prog_name) {
  std::cout << "  " << prog_name << " -f config.json\n"
            << "  " << prog_name << " -f config.json -o /tmp/output\n"
            << "  " << prog_name << " -f config.json --format png\n"
            << "  " << prog_name << " -f config.json --quality 80\n"
            << "  " << prog_name << " -f config.json --backend metal\n"
            << "  " << prog_name << " -f config.json --workers 4\n"
            << "  " << prog_name << " -f config.json -v\n";
}

void PrintRenderUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " [render] -f <config_file> [options]\n"
            << "\n"
            << "Simulate ice halos and write the rendered images. `render` is the default\n"
            << "subcommand, so `" << prog_name << " -f config.json` is the same command.\n"
            << "\n"
            << "Options:\n";
  PrintRenderOptions();
  std::cout << "\n"
            << "Examples:\n";
  PrintRenderExamples(prog_name);
}

void PrintBenchmarkUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " benchmark -f <config_file> [options]\n"
            << "\n"
            << "Run a throughput benchmark and print [BENCHMARK] JSON. The legacy CPU route\n"
            << "runs a dual pass (single-worker + multi-worker → per-core and parallel-\n"
            << "efficiency data); a GPU route is single-engine, so it runs one steady pass\n"
            << "only (single/multi would not be parallel). The worker counts are part of the\n"
            << "measurement methodology, which is why there is no --workers here; nothing is\n"
            << "written to disk, which is why there is no -o.\n"
            << "\n"
            << "Options:\n"
            << kHelpConfigOption << kHelpBackendOption << kHelpLogAndHelpOptions << "\n"
            << "Examples:\n"
            << "  " << prog_name << " benchmark -f examples/bench_config.json\n"
            << "  " << prog_name << " benchmark -f examples/bench_config.json --backend metal\n";
}

void PrintAnalyzeUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " analyze -f <config_file> [options]\n"
            << "\n"
            << "Trace the scene and list the raypath chains that delivered energy into a region\n"
            << "of the sky, most energetic first, as CSV — the same file the GUI's Raypath\n"
            << "Analysis window exports. The config is the scene; the question asked of it is\n"
            << "given by the options below and never read from the config. The analysis always\n"
            << "traces on the CPU (the chain record exists on that route only).\n"
            << "\n"
            << "Output: the CSV goes to stdout, or to --csv <path> instead (never both). A `#`\n"
            << "head names the region, the symmetry, the ray total and the export time; then\n"
            << "one row per chain: Raypath, Energy (% of the total), Cumulative %, +/- (the\n"
            << "row's 1/sqrt(count) relative noise, in %). Progress goes to stderr, one line per\n"
            << "second. Ctrl-C ends the run early and still writes the result accumulated so far\n"
            << "(exit 0) — which is how a scene whose ray_num is \"infinite\" is meant to be run.\n"
            << "With --csv the file is rewritten atomically every second, so it is complete at\n"
            << "any moment it is read.\n"
            << "\n"
            << "Options:\n"
            << kHelpConfigOption
            << "  --roi <region>     Which rays count: sky (every outgoing ray; default), frame (the\n"
            << "                     rays that land inside one of the config's render[] frames), or\n"
            << "                     cone (the rays within --radius of --center).\n"
            << "  --render-id <id>   frame only: the render[] entry whose lens / view / visible /\n"
            << "                     front / resolution define the frame (default: the first entry).\n"
            << "  --center <alt>,<az>\n"
            << "                     cone only (required): the cone's centre as the altitude and\n"
            << "                     azimuth, in degrees, of the sky point — azimuth measured as the\n"
            << "                     sun's is, so the sun sits at --center <sun_altitude>,0.\n"
            << "  --radius <deg>     cone only (required): the cone's angular radius in degrees.\n"
            << "  --symmetry <spec>  Merge chains that are the same path up to crystal symmetry when\n"
            << "                     listing: any combination of P, B, D (case-insensitive) or\n"
            << "                     `none` (default: PBD). Changes the grouping, never the totals.\n"
            << "  --rays <N>         This run's ray budget, total across wavelengths; N may carry a\n"
            << "                     K, M or G suffix (e.g. 20M). Default: the scene's own ray_num,\n"
            << "                     including \"infinite\".\n"
            << "  --seed <N>         Fix the simulation's random seed (a positive integer) so two\n"
            << "                     runs of one question are the same run; this also sizes the pool\n"
            << "                     to one worker (a seeded run is single-threaded by contract).\n"
            << "                     Default: random.\n"
            << "  --csv <path>       Write the CSV to this file instead of stdout.\n"
            << kHelpBackendOption << kHelpWorkersOption << kHelpLogAndHelpOptions << "\n"
            << "Examples:\n"
            << "  " << prog_name << " analyze -f config.json\n"
            << "  " << prog_name << " analyze -f config.json --roi cone --center 43,0 --radius 2\n"
            << "  " << prog_name << " analyze -f config.json --roi frame --render-id 1 --csv frame.csv\n"
            << "  " << prog_name << " analyze -f config.json --symmetry none --rays 5M --seed 7\n";
}

// Top-level `-h` (no subcommand named): the subcommand overview followed by the
// implicit subcommand's full option list, so the help a user reaches from the
// form they already know is complete on its own.
void PrintTopLevelUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " [render] -f <config_file> [options]\n"
            << "       " << prog_name << " benchmark -f <config_file> [options]\n"
            << "       " << prog_name << " analyze -f <config_file> [options]\n"
            << "       " << prog_name << " <subcommand> -h\n"
            << "\n"
            << "Lumice — simulate ice halos by tracing rays through ice crystals.\n"
            << "\n"
            << "Subcommands:\n"
            << "  render             Simulate and write halo images. This is the default when\n"
            << "                     no subcommand is given: `" << prog_name << " -f ...` is a render.\n"
            << "  benchmark          Run a throughput benchmark and print [BENCHMARK] JSON\n"
            << "                     (`" << prog_name << " benchmark -h` for its options)\n"
            << "  analyze            List the raypath chains that light a region of the sky, as CSV\n"
            << "                     (`" << prog_name << " analyze -h` for its options)\n"
            << "\n"
            << "Options for render (the default subcommand):\n";
  PrintRenderOptions();
  std::cout << "\n"
            << "Examples:\n";
  PrintRenderExamples(prog_name);
  std::cout << "  " << prog_name << " benchmark -f examples/bench_config.json\n"
            << "  " << prog_name << " analyze -f config.json --roi cone --center 43,0 --radius 2\n";
}

// Maps a --backend argument to a LUMICE_BACKEND_* id. Returns -1 for an
// unrecognized name. "auto" resolves to the library default (CPU); the env-var
// LUMICE_TRACE_BACKEND still overrides this at runtime (debug/CI escape hatch,
// see doc/env-var-policy.md).
int ParseBackend(std::string_view name) {
  if (name == "auto" || name == "cpu") {
    return LUMICE_BACKEND_CPU;
  }
  if (name == "metal") {
    return LUMICE_BACKEND_METAL;
  }
  if (name == "cuda") {
    return LUMICE_BACKEND_CUDA;
  }
  return -1;
}

std::filesystem::path FormatImagePath(const std::filesystem::path& output_dir, int renderer_id, std::string_view format,
                                      std::string_view suffix = "") {
  std::ostringstream oss;
  oss << "img_" << std::setfill('0') << std::setw(2) << renderer_id << suffix << "." << format;
  return output_dir / oss.str();
}

void SaveRenderResults(LUMICE_Server* server, const std::filesystem::path& output_dir, std::string_view image_format,
                       int jpeg_quality) {
  LUMICE_ResultFrame* raw_frame = nullptr;
  if (LUMICE_AcquireResultFrame(server, &raw_frame) != LUMICE_OK) {
    return;
  }
  ResultFramePtr frame(raw_frame);
  LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1]{};
  if (LUMICE_FrameGetRender(frame.get(), renders, LUMICE_MAX_RENDER_RESULTS) != LUMICE_OK) {
    return;
  }
  for (int i = 0; renders[i].img_buffer != nullptr; i++) {
    auto filepath = FormatImagePath(output_dir, renders[i].renderer_id, image_format);
    auto filepath_u8 = filepath.u8string();
    int ok = 0;
    if (image_format == "png") {
      int stride = renders[i].img_width * 3;
      ok = stbi_write_png(filepath_u8.c_str(), renders[i].img_width, renders[i].img_height, 3, renders[i].img_buffer,
                          stride);
    } else {
      ok = stbi_write_jpg(filepath_u8.c_str(), renders[i].img_width, renders[i].img_height, 3, renders[i].img_buffer,
                          jpeg_quality);
    }
    if (ok) {
      std::cout << "Saved: " << filepath_u8 << " (" << renders[i].img_width << "x" << renders[i].img_height << ")\n";
    } else {
      std::cerr << "Error: failed to write " << filepath << "\n";
    }
  }
}

// task-336.4: additive per-raypath composite output. When `raypath_color` is
// configured, LUMICE_FrameGetComposite yields one colored image per renderer
// (written as img_XX_components.<fmt>); otherwise it returns an empty set
// (out[0] sentinel) and nothing is written — the mono img_XX.<fmt> path above is
// byte-for-byte unchanged (zero-regression).
void SaveCompositeResults(LUMICE_Server* server, const std::filesystem::path& output_dir, std::string_view image_format,
                          int jpeg_quality) {
  LUMICE_ResultFrame* raw_frame = nullptr;
  if (LUMICE_AcquireResultFrame(server, &raw_frame) != LUMICE_OK) {
    return;
  }
  ResultFramePtr frame(raw_frame);
  LUMICE_RenderResult composites[LUMICE_MAX_RENDER_RESULTS + 1]{};
  if (LUMICE_FrameGetComposite(frame.get(), composites, LUMICE_MAX_RENDER_RESULTS) != LUMICE_OK) {
    return;
  }
  for (int i = 0; composites[i].img_buffer != nullptr; i++) {
    auto filepath = FormatImagePath(output_dir, composites[i].renderer_id, image_format, "_components");
    auto filepath_u8 = filepath.u8string();
    int ok = 0;
    if (image_format == "png") {
      int stride = composites[i].img_width * 3;
      ok = stbi_write_png(filepath_u8.c_str(), composites[i].img_width, composites[i].img_height, 3,
                          composites[i].img_buffer, stride);
    } else {
      ok = stbi_write_jpg(filepath_u8.c_str(), composites[i].img_width, composites[i].img_height, 3,
                          composites[i].img_buffer, jpeg_quality);
    }
    if (ok) {
      std::cout << "Saved: " << filepath_u8 << " (" << composites[i].img_width << "x" << composites[i].img_height
                << ")\n";
    } else {
      std::cerr << "Error: failed to write " << filepath << "\n";
    }
  }
}

void PrintStats(LUMICE_Server* server) {
  LUMICE_ResultFrame* raw_frame = nullptr;
  if (LUMICE_AcquireResultFrame(server, &raw_frame) != LUMICE_OK) {
    return;
  }
  ResultFramePtr frame(raw_frame);
  LUMICE_StatsResult stats{};
  if (LUMICE_FrameGetStats(frame.get(), &stats) != LUMICE_OK) {
    return;
  }
  if (stats.sim_ray_num != 0) {
    std::cout << "Stats: sim_rays=" << stats.sim_ray_num << ", crystals=" << stats.crystal_num
              << ", orientations=" << stats.orientation_num << "\n";
  }
}

// `silent`: when true, suppress the final `[BENCHMARK]` JSON line on stdout and
// the `wall_fallback` warning on stderr. Used by the GPU-route warm-up pass
// (see RunBenchmark's GPU-route branch): its purpose is to absorb one-time GPU
// context/PSO lazy-init before the real steady pass, so its own rate is
// meaningless and would only mislead if reported. Every other observable side
// effect (server create/commit/poll-to-IDLE/destroy, and the stderr commit-fail
// path at line ~305) is kept identical to a normal pass — do not carve out a
// parallel slim path (a05).
void RunBenchmarkPass(const std::string& config_str, int num_workers, const char* mode, int cores,
                      LUMICE_LogLevel log_level, int preferred_backend, bool silent = false) {
  LUMICE_ServerConfig server_config{};
  server_config.num_workers = num_workers;
  server_config.preferred_backend = preferred_backend;
  auto* server = LUMICE_CreateServerEx(&server_config);
  LUMICE_SetLogLevel(server, log_level);

  // JSON -> handle -> commit. Parse and commit are separate entry points now, so the two
  // failures are reported separately instead of being flattened into one "failed to commit".
  LUMICE_Scene* raw_scene = nullptr;
  if (LUMICE_SceneFromJson(config_str.c_str(), &raw_scene) != LUMICE_OK) {
    std::cerr << "Error: failed to parse config for " << mode << " benchmark pass\n";
    LUMICE_DestroyServer(server);
    return;
  }
  ScenePtr scene(raw_scene);
  if (LUMICE_CommitScene(server, scene.get(), /*out_reused=*/nullptr) != LUMICE_OK) {
    std::cerr << "Error: failed to commit config for " << mode << " benchmark pass\n";
    LUMICE_DestroyServer(server);
    return;
  }

  // Throughput honesty (task-fix-throughput-bench-honesty): rays_per_sec must
  // measure the engine's sustained trace rate, NOT (rays / whole-run-wall). The
  // whole run includes one-time setup (server alloc + scene gen + first-dispatch
  // latency) during which sim_ray_num stays 0; folding that into the denominator
  // systematically deflated fast backends. We therefore start the throughput
  // clock at the first poll where tracing has actually produced rays
  // (sim_ray_num > 0) and measure the steady window from there to IDLE,
  // excluding the first observed chunk (its rays were produced before we could
  // sample them). `wall_sec`/`setup_sec`/`active_sec` are reported alongside for
  // transparency; existing keys (mode/workers/cores/rays/rays_per_sec) are kept.
  //
  // Drain-count-driven path (task-gpu-bench-drain-aligned-rate): the setup-honest
  // steady window above still under-reports fast backends when the config's
  // finite ray_num yields fewer than ~10 drains — sim_ray_num is drain-quantized
  // (each drain = kDefaultXyzDrainBatches * dispatch_size rays; simulator.cpp
  // xyz_win_ triggers on 64 batches). CUDA default dispatch (262144) = 16.8M
  // rays/drain, so a 20M-ray config only sees ~1.19 drains and lumps most trace
  // work into "setup". Fix: when the config's scene.ray_num is "infinite"
  // (config_manager.cpp:120 -> kInfSize), measure the window from the 1st
  // observed drain (warmup-end anchor) to the (N+1)-th drain, then StopServer.
  // Endpoints are already sampled before Stop, so the number is unaffected by
  // Stop semantics (task-262 lost-wakeup already fixed).
  bool drain_count_mode = false;
  try {
    auto j_cfg = nlohmann::json::parse(config_str);
    const auto& j_ray_num = j_cfg.at("scene").at("ray_num");
    // Sentinel value "infinite" mirrors config_manager.cpp:120 (which maps it to
    // kInfSize on the core side). This is a second independent check because
    // RunBenchmarkPass only receives the raw config_str, not an already-parsed
    // SceneConfig — the two sites must stay in sync. If a future refactor gives
    // this function access to the resolved config object, collapse to one site.
    drain_count_mode = j_ray_num.is_string() && j_ray_num.get<std::string>() == "infinite";
  } catch (const nlohmann::json::exception&) {
    // Malformed / unexpected shape: fall back to finite-path measurement.
  }

  auto t_run_start = std::chrono::steady_clock::now();
  auto t_active_start = t_run_start;
  LUMICE_RayCount rays_at_active_start = 0;
  bool active_started = false;

  // Drain-count-driven state (only meaningful when drain_count_mode == true).
  LUMICE_RayCount prev_rays = 0;
  int n_drains_observed = 0;
  LUMICE_RayCount rays_per_drain_estimate = 0;
  bool first_drain_captured = false;
  LUMICE_RayCount rays_at_first_drain = 0;
  auto t_first_drain = t_run_start;
  bool window_closed = false;
  LUMICE_RayCount rays_at_final_drain = 0;
  auto t_final_drain = t_run_start;
  int n_drains_in_window = 0;

  while (true) {
    std::this_thread::sleep_for(kFinePollInterval);
    LUMICE_ServerState state{};
    if (LUMICE_QueryServerState(server, &state) != LUMICE_OK) {
      continue;
    }
    // Read sim_ray_num via the cheap O(1) live counter, NOT by acquiring a result
    // frame — the latter triggers a full DoSnapshot +
    // RenderConsumer sRGB (powf/pixel) on EVERY poll. For the drain-count path
    // (many polls) that render tax dominated wall-time, starved drain-window
    // closure, and (on CUDA) let the unbounded session run long enough to trip
    // the 32-bit device PCG ray-index cap -> silent legacy fallback + hang.
    // This is now a gate, not a comment: the `no-render-in-benchmark-poll` rule
    // in scripts/check_policies.py rejects an acquire anywhere in this function.
    // The same principle paces the render loop in main() — it polls completion
    // at this same interval and materializes only once per kSaveInterval — so
    // "cheap counter to decide, expensive frame only when publishing" is one
    // rule with two call sites, not a benchmark-only precaution.
    LUMICE_RayCount cur_rays = 0;
    LUMICE_GetSimRayCount(server, &cur_rays);
    auto now = std::chrono::steady_clock::now();

    // Mark end-of-setup the first time tracing has produced rays.
    if (!active_started && cur_rays > 0) {
      active_started = true;
      t_active_start = now;
      rays_at_active_start = cur_rays;
    }

    // Drain-count-driven bookkeeping: detect drain events by sim_ray_num jumps.
    // First observed jump defines rays_per_drain_estimate (= that jump's size,
    // since sim_ray_num starts at 0 and moves in whole-drain increments); later
    // jumps may span >1 drain if poll granularity is coarser than one drain, so
    // reverse-infer the drain count from the ray increment (plan §3 D2 / Step 1
    // test point). On reaching (N+1) drains we lock the endpoint, StopServer,
    // and let the loop drain to IDLE for a clean shutdown.
    if (drain_count_mode && !window_closed && cur_rays > prev_rays) {
      LUMICE_RayCount delta = cur_rays - prev_rays;
      int new_drains = 0;
      if (rays_per_drain_estimate == 0) {
        rays_per_drain_estimate = delta;
        new_drains = 1;
      } else {
        double ratio = static_cast<double>(delta) / static_cast<double>(rays_per_drain_estimate);
        // Invariant (do NOT "fix" the floor(1) away): rays_per_drain_estimate is
        // seeded from the first jump, which is >= one true drain, so ratio <= the
        // true drain count of this delta and max(1,...) can only UNDER-count
        // drains — the window therefore only ever grows WIDER (>= N drains), never
        // closes early with < N. rays_per_sec stays honest regardless because both
        // endpoints are raw, drain-aligned sim_ray_num reads. Removing the floor
        // would let a 0-round introduce a real early-close bug.
        new_drains = std::max(1, static_cast<int>(std::llround(ratio)));
      }
      n_drains_observed += new_drains;
      if (!first_drain_captured) {
        first_drain_captured = true;
        rays_at_first_drain = cur_rays;
        t_first_drain = now;
      } else if (n_drains_observed >= kBenchmarkDrainWindows + 1) {
        rays_at_final_drain = cur_rays;
        t_final_drain = now;
        // Window = drain #1 -> drain #(N+1) skips warmup drain, holds N drains
        // (or M >= N if a single poll observed a super-drain jump; still
        // integer-drain-aligned so the rate stays honest).
        n_drains_in_window = n_drains_observed - 1;
        window_closed = true;
        LUMICE_StopServer(server);
      }
      prev_rays = cur_rays;
    }

    if (state == LUMICE_SERVER_IDLE && cur_rays > 0) {
      auto t_end = now;
      LUMICE_RayCount r_end = cur_rays;
      double wall_sec = std::chrono::duration<double>(t_end - t_run_start).count();
      double setup_sec = std::chrono::duration<double>(t_active_start - t_run_start).count();
      double active_sec = std::chrono::duration<double>(t_end - t_active_start).count();

      // rate_basis ladder:
      //   drain_count_mode true  -> `drain_aligned` (window closed) or
      //                             `too_few_drains` (infinite path exited early
      //                             w/o observing N+1 drains — unexpected).
      //   drain_count_mode false -> `steady` / `active_short` / `wall_fallback`
      //                             (task-fix-throughput-bench-honesty ladder).
      // The two branches are independent enums; downstream (docs, gate) parses
      // them by drain_count_mode / config context, not by string equality.
      double rays_per_sec = 0.0;
      const char* rate_basis = "wall_fallback";
      double window_sec = 0.0;
      LUMICE_RayCount window_rays = 0;
      // Shared by `active_short` and `wall_fallback`: both degrade to a wall-clock
      // lower bound (see the `active_short` branch comment below for why). Computed
      // once so the two branches cannot silently diverge if only one is edited later.
      const double wall_bounded_rate = wall_sec > 0 ? static_cast<double>(r_end) / wall_sec : 0.0;
      if (drain_count_mode) {
        if (window_closed && t_final_drain > t_first_drain && rays_at_final_drain > rays_at_first_drain) {
          window_sec = std::chrono::duration<double>(t_final_drain - t_first_drain).count();
          window_rays = rays_at_final_drain - rays_at_first_drain;
          rays_per_sec = static_cast<double>(window_rays) / window_sec;
          rate_basis = "drain_aligned";
        } else {
          rays_per_sec = wall_sec > 0 ? static_cast<double>(r_end) / wall_sec : 0.0;
          rate_basis = "too_few_drains";
        }
      } else if (active_sec > 1e-4 && r_end > rays_at_active_start) {
        rays_per_sec = static_cast<double>(r_end - rays_at_active_start) / active_sec;
        rate_basis = "steady";
      } else if (active_started && active_sec > 1e-4) {
        // Degenerate window: sim_ray_num was observed exactly ONCE (r_end ==
        // rays_at_active_start), so there is no interior sample and `active_sec` measures
        // IDLE-detection latency, not trace duration. On a GPU backend this is not
        // hypothetical: sim_ray_num advances in whole drain quanta
        // (kDefaultXyzDrainBatches * dispatch_size = 64 * 32768 = 2,097,152 rays with the
        // Metal default), so a config whose entire ray_num is below one quantum publishes
        // its counter exactly once, at the end — whether that publish shares a poll with
        // the IDLE transition or precedes it by one is a race, which is why the same
        // binary/config/machine alternated between `wall_fallback` and a measured 14-29x
        // phantom rate at ~4% of runs when this branch divided by `active_sec` itself. Use
        // the wall-clock denominator instead — it is a real duration that provably contains
        // the whole trace, so the number is a conservative LOWER bound on the true rate
        // rather than an unbounded upward fantasy. Same formula as `wall_fallback`, but
        // the basis label is deliberately kept distinct: the two say different things
        // (`active_short` = exactly one counter publish observed; `wall_fallback` = no
        // usable active window at all), and merging them would change the rate_basis
        // value set for no gain. See doc/performance-testing.md §C rule 5 for the full
        // derivation (quantum value, 240-run measurement, short-window bias decay curve).
        rays_per_sec = wall_bounded_rate;
        rate_basis = "active_short";
      } else {
        rays_per_sec = wall_bounded_rate;
        rate_basis = "wall_fallback";
      }

      nlohmann::json result;
      result["mode"] = mode;
      result["workers"] = num_workers;
      result["cores"] = cores;
      result["rays"] = r_end;
      result["wall_sec"] = std::round(wall_sec * 1000.0) / 1000.0;
      result["setup_sec"] = std::round(setup_sec * 1000.0) / 1000.0;
      result["active_sec"] = std::round(active_sec * 1000.0) / 1000.0;
      result["rays_per_sec"] = std::round(rays_per_sec * 10.0) / 10.0;
      result["rate_basis"] = rate_basis;
      // ISA tier this binary was actually compiled for, so a recorded [BENCHMARK] line
      // answers "which build was this measured on?" without anyone having to still have
      // the configure log. "native" means -march=native was compiled in (local default);
      // "x86-64-v4" is the AVX-512 variant the Linux release ships beside the baseline;
      // "x86-64-v3" is the AVX2 variant the Windows release ships beside it (clang-cl);
      // "baseline" is what CI tests and every other release build ships, and is the only
      // tier comparable across platforms — real MSVC cl.exe has no equivalent flag, so a
      // Windows-vs-other A/B taken on "native" numbers is not measuring what it looks like
      // it is measuring. The string is CMakeLists.txt's LUMICE_ISA_LEVEL_STR, resolved in
      // lumice_apply_isa_march() by the same condition that gates the -march flag itself,
      // so it reads "baseline" whenever no flag was applied (any non-Release config
      // included); see doc/performance-testing.md. cl.exe never defines it.
#if defined(LUMICE_ISA_LEVEL_STR)
      result["isa"] = LUMICE_ISA_LEVEL_STR;
#else
      result["isa"] = "baseline";
#endif
      if (drain_count_mode) {
        result["n_drains_in_window"] = n_drains_in_window;
        result["window_sec"] = std::round(window_sec * 1000.0) / 1000.0;
        result["window_rays"] = window_rays;
      }
      if (!silent) {
        // `wall_fallback` means active_sec was too small to measure a steady
        // trace rate, so `rays_per_sec = r_end / wall_sec` was used — that
        // denominator includes one-time setup (server alloc + scene gen + first
        // GPU dispatch triggering CUDA/Metal context/PSO lazy init). Treat such
        // numbers as unusable for perf comparison; the GPU-route warm-up pass
        // (see RunBenchmark's GPU-route branch) exists precisely to move that setup
        // cost out of the measured pass. Warning suppressed under `silent`
        // (i.e. the warm-up pass itself) — the warm-up rate is not reported,
        // so warning about its basis would only mislead.
        if (std::string_view(rate_basis) == "wall_fallback") {
          std::cerr << "Warning: [BENCHMARK] mode=" << mode
                    << " rate_basis=wall_fallback — rays_per_sec includes one-time setup/context-init "
                    << "and is not a steady trace rate; do not use for perf comparison.\n";
        } else if (std::string_view(rate_basis) == "active_short") {
          // Mechanism: see the `active_short` branch comment above (where rate_basis is
          // assigned) and doc/performance-testing.md §C rule 5. Not re-derived here so the
          // two sites can't drift apart.
          std::cerr << "Warning: [BENCHMARK] mode=" << mode
                    << " rate_basis=active_short — the run produced only a single "
                    << "sim_ray_num observation (ray_num below one drain quantum), so no "
                    << "steady window exists; rays_per_sec falls back to the wall-clock "
                    << "denominator and includes setup — a lower bound, not a steady trace "
                    << "rate; do not use for perf comparison.\n";
        }
        std::cout << "[BENCHMARK] " << result.dump() << "\n";
      }
      break;
    }
  }

  LUMICE_DestroyServer(server);
}

// --- Option parsing ------------------------------------------------------------

// What every subcommand needs to know before it can build a server.
struct SharedOptions {
  std::filesystem::path config_filename;
  int preferred_backend = LUMICE_BACKEND_CPU;
  LUMICE_LogLevel log_level = LUMICE_LOG_INFO;
};

struct RenderOptions {
  SharedOptions shared;
  std::filesystem::path output_dir = ".";
  std::string image_format = "jpg";
  int jpeg_quality = kDefaultJpegQuality;
  // 0 = "not specified" — the same value LUMICE_ServerConfig::num_workers already uses to mean
  // "let the server pick" (one per physical core, capped), so no separate was-it-set flag is
  // needed.
  int cli_workers = 0;
};

struct BenchmarkOptions {
  SharedOptions shared;
};

struct AnalyzeOptions {
  SharedOptions shared;
  int roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
  // The three per-ROI options, kept as "was it given" so the parser can reject one the chosen
  // ROI has no use for (an option that is accepted and ignored is the thing this CLI's option
  // sets are designed not to have).
  std::optional<float> center_alt_deg;
  std::optional<float> center_az_deg;
  std::optional<float> radius_deg;
  std::optional<int> render_id;
  std::uint8_t symmetry_bits = LUMICE_RAYPATH_SYMMETRY_P | LUMICE_RAYPATH_SYMMETRY_B | LUMICE_RAYPATH_SYMMETRY_D;
  // nullopt = the scene's own budget (LUMICE_RAYPATH_RAY_BUDGET_SCENE_DEFAULT).
  std::optional<LUMICE_RayCount> ray_num;
  unsigned int sim_seed = 0;       // 0 = random, as LUMICE_ServerConfig::sim_seed spells it
  std::filesystem::path csv_path;  // empty = stdout
  int cli_workers = 0;             // 0 = automatic, as RenderOptions::cli_workers
};

// Outcome of offering argv[i] to the option set every subcommand shares.
enum class SharedStep {
  kNotShared,  // not one of the shared options — the subcommand's own parser decides
  kConsumed,   // handled; `i` now indexes the last token consumed
  kHelp,       // -h / --help — the caller prints its own usage and exits 0
  kError,      // diagnosed on stderr — the caller prints its own usage and exits 1
};

// Parses the one option at argv[i] if it is shared (-f, --backend, -v, -d, -h/--help), advancing
// `i` past any value it takes. The retired `--benchmark` flag is diagnosed here too, so both
// subcommands point at the migration path instead of calling it an unknown option.
SharedStep ParseSharedOption(int argc, char** argv, int& i, SharedOptions& out) {
  std::string_view arg = argv[i];
  if (arg == "-f") {
    if (++i >= argc) {
      std::cerr << "Error: -f requires an argument\n\n";
      return SharedStep::kError;
    }
    out.config_filename = argv[i];
    return SharedStep::kConsumed;
  }
  if (arg == "--backend") {
    if (++i >= argc) {
      std::cerr << "Error: --backend requires an argument\n\n";
      return SharedStep::kError;
    }
    out.preferred_backend = ParseBackend(argv[i]);
    if (out.preferred_backend < 0) {
      std::cerr << "Error: --backend must be 'auto', 'cpu', 'metal', or 'cuda', got '" << argv[i] << "'\n\n";
      return SharedStep::kError;
    }
    return SharedStep::kConsumed;
  }
  if (arg == "-v") {
    out.log_level = LUMICE_LOG_VERBOSE;
    return SharedStep::kConsumed;
  }
  if (arg == "-d") {
    out.log_level = LUMICE_LOG_DEBUG;
    return SharedStep::kConsumed;
  }
  if (arg == "-h" || arg == "--help") {
    return SharedStep::kHelp;
  }
  if (arg == "--benchmark") {
    std::cerr << "Error: --benchmark has been replaced by the 'benchmark' subcommand: " << argv[0]
              << " benchmark -f <config>\n\n";
    return SharedStep::kError;
  }
  return SharedStep::kNotShared;
}

#ifdef _WIN32
// Re-parse file paths from the wide-char command line for full Unicode support.
// argv[i] on Windows uses the ANSI codepage, which loses non-ASCII characters.
// Only path arguments (-f, -o, --csv) need wide-char re-parsing; ASCII-only args
// (the subcommand token, --format, --quality, --workers, the analyze request) are
// safe as-is. `output_dir` / `csv_path` are null for a subcommand that has no -o /
// --csv: the option was already rejected by that subcommand's parser, so there is
// nothing to re-read.
void ReparseWidePathArgs(std::filesystem::path& config_filename, std::filesystem::path* output_dir,
                         std::filesystem::path* csv_path) {
  int wargc = 0;
  wchar_t** wargv = CommandLineToArgvW(GetCommandLineW(), &wargc);
  if (!wargv) {
    return;
  }
  for (int i = 1; i < wargc; i++) {
    std::wstring_view warg = wargv[i];
    if (warg == L"-f" && i + 1 < wargc) {
      config_filename = wargv[++i];
    } else if (output_dir && warg == L"-o" && i + 1 < wargc) {
      *output_dir = wargv[++i];
    } else if (csv_path && warg == L"--csv" && i + 1 < wargc) {
      *csv_path = wargv[++i];
    }
  }
  LocalFree(wargv);
}
#endif

// The checks that follow parsing for every subcommand: -f is required, and a GPU
// backend the machine cannot provide falls back to CPU with a visible notice
// rather than silently. (The core would fall back anyway; this just makes the
// substitution explicit on the CLI.) Returns false when the caller must print
// its usage and exit 1.
bool FinishSharedOptions(SharedOptions& opts) {
  if (opts.config_filename.empty()) {
    std::cerr << "Error: configuration file is required (-f <file>)\n\n";
    return false;
  }
  if (opts.preferred_backend == LUMICE_BACKEND_METAL && !LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL)) {
    std::cerr << "Warning: --backend metal requested but no Metal device is available; using CPU.\n";
    opts.preferred_backend = LUMICE_BACKEND_CPU;
  }
  if (opts.preferred_backend == LUMICE_BACKEND_CUDA && !LUMICE_IsBackendAvailable(LUMICE_BACKEND_CUDA)) {
    std::cerr << "Warning: --backend cuda requested but no eligible CUDA device is available; using CPU.\n";
    opts.preferred_backend = LUMICE_BACKEND_CPU;
  }
  return true;
}

// A strictly-typed positive integer, the discipline every numeric option here follows.
// std::stoi / stoull stop at the first non-digit WITHOUT throwing, so "3abc" would parse as 3
// and be silently accepted; the `pos == size` check is what makes trailing garbage an error
// rather than a value the user never typed. They also skip LEADING whitespace and accept a
// '+'/'-' sign, so " 3" or "+3" would pass as a bare "3" — reject anything that does not start
// with a digit up front. Returns nullopt for anything but digits-only text that fits.
std::optional<unsigned long long> ParseStrictUnsigned(std::string_view text) {
  if (text.empty() || !std::isdigit(static_cast<unsigned char>(text[0]))) {
    return std::nullopt;
  }
  const std::string owned(text);
  std::size_t parsed_len = 0;
  unsigned long long value = 0;
  try {
    value = std::stoull(owned, &parsed_len);
  } catch (const std::exception&) {
    return std::nullopt;
  }
  if (parsed_len != owned.size()) {
    return std::nullopt;
  }
  return value;
}

// A float with the same discipline (digits, an optional sign and decimal point, nothing after):
// std::stof's "consume a prefix" behaviour is the same footgun, so the full-consumption check
// applies. Leading whitespace is rejected as above. inf / nan spellings are rejected too — a
// degree value is a number the user typed, not a special value.
std::optional<float> ParseStrictFloat(std::string_view text) {
  if (text.empty()) {
    return std::nullopt;
  }
  const char first = text[0];
  if (!(std::isdigit(static_cast<unsigned char>(first)) || first == '-' || first == '+' || first == '.')) {
    return std::nullopt;
  }
  const std::string owned(text);
  std::size_t parsed_len = 0;
  float value = 0.0f;
  try {
    value = std::stof(owned, &parsed_len);
  } catch (const std::exception&) {
    return std::nullopt;
  }
  if (parsed_len != owned.size() || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

// The `--workers <N>` value at argv[i+1], for the two subcommands that size a CPU worker pool
// (`render` and `analyze`; `benchmark` never accepts it — its worker counts are the methodology,
// so it does not call this and reports the option as unknown). On success advances `i` past the
// value and writes it; on failure the diagnostic is already on stderr and the caller prints its
// usage. One function rather than a step in ParseSharedOption, so the subcommand that rejects the
// option needs no exclusion logic — it simply never offers argv[i] here.
bool TryParseWorkersOption(int argc, char** argv, int& i, int& out_workers) {
  if (++i >= argc) {
    std::cerr << "Error: --workers requires an argument\n\n";
    return false;
  }
  const std::string workers_arg = argv[i];
  const auto parsed = ParseStrictUnsigned(workers_arg);
  if (!parsed.has_value() || *parsed > static_cast<unsigned long long>(std::numeric_limits<int>::max())) {
    std::cerr << "Error: --workers requires a numeric value, got '" << workers_arg << "'\n\n";
    return false;
  }
  if (*parsed == 0) {
    std::cerr << "Error: --workers must be a positive integer, got 0\n\n";
    return false;
  }
  out_workers = static_cast<int>(*parsed);
  return true;
}

// Parses argv[first..) as the `render` option set. `print_usage` is the help this
// invocation form should show — the top-level overview when `render` was implicit,
// the subcommand's own page when it was named — and is what every diagnostic
// below follows. Returns the process exit code, or -1 to proceed to RunRender.
int ParseRenderOptions(int argc, char** argv, int first, void (*print_usage)(const char*), RenderOptions& opts) {
  for (int i = first; i < argc; i++) {
    switch (ParseSharedOption(argc, argv, i, opts.shared)) {
      case SharedStep::kConsumed:
        continue;
      case SharedStep::kHelp:
        print_usage(argv[0]);
        return 0;
      case SharedStep::kError:
        print_usage(argv[0]);
        return 1;
      case SharedStep::kNotShared:
        break;
    }
    std::string_view arg = argv[i];
    if (arg == "-o") {
      if (++i >= argc) {
        std::cerr << "Error: -o requires an argument\n\n";
        print_usage(argv[0]);
        return 1;
      }
      opts.output_dir = argv[i];
    } else if (arg == "--format") {
      if (++i >= argc) {
        std::cerr << "Error: --format requires an argument\n\n";
        print_usage(argv[0]);
        return 1;
      }
      opts.image_format = argv[i];
      if (opts.image_format != "jpg" && opts.image_format != "png") {
        std::cerr << "Error: --format must be 'jpg' or 'png', got '" << opts.image_format << "'\n\n";
        print_usage(argv[0]);
        return 1;
      }
    } else if (arg == "--quality") {
      if (++i >= argc) {
        std::cerr << "Error: --quality requires an argument\n\n";
        print_usage(argv[0]);
        return 1;
      }
      try {
        opts.jpeg_quality = std::stoi(argv[i]);
      } catch (const std::exception&) {
        std::cerr << "Error: --quality requires a numeric value, got '" << argv[i] << "'\n\n";
        print_usage(argv[0]);
        return 1;
      }
      if (opts.jpeg_quality < 1 || opts.jpeg_quality > 100) {
        std::cerr << "Error: --quality must be between 1 and 100, got " << opts.jpeg_quality << "\n\n";
        print_usage(argv[0]);
        return 1;
      }
    } else if (arg == "--workers") {
      if (!TryParseWorkersOption(argc, argv, i, opts.cli_workers)) {
        print_usage(argv[0]);
        return 1;
      }
    } else {
      std::cerr << "Error: unknown option: " << arg << "\n\n";
      print_usage(argv[0]);
      return 1;
    }
  }

#ifdef _WIN32
  ReparseWidePathArgs(opts.shared.config_filename, &opts.output_dir, /*csv_path=*/nullptr);
#endif

  if (!FinishSharedOptions(opts.shared)) {
    print_usage(argv[0]);
    return 1;
  }
  return -1;
}

// Parses argv[first..) as the `benchmark` option set: the shared options and nothing else.
// The old `--benchmark` mode accepted `-o` (unused) and `--workers` (announced as ignored);
// under a subcommand neither has a reason to be accepted, so both are unknown options here.
// Returns the process exit code, or -1 to proceed to RunBenchmark.
int ParseBenchmarkOptions(int argc, char** argv, int first, BenchmarkOptions& opts) {
  for (int i = first; i < argc; i++) {
    switch (ParseSharedOption(argc, argv, i, opts.shared)) {
      case SharedStep::kConsumed:
        continue;
      case SharedStep::kHelp:
        PrintBenchmarkUsage(argv[0]);
        return 0;
      case SharedStep::kError:
        PrintBenchmarkUsage(argv[0]);
        return 1;
      case SharedStep::kNotShared:
        break;
    }
    std::cerr << "Error: unknown option: " << argv[i] << "\n\n";
    PrintBenchmarkUsage(argv[0]);
    return 1;
  }

#ifdef _WIN32
  ReparseWidePathArgs(opts.shared.config_filename, /*output_dir=*/nullptr, /*csv_path=*/nullptr);
#endif

  if (!FinishSharedOptions(opts.shared)) {
    PrintBenchmarkUsage(argv[0]);
    return 1;
  }
  return -1;
}

// `--rays <N>`: digits with an optional K / M / G suffix (case-insensitive), e.g. 20M. The
// suffix is decimal (1000-based), as the config's ray_num is read by a human and 20M means
// twenty million rays, not 20 * 2^20. Zero is rejected: the C API would honour it as a
// zero-ray run, which is never what a person typed --rays for.
std::optional<LUMICE_RayCount> ParseRayBudget(std::string_view text) {
  if (text.empty()) {
    return std::nullopt;
  }
  unsigned long long multiplier = 1;
  std::string_view digits = text;
  switch (std::tolower(static_cast<unsigned char>(text.back()))) {
    case 'k':
      multiplier = 1'000ull;
      digits.remove_suffix(1);
      break;
    case 'm':
      multiplier = 1'000'000ull;
      digits.remove_suffix(1);
      break;
    case 'g':
      multiplier = 1'000'000'000ull;
      digits.remove_suffix(1);
      break;
    default:
      break;
  }
  const auto parsed = ParseStrictUnsigned(digits);
  if (!parsed.has_value() || *parsed == 0) {
    return std::nullopt;
  }
  if (*parsed > std::numeric_limits<unsigned long long>::max() / multiplier) {
    return std::nullopt;
  }
  return static_cast<LUMICE_RayCount>(*parsed * multiplier);
}

// `--symmetry <spec>`: `none`, or any combination of the letters P, B, D (case-insensitive,
// repeats harmless), as a LUMICE_RAYPATH_SYMMETRY_* bit set. Anything else is nullopt.
std::optional<std::uint8_t> ParseSymmetrySpec(std::string_view text) {
  if (text.empty()) {
    return std::nullopt;
  }
  std::string lowered(text);
  for (char& c : lowered) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  if (lowered == "none") {
    return 0;
  }
  std::uint8_t bits = 0;
  for (const char c : lowered) {
    switch (c) {
      case 'p':
        bits |= LUMICE_RAYPATH_SYMMETRY_P;
        break;
      case 'b':
        bits |= LUMICE_RAYPATH_SYMMETRY_B;
        break;
      case 'd':
        bits |= LUMICE_RAYPATH_SYMMETRY_D;
        break;
      default:
        return std::nullopt;
    }
  }
  return bits;
}

// Parses argv[first..) as the `analyze` option set: the shared options, --workers (the same
// step `render` takes), and the request. The per-ROI options are checked against the ROI once
// every token is in, so `--center` before or after `--roi cone` reads the same; each rejection
// names the option and the ROI it conflicts with. Returns the process exit code, or -1 to
// proceed to RunAnalyze.
int ParseAnalyzeOptions(int argc, char** argv, int first, AnalyzeOptions& opts) {
  bool roi_given = false;
  for (int i = first; i < argc; i++) {
    switch (ParseSharedOption(argc, argv, i, opts.shared)) {
      case SharedStep::kConsumed:
        continue;
      case SharedStep::kHelp:
        PrintAnalyzeUsage(argv[0]);
        return 0;
      case SharedStep::kError:
        PrintAnalyzeUsage(argv[0]);
        return 1;
      case SharedStep::kNotShared:
        break;
    }
    std::string_view arg = argv[i];
    // Every option below takes a value; one check for "the value is missing".
    const bool takes_value = arg == "--roi" || arg == "--center" || arg == "--radius" || arg == "--render-id" ||
                             arg == "--symmetry" || arg == "--rays" || arg == "--seed" || arg == "--csv";
    if (takes_value && i + 1 >= argc) {
      std::cerr << "Error: " << arg << " requires an argument\n\n";
      PrintAnalyzeUsage(argv[0]);
      return 1;
    }
    if (arg == "--workers") {
      if (!TryParseWorkersOption(argc, argv, i, opts.cli_workers)) {
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
    } else if (arg == "--roi") {
      const std::string_view roi = argv[++i];
      if (roi == "sky") {
        opts.roi_mode = LUMICE_RAYPATH_ROI_FULL_SKY;
      } else if (roi == "frame") {
        opts.roi_mode = LUMICE_RAYPATH_ROI_IN_FRAME;
      } else if (roi == "cone") {
        opts.roi_mode = LUMICE_RAYPATH_ROI_CONE;
      } else {
        std::cerr << "Error: --roi must be 'sky', 'frame' or 'cone', got '" << roi << "'\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      roi_given = true;
    } else if (arg == "--center") {
      const std::string_view value = argv[++i];
      const auto comma = value.find(',');
      const auto alt = comma == std::string_view::npos ? std::nullopt : ParseStrictFloat(value.substr(0, comma));
      const auto az = comma == std::string_view::npos ? std::nullopt : ParseStrictFloat(value.substr(comma + 1));
      if (!alt.has_value() || !az.has_value()) {
        std::cerr << "Error: --center must be '<altitude_deg>,<azimuth_deg>' (two numbers), got '" << value << "'\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      if (*alt < -90.0f || *alt > 90.0f) {
        std::cerr << "Error: --center altitude must be between -90 and 90 degrees, got " << *alt << "\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      opts.center_alt_deg = alt;
      opts.center_az_deg = az;
    } else if (arg == "--radius") {
      const std::string_view value = argv[++i];
      const auto radius = ParseStrictFloat(value);
      if (!radius.has_value() || !(*radius > 0.0f) || *radius > 180.0f) {
        std::cerr << "Error: --radius must be a number of degrees in (0, 180], got '" << value << "'\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      opts.radius_deg = radius;
    } else if (arg == "--render-id") {
      const std::string_view value = argv[++i];
      const auto id = ParseStrictUnsigned(value);
      if (!id.has_value() || *id > static_cast<unsigned long long>(std::numeric_limits<int>::max())) {
        std::cerr << "Error: --render-id requires a non-negative integer, got '" << value << "'\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      opts.render_id = static_cast<int>(*id);
    } else if (arg == "--symmetry") {
      const std::string_view value = argv[++i];
      const auto bits = ParseSymmetrySpec(value);
      if (!bits.has_value()) {
        std::cerr << "Error: --symmetry must be 'none' or a combination of P, B, D (e.g. PBD, PD), got '" << value
                  << "'\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      opts.symmetry_bits = *bits;
    } else if (arg == "--rays") {
      const std::string_view value = argv[++i];
      const auto rays = ParseRayBudget(value);
      if (!rays.has_value()) {
        std::cerr << "Error: --rays must be a positive integer with an optional K/M/G suffix (e.g. 20M), got '" << value
                  << "'\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      opts.ray_num = rays;
    } else if (arg == "--seed") {
      const std::string_view value = argv[++i];
      const auto seed = ParseStrictUnsigned(value);
      if (!seed.has_value() || *seed == 0 || *seed > std::numeric_limits<unsigned int>::max()) {
        std::cerr << "Error: --seed must be a positive integer up to " << std::numeric_limits<unsigned int>::max()
                  << ", got '" << value << "'\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
      opts.sim_seed = static_cast<unsigned int>(*seed);
    } else if (arg == "--csv") {
      opts.csv_path = argv[++i];
      if (opts.csv_path.empty()) {
        std::cerr << "Error: --csv requires a file path\n\n";
        PrintAnalyzeUsage(argv[0]);
        return 1;
      }
    } else {
      std::cerr << "Error: unknown option: " << arg << "\n\n";
      PrintAnalyzeUsage(argv[0]);
      return 1;
    }
  }

  // The per-ROI options against the ROI. Each message says which option and which ROI, so the
  // fix is readable off the line: a `cone` without its geometry, a geometry option under an ROI
  // that has no centre, a frame id under an ROI that has no frame.
  const char* roi_name = opts.roi_mode == LUMICE_RAYPATH_ROI_CONE     ? "cone" :
                         opts.roi_mode == LUMICE_RAYPATH_ROI_IN_FRAME ? "frame" :
                                                                        "sky";
  if (opts.roi_mode == LUMICE_RAYPATH_ROI_CONE) {
    if (!opts.center_alt_deg.has_value() || !opts.radius_deg.has_value()) {
      std::cerr << "Error: --roi cone requires both --center <alt_deg>,<az_deg> and --radius <deg>; missing "
                << (!opts.center_alt_deg.has_value() && !opts.radius_deg.has_value() ? "--center and --radius" :
                    !opts.center_alt_deg.has_value()                                 ? "--center" :
                                                                                       "--radius")
                << "\n\n";
      PrintAnalyzeUsage(argv[0]);
      return 1;
    }
  } else {
    if (opts.center_alt_deg.has_value() || opts.radius_deg.has_value()) {
      std::cerr << "Error: " << (opts.center_alt_deg.has_value() ? "--center" : "--radius") << " only applies to "
                << "--roi cone, but the ROI is '" << roi_name << "'" << (roi_given ? "" : " (the default)") << "\n\n";
      PrintAnalyzeUsage(argv[0]);
      return 1;
    }
  }
  if (opts.roi_mode != LUMICE_RAYPATH_ROI_IN_FRAME && opts.render_id.has_value()) {
    std::cerr << "Error: --render-id only applies to --roi frame, but the ROI is '" << roi_name << "'"
              << (roi_given ? "" : " (the default)") << "\n\n";
    PrintAnalyzeUsage(argv[0]);
    return 1;
  }

#ifdef _WIN32
  ReparseWidePathArgs(opts.shared.config_filename, /*output_dir=*/nullptr, &opts.csv_path);
#endif

  if (!FinishSharedOptions(opts.shared)) {
    PrintAnalyzeUsage(argv[0]);
    return 1;
  }
  return -1;
}

// --- Subcommand bodies ---------------------------------------------------------

// Benchmark: dual-pass (single-worker + multi-worker) on the legacy CPU route, one steady pass
// on a GPU route. The worker counts are part of the measurement methodology, not a default a
// user preference may override: the "single" pass is 1 worker BECAUSE that is what per-core
// efficiency means, and "multi" is PhysicalCoreCount() BECAUSE that is what the parallel figure
// is defined against — which is why this subcommand has no --workers option at all.
//
// Note "multi" is an EXPLICIT worker count (num_workers > 0), so it deliberately escapes the cap
// the automatic default is subject to (kMaxDefaultWorkerCount, server.cpp). On a machine with
// more physical cores than that cap, "multi" therefore does not report the throughput the
// shipping default produces: it reports full-core parallel efficiency, which is what this
// pass is FOR. doc/performance-testing.md says the same thing to whoever reads the number.
int RunBenchmark(const BenchmarkOptions& opts) {
  const SharedOptions& shared = opts.shared;
  std::ifstream config_file(shared.config_filename);
  if (!config_file.is_open()) {
    std::cerr << "Error: cannot open config file: " << shared.config_filename.u8string() << "\n";
    return 1;
  }
  nlohmann::json config_json;
  try {
    config_file >> config_json;
  } catch (const nlohmann::json::parse_error& e) {
    std::cerr << "Error: invalid JSON in config file: " << e.what() << "\n";
    return 1;
  }

  WarnIfLastScatteringLayerProbNonzero(config_json);

  auto cores = static_cast<int>(std::thread::hardware_concurrency());

  // The GPU route is single-engine (worker_count=1, server.cpp) regardless of
  // num_workers. Its "single" (2M-ray, JIT-warmup-dominated) and "multi"
  // (full-ray, warm) passes are therefore NOT single-vs-parallel — the gap is
  // warmup + ray-count, not workers. So for the GPU route we run ONE steady pass
  // (kept labelled "multi" for output continuity) and skip the meaningless warmup
  // pass. Only the legacy CPU route keeps the genuine dual-pass: "single" = 1
  // worker (per-core efficiency), "multi" = PhysicalCoreCount() workers (real
  // parallelism — full-core, which is above the shipping default's cap on a
  // machine with many cores; see the note at the top of this function). LUMICE_WillUseGpuRoute is env-aware
  // (LUMICE_TRACE_BACKEND wins over --backend), so this matches how bench_throughput.py selects a GPU run.
  bool gpu_route = LUMICE_WillUseGpuRoute(shared.preferred_backend) != 0;

  if (!gpu_route) {
    // Pass 1: reduced rays, single worker (label="single") — CPU per-core efficiency.
    auto single_config = config_json;
    single_config["scene"]["ray_num"] = kBenchmarkSingleRays;
    RunBenchmarkPass(single_config.dump(), 1, "single", cores, shared.log_level, shared.preferred_backend);
  } else {
    // GPU route: run one throwaway warm-up pass BEFORE the timed steady pass.
    // Purpose: the first GPU call in a process triggers backend-specific lazy
    // init (CUDA context ~1.3s cold; Metal PSO compile), and when the measured
    // pass is short enough for `active_sec` to round to ~0 the `rate_basis`
    // ladder falls back to `wall_fallback` and folds that one-time init into
    // `rays_per_sec` — this is the mechanism behind the "stoch vs det 15.7×"
    // cold/warm measurement artifact. Silent=true suppresses both the JSON
    // stdout line and the wall_fallback stderr warning so downstream parsers
    // (bench_throughput.py, test_metal_throughput.py) see the same single
    // `[BENCHMARK]` line they always did. The warm-up config clones the
    // user's original scene (same crystals / renders / spectrum), only overriding
    // scene.ray_num to a small finite value — this keeps warm-up wall-time
    // short (ray count is not what drives init cost) while still exercising
    // the real GPU dispatch path the steady pass will use.
    auto warmup_config = config_json;
    warmup_config["scene"]["ray_num"] = kBenchmarkGpuWarmupRays;
    RunBenchmarkPass(warmup_config.dump(), 1, "warmup", cores, shared.log_level, shared.preferred_backend,
                     /*silent=*/true);
  }

  // Steady pass (label="multi"): original ray count. CPU = PhysicalCoreCount()
  // workers (parallel — explicit, so uncapped: this is the parallel-efficiency figure,
  // not the shipping default); GPU = the single engine (the representative steady figure).
  int multi_workers = gpu_route ? 1 : lumice::PhysicalCoreCount();
  RunBenchmarkPass(config_json.dump(), multi_workers, "multi", cores, shared.log_level, shared.preferred_backend);

  return 0;
}

int RunRender(const RenderOptions& opts) {
  const SharedOptions& shared = opts.shared;
  if (!std::filesystem::is_directory(opts.output_dir)) {
    std::cerr << "Error: output directory does not exist: " << opts.output_dir.u8string() << "\n";
    return 1;
  }

  LUMICE_ServerConfig server_config{};
  server_config.preferred_backend = shared.preferred_backend;
  server_config.num_workers = opts.cli_workers;  // 0 = automatic: one per physical core, capped (server.cpp)
  auto* server = LUMICE_CreateServerEx(&server_config);
  LUMICE_SetLogLevel(server, shared.log_level);

  WarnIfLastScatteringLayerProbNonzero(shared.config_filename);
  // File -> handle -> commit. The parse half reports through its return code only (no internal
  // LOG_ERROR, unlike the core commit path), so the CLI says out loud which half failed instead
  // of exiting 1 with nothing on the console.
  LUMICE_Scene* raw_scene = nullptr;
  if (auto err = LUMICE_SceneFromJsonFile(shared.config_filename.u8string().c_str(), &raw_scene); err != LUMICE_OK) {
    std::cerr << "Error: failed to load configuration from file '" << shared.config_filename.u8string()
              << "' (error code " << static_cast<int>(err) << ")\n";
    LUMICE_DestroyServer(server);
    return 1;
  }
  ScenePtr scene(raw_scene);
  if (LUMICE_CommitScene(server, scene.get(), /*out_reused=*/nullptr) != LUMICE_OK) {
    LUMICE_DestroyServer(server);
    return 1;
  }

  // Check completion FIRST, then sleep — the reverse order put a floor of one
  // kSaveInterval under every render, however small. The two frequencies are
  // deliberately separate: completion is cheap and polled at kFinePollInterval,
  // while the materialization below is a full render and stays paced at
  // kSaveInterval (so a run shorter than one interval now performs it exactly
  // once, in the final fetch after the loop, instead of twice).
  auto next_save_time = std::chrono::steady_clock::now() + kSaveInterval;

  while (true) {
    // Completion via the explicit single-source lifecycle: COMPLETED = a finite
    // run drained clean (incl. zero-output convergence). Replaces the fragile
    // `IDLE && stats.sim_ray_num>0` side-signal (scrum-296.7 early-IDLE truncation
    // history). Infinite runs never reach COMPLETED, matching prior behavior.
    LUMICE_SimLifecycleResult lifecycle{};
    if (LUMICE_GetSimLifecycle(server, &lifecycle) == LUMICE_OK && lifecycle.lifecycle == LUMICE_LIFECYCLE_COMPLETED) {
      break;
    }

    auto now = std::chrono::steady_clock::now();
    if (now >= next_save_time) {
      SaveRenderResults(server, opts.output_dir, opts.image_format, opts.jpeg_quality);
      SaveCompositeResults(server, opts.output_dir, opts.image_format, opts.jpeg_quality);
      PrintStats(server);
      next_save_time = std::chrono::steady_clock::now() + kSaveInterval;
    }

    std::this_thread::sleep_for(kFinePollInterval);
  }

  // Final fetch after loop exit
  SaveRenderResults(server, opts.output_dir, opts.image_format, opts.jpeg_quality);
  SaveCompositeResults(server, opts.output_dir, opts.image_format, opts.jpeg_quality);
  PrintStats(server);
  PrintColorClassSignal(server, shared.config_filename);

  LUMICE_DestroyServer(server);
  return 0;
}

// --- analyze -------------------------------------------------------------------

// Set by the SIGINT handler, read by RunAnalyze's loop. A signal handler may touch nothing but
// a lock-free atomic (or a volatile sig_atomic_t); the static_assert is the compile-time form of
// that rule. The flag is only ever installed by RunAnalyze — `render` keeps the default
// disposition (an interrupted render writes nothing; that is a separate change).
std::atomic<bool> g_analyze_stop_requested{ false };
static_assert(std::atomic<bool>::is_always_lock_free, "the SIGINT flag must be async-signal-safe");

void HandleAnalyzeSigint(int /*signal*/) {
  g_analyze_stop_requested.store(true, std::memory_order_relaxed);
}

// The LUMICE_AnnotationView of a `--roi frame` request, from the SCENE's own renderer: the
// entry whose id is `render_id`, or the first one (index 0) when nullopt. Read back through
// LUMICE_SceneGetRenderer, so what arrives is the LUMICE_RenderParam the engine will use — core's
// defaults already applied, the lens and visible-range enums already LUMICE_* constants — and
// the copy below is field for field, with no parsing, no default and no enum table of this
// CLI's own. `render_id` is matched against LUMICE_RenderParam::id, which is NOT the array
// index it is read at (lumice.h, at the getter): a document's render[] keeps the ids it
// declared, so the entries are enumerated and compared. On failure `error` says what, and the
// caller exits 1 before the analysis starts.
bool BuildFrameViewFromScene(const LUMICE_Scene* scene, std::optional<int> render_id, LUMICE_AnnotationView* out,
                             std::string* error) {
  LUMICE_RenderParam r{};
  if (render_id.has_value()) {
    std::string seen;
    bool found = false;
    for (int index = 0; LUMICE_SceneGetRenderer(scene, index, &r) == LUMICE_OK; index++) {
      if (r.id == *render_id) {
        found = true;
        break;
      }
      seen += (seen.empty() ? "" : ", ") + std::to_string(r.id);
    }
    if (!found) {
      *error = "--render-id " + std::to_string(*render_id) +
               " names no render[] entry in the config (ids present: " + (seen.empty() ? std::string("none") : seen) +
               ")";
      return false;
    }
  } else if (LUMICE_SceneGetRenderer(scene, 0, &r) != LUMICE_OK) {
    *error = "--roi frame needs a render[] entry in the config to define the frame, and this config has none";
    return false;
  }
  *out = LUMICE_AnnotationView{};
  out->width = r.resolution_w;
  out->height = r.resolution_h;
  out->lens_type = r.lens_type;
  out->lens_fov = r.lens_fov;
  out->lens_shift[0] = r.lens_shift[0];
  out->lens_shift[1] = r.lens_shift[1];
  out->overlap = r.overlap;
  out->view_azimuth = r.view_azimuth;
  out->view_elevation = r.view_elevation;
  out->view_roll = r.view_roll;
  out->visible = r.visible;
  out->front = r.front;
  if (out->width <= 0 || out->height <= 0) {
    *error = "the render[] entry for --roi frame has a non-positive resolution";
    return false;
  }
  return true;
}

// One read of the analysis frame under `symmetry`: the frame-level info and every entry, the
// sentinel read the GUI's RefreshAnalysisEntries makes. `present` false means the server holds
// no analysis frame yet (the first snapshot has not landed) — not an error, just nothing to say.
struct AnalysisRead {
  LUMICE_RaypathAnalysisInfo info{};
  std::vector<LUMICE_RaypathHistogramEntry> entries;
};

bool ReadAnalysisFrame(LUMICE_Server* server, std::uint8_t symmetry, AnalysisRead* out) {
  LUMICE_ResultFrame* raw_frame = nullptr;
  if (LUMICE_AcquireResultFrame(server, &raw_frame) != LUMICE_OK || raw_frame == nullptr) {
    return false;
  }
  lumice::ResultFramePtr frame(raw_frame);
  out->info = LUMICE_RaypathAnalysisInfo{};
  out->entries.clear();
  if (LUMICE_FrameGetRaypathAnalysisInfo(frame.get(), symmetry, &out->info) != LUMICE_OK || out->info.present == 0) {
    return false;
  }
  // One more slot than entries: the sentinel (count == 0) lands at [entry_count] when the frame
  // holds exactly entry_count entries, and the read below stops at it in every case.
  std::vector<LUMICE_RaypathHistogramEntry> raw(static_cast<size_t>(std::max(out->info.entry_count, 0)) + 1);
  if (LUMICE_FrameGetRaypathAnalysis(frame.get(), symmetry, raw.data(), out->info.entry_count) != LUMICE_OK) {
    return false;
  }
  size_t n = 0;
  while (n < raw.size() && raw[n].count != 0) {
    ++n;
  }
  raw.resize(n);
  out->entries = std::move(raw);
  return true;
}

std::string LocalTimeNow() {
  const std::time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm tm{};
#if defined(_WIN32)
  const bool ok = localtime_s(&tm, &now) == 0;
#else
  const bool ok = localtime_r(&now, &tm) != nullptr;
#endif
  char buf[32] = { 0 };
  if (ok) {
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm);
  }
  return buf;
}

// The CSV of one read, through the one formatter the GUI's Export CSV also uses
// (util/raypath_analysis_display.hpp). The display radius is the request radius: the CLI has no
// slider, so every ring is summed and cone_rings_summed reads "N / N".
std::string AnalysisCsvText(const AnalysisRead& read, const LUMICE_RaypathAnalysisRequest& request,
                            std::uint8_t symmetry, std::string_view exported_at) {
  lumice::RaypathAnalysisCsvInputs in;
  in.roi_mode = request.roi_mode;
  std::copy(std::begin(request.cone_center), std::end(request.cone_center), std::begin(in.cone_center_dir));
  in.cone_request_radius_rad = read.info.present ? read.info.cone_radius_rad : request.cone_radius_rad;
  in.cone_ring_count = read.info.present ? read.info.cone_ring_count : request.cone_ring_count;
  in.cone_display_radius_deg = in.cone_request_radius_rad * lumice::kRad2Deg;
  in.symmetry_bits = symmetry;
  in.other_energy = read.info.other_energy;
  in.other_count = read.info.other_count;
  in.truncated_chain_count = read.info.truncated_chain_count;
  const lumice::RaypathDisplayOrder order =
      lumice::ComputeRaypathDisplayOrder(read.entries, in.roi_mode, in.cone_ring_count, in.cone_request_radius_rad,
                                         in.cone_display_radius_deg, in.other_energy);
  return lumice::BuildRaypathAnalysisCsv(read.entries, order, in, exported_at);
}

// Write `text` to `path` so that the file at `path` is complete at every instant: the bytes go
// to a sibling temporary first and are renamed over the target, which is atomic on every
// filesystem this CLI runs on. A reader that opens the path mid-run sees either the previous
// complete file or the new one, never a prefix.
bool WriteFileAtomically(const std::filesystem::path& path, const std::string& text, std::string* error) {
  std::filesystem::path tmp = path;
  tmp += ".tmp";
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
      *error = "cannot open '" + tmp.u8string() + "' for writing";
      return false;
    }
    out << text;
    if (!out.good()) {
      *error = "write to '" + tmp.u8string() + "' failed";
      out.close();
      std::error_code ec;
      std::filesystem::remove(tmp, ec);  // same cleanup discipline as the rename branch below
      return false;
    }
  }
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    *error = "cannot rename '" + tmp.u8string() + "' to '" + path.u8string() + "': " + ec.message();
    std::filesystem::remove(tmp, ec);
    return false;
  }
  return true;
}

int RunAnalyze(const AnalyzeOptions& opts) {
  const SharedOptions& shared = opts.shared;
  // The config is read here as JSON only for what the CLI itself needs from it — the last-layer
  // warning — so that diagnostic is given before a server exists. Everything else about the
  // document, the `--roi frame` renderer included, is the engine's reading of it: the file is
  // parsed through LUMICE_SceneFromJsonFile, the same path `render` takes.
  nlohmann::json config_json;
  {
    std::ifstream config_file(shared.config_filename);
    if (!config_file.is_open()) {
      std::cerr << "Error: cannot open config file: " << shared.config_filename.u8string() << "\n";
      return 1;
    }
    try {
      config_file >> config_json;
    } catch (const nlohmann::json::parse_error& e) {
      std::cerr << "Error: invalid JSON in config file: " << e.what() << "\n";
      return 1;
    }
  }
  WarnIfLastScatteringLayerProbNonzero(config_json);

  LUMICE_ServerConfig server_config{};
  server_config.preferred_backend = shared.preferred_backend;
  server_config.num_workers = opts.cli_workers;  // 0 = automatic (server.cpp); a seed forces 1
  server_config.sim_seed = opts.sim_seed;
  auto* server = LUMICE_CreateServerEx(&server_config);
  LUMICE_SetLogLevel(server, shared.log_level);

  // The scene is loaded before the request is assembled: a `--roi frame` request frames its
  // ROI on the scene's own renderer, read back through the handle (BuildFrameViewFromScene).
  LUMICE_Scene* raw_scene = nullptr;
  if (auto err = LUMICE_SceneFromJsonFile(shared.config_filename.u8string().c_str(), &raw_scene); err != LUMICE_OK) {
    std::cerr << "Error: failed to load configuration from file '" << shared.config_filename.u8string()
              << "' (error code " << static_cast<int>(err) << ")\n";
    LUMICE_DestroyServer(server);
    return 1;
  }
  ScenePtr scene(raw_scene);

  LUMICE_RaypathAnalysisRequest request{};
  request.roi_mode = opts.roi_mode;
  if (opts.roi_mode == LUMICE_RAYPATH_ROI_CONE) {
    lumice::AltAzToDir(*opts.center_alt_deg, *opts.center_az_deg, request.cone_center);
    request.cone_radius_rad = *opts.radius_deg * lumice::kDeg2Rad;
    request.cone_ring_count = lumice::kRaypathAnalysisConeRingCount;
  } else if (opts.roi_mode == LUMICE_RAYPATH_ROI_IN_FRAME) {
    std::string error;
    if (scene == nullptr || !BuildFrameViewFromScene(scene.get(), opts.render_id, &request.frame_view, &error)) {
      // `scene` cannot be null here — the load above returned on failure — and the test stays as
      // a statement of what this branch relies on, for whoever next reorders this function.
      std::cerr << "Error: " << error << "\n";
      LUMICE_DestroyServer(server);
      return 1;
    }
  }
  if (opts.ray_num.has_value()) {
    request.infinite = 0;
    request.ray_num = *opts.ray_num;
  } else {
    request.infinite = LUMICE_RAYPATH_RAY_BUDGET_SCENE_DEFAULT;
  }

  // A --csv target that cannot be written is found out now, not after the run: the first
  // periodic write would report it, but a one-second run has no periodic write.
  if (!opts.csv_path.empty()) {
    const auto parent = opts.csv_path.parent_path();
    if (!parent.empty() && !std::filesystem::is_directory(parent)) {
      std::cerr << "Error: --csv directory does not exist: " << parent.u8string() << "\n";
      LUMICE_DestroyServer(server);
      return 1;
    }
  }

  if (auto err = LUMICE_StartRaypathAnalysis(server, scene.get(), &request); err != LUMICE_OK) {
    std::cerr << "Error: the analysis could not start (error code " << static_cast<int>(err) << ")\n";
    LUMICE_DestroyServer(server);
    return 1;
  }

  // Ctrl-C ends the run and still writes what it accumulated: the handler only raises the
  // flag; the loop below sees it, stops the server (the frame published after
  // LUMICE_StopServer returns carries the histogram consumed up to the stop — lumice.h v4.34)
  // and falls through to the same final write a completed run makes. Installed after the run
  // is started so a Ctrl-C during setup keeps the default disposition (exit, nothing written).
  g_analyze_stop_requested.store(false, std::memory_order_relaxed);
  std::signal(SIGINT, HandleAnalyzeSigint);

  const auto start_time = std::chrono::steady_clock::now();
  auto next_save_time = start_time + kSaveInterval;
  AnalysisRead read;
  bool interrupted = false;
  // Same two clocks as RunRender: completion polled at kFinePollInterval, materialization
  // paced at kSaveInterval. The one exit test — COMPLETED, or the flag — is the same for a
  // finite and an "infinite" budget: the first reaches COMPLETED on its own, the second only
  // ever leaves through the flag, and neither needs the CLI to know which it is.
  while (true) {
    LUMICE_SimLifecycleResult lifecycle{};
    if (LUMICE_GetSimLifecycle(server, &lifecycle) == LUMICE_OK && lifecycle.lifecycle == LUMICE_LIFECYCLE_COMPLETED) {
      break;
    }
    if (g_analyze_stop_requested.load(std::memory_order_relaxed)) {
      interrupted = true;
      LUMICE_StopServer(server);
      break;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_save_time) {
      LUMICE_RayCount rays = 0;
      LUMICE_GetSimRayCount(server, &rays);
      const bool have_frame = ReadAnalysisFrame(server, opts.symmetry_bits, &read);
      if (have_frame && !opts.csv_path.empty()) {
        std::string error;
        if (!WriteFileAtomically(opts.csv_path, AnalysisCsvText(read, request, opts.symmetry_bits, LocalTimeNow()),
                                 &error)) {
          std::cerr << "Error: " << error << "\n";
          LUMICE_StopServer(server);
          LUMICE_DestroyServer(server);
          return 1;
        }
      }
      const double elapsed = std::chrono::duration<double>(now - start_time).count();
      std::uint64_t recorded = 0;
      if (have_frame) {
        recorded = static_cast<std::uint64_t>(read.info.other_count);
        for (const auto& e : read.entries) {
          recorded += static_cast<std::uint64_t>(e.count);
        }
      }
      // Progress is stderr's: stdout carries the product output and nothing else.
      std::cerr << "[analyze] " << rays << " rays traced, " << recorded << " raypaths recorded, " << std::fixed
                << std::setprecision(1) << elapsed << " s elapsed\n";
      next_save_time = std::chrono::steady_clock::now() + kSaveInterval;
    }
    std::this_thread::sleep_for(kFinePollInterval);
  }
  std::signal(SIGINT, SIG_DFL);

  // The final materialization — the one write of a run shorter than a save interval, the
  // last of a longer one, and the only time stdout is written to.
  if (!ReadAnalysisFrame(server, opts.symmetry_bits, &read)) {
    std::cerr << "Warning: the run ended before it published a result; writing an empty result\n";
    read = AnalysisRead{};
  }
  const std::string csv = AnalysisCsvText(read, request, opts.symmetry_bits, LocalTimeNow());
  int rc = 0;
  if (opts.csv_path.empty()) {
    // stdout is the CSV's alone: the engine's diagnostics go to stderr (util/logger.hpp), as
    // does every line this subcommand prints about its own progress.
    std::cout << csv << std::flush;
    if (!std::cout.good()) {
      std::cerr << "Error: writing the CSV to stdout failed\n";
      rc = 1;
    }
  } else {
    std::string error;
    if (!WriteFileAtomically(opts.csv_path, csv, &error)) {
      std::cerr << "Error: " << error << "\n";
      rc = 1;
    }
  }
  if (interrupted) {
    std::cerr << "[analyze] interrupted; the result accumulated up to the stop has been written\n";
  }
  LUMICE_DestroyServer(server);
  return rc;
}

}  // namespace


int main(int argc, char** argv) {
  // Subcommand dispatch. `argv[1]` is the subcommand only when it spells one; every
  // other argv[1] — an option, or nothing — is the implicit `render`, whose options
  // then start at argv[1] instead of argv[2] and whose help is the top-level page.
  const std::string_view subcommand = argc > 1 ? std::string_view(argv[1]) : std::string_view();

  if (subcommand == kSubcommandBenchmark) {
    BenchmarkOptions opts;
    if (int rc = ParseBenchmarkOptions(argc, argv, /*first=*/2, opts); rc >= 0) {
      return rc;
    }
    return RunBenchmark(opts);
  }

  if (subcommand == kSubcommandAnalyze) {
    AnalyzeOptions opts;
    if (int rc = ParseAnalyzeOptions(argc, argv, /*first=*/2, opts); rc >= 0) {
      return rc;
    }
    return RunAnalyze(opts);
  }

  const bool explicit_render = subcommand == kSubcommandRender;
  RenderOptions opts;
  if (int rc = ParseRenderOptions(argc, argv, /*first=*/explicit_render ? 2 : 1,
                                  explicit_render ? PrintRenderUsage : PrintTopLevelUsage, opts);
      rc >= 0) {
    return rc;
  }
  return RunRender(opts);
}
