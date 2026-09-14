#include <cctype>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <string_view>
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
#include "util/result_frame.hpp"

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
// argv[1] names the subcommand: `render` or `benchmark`. Anything else in that
// slot (an option, or nothing at all) means the implicit `render`, so the
// `Lumice -f config.json ...` form every README / quickstart / user script uses
// keeps working verbatim. Each subcommand accepts ONLY its own option set and
// reports everything else as an unknown option — there is no "accepted but
// ignored" flag anywhere, which is what keeps the "which flag means what in
// which mode" matrix from growing a dimension per mode. A third subcommand
// (`analyze` is the planned one) slots in as one more Parse*/Run* pair below
// plus one more branch in main(); nothing shared has to learn about it.

constexpr std::string_view kSubcommandRender = "render";
constexpr std::string_view kSubcommandBenchmark = "benchmark";

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

void PrintRenderOptions() {
  std::cout << kHelpConfigOption
            << "  -o <dir>           Output directory for rendered images (default: current directory)\n"
            << "  --format <fmt>     Output image format: jpg or png (default: jpg)\n"
            << "  --quality <1-100>  JPEG quality (default: 95, ignored for PNG)\n"
            << kHelpBackendOption
            << "  --workers <N>      Number of CPU simulation worker threads (default: automatic —\n"
            << "                     one per physical core, capped at a ceiling above which no\n"
            << "                     machine measured ran faster; an explicit N is never capped).\n"
            << "                     Machine-dependent, so it is a command-line switch rather than\n"
            << "                     a config-file field: a config travels between machines and a\n"
            << "                     worker count should not travel with it. Ignored on a GPU route\n"
            << "                     (single engine).\n"
            << kHelpLogAndHelpOptions;
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

// Top-level `-h` (no subcommand named): the subcommand overview followed by the
// implicit subcommand's full option list, so the help a user reaches from the
// form they already know is complete on its own.
void PrintTopLevelUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " [render] -f <config_file> [options]\n"
            << "       " << prog_name << " benchmark -f <config_file> [options]\n"
            << "       " << prog_name << " <subcommand> -h\n"
            << "\n"
            << "Lumice — simulate ice halos by tracing rays through ice crystals.\n"
            << "\n"
            << "Subcommands:\n"
            << "  render             Simulate and write halo images. This is the default when\n"
            << "                     no subcommand is given: `" << prog_name << " -f ...` is a render.\n"
            << "  benchmark          Run a throughput benchmark and print [BENCHMARK] JSON\n"
            << "                     (`" << prog_name << " benchmark -h` for its options)\n"
            << "\n"
            << "Options for render (the default subcommand):\n";
  PrintRenderOptions();
  std::cout << "\n"
            << "Examples:\n";
  PrintRenderExamples(prog_name);
  std::cout << "  " << prog_name << " benchmark -f examples/bench_config.json\n";
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
// Only path arguments (-f, -o) need wide-char re-parsing; ASCII-only args
// (the subcommand token, --format, --quality, --workers) are safe as-is.
// `output_dir` is null for a subcommand that has no -o: the option was already
// rejected by that subcommand's parser, so there is nothing to re-read.
void ReparseWidePathArgs(std::filesystem::path& config_filename, std::filesystem::path* output_dir) {
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
      if (++i >= argc) {
        std::cerr << "Error: --workers requires an argument\n\n";
        print_usage(argv[0]);
        return 1;
      }
      // std::stoi stops at the first non-digit WITHOUT throwing, so "3abc" would parse as 3 and be
      // silently accepted. The `pos == size` check is what makes a trailing-garbage argument an
      // error rather than a value the user never typed. std::stoi also skips LEADING whitespace and
      // accepts a leading '+'/'-' before parsing, so those two checks alone would let " 3" or "+3"
      // through as if the user had typed a bare "3" — reject anything that doesn't start with a
      // digit up front, since AC1 only ever wants a positive integer typed as one.
      const std::string workers_arg = argv[i];
      if (workers_arg.empty() || !std::isdigit(static_cast<unsigned char>(workers_arg[0]))) {
        std::cerr << "Error: --workers requires a numeric value, got '" << workers_arg << "'\n\n";
        print_usage(argv[0]);
        return 1;
      }
      std::size_t parsed_len = 0;
      try {
        opts.cli_workers = std::stoi(workers_arg, &parsed_len);
      } catch (const std::exception&) {
        std::cerr << "Error: --workers requires a numeric value, got '" << workers_arg << "'\n\n";
        print_usage(argv[0]);
        return 1;
      }
      if (parsed_len != workers_arg.size()) {
        std::cerr << "Error: --workers requires a numeric value, got '" << workers_arg << "'\n\n";
        print_usage(argv[0]);
        return 1;
      }
      if (opts.cli_workers <= 0) {
        std::cerr << "Error: --workers must be a positive integer, got " << opts.cli_workers << "\n\n";
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
  ReparseWidePathArgs(opts.shared.config_filename, &opts.output_dir);
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
  ReparseWidePathArgs(opts.shared.config_filename, /*output_dir=*/nullptr);
#endif

  if (!FinishSharedOptions(opts.shared)) {
    PrintBenchmarkUsage(argv[0]);
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

  const bool explicit_render = subcommand == kSubcommandRender;
  RenderOptions opts;
  if (int rc = ParseRenderOptions(argc, argv, /*first=*/explicit_render ? 2 : 1,
                                  explicit_render ? PrintRenderUsage : PrintTopLevelUsage, opts);
      rc >= 0) {
    return rc;
  }
  return RunRender(opts);
}
