#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
// clang-format off
#ifdef _WIN32
#include <windows.h>   // Must come before shellapi.h (defines EXTERN_C etc.)
#include <shellapi.h>  // CommandLineToArgvW
#endif
// clang-format on

#include "lumice.h"
#include "util/cpu_info.hpp"

#ifdef _WIN32
#define STBIW_WINDOWS_UTF8
#endif
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace {

constexpr int kDefaultJpegQuality = 95;
constexpr auto kPollInterval = std::chrono::seconds(1);
// Fine poll granularity (was 100ms). At 100ms the IDLE-detection quantization
// alone could add up to a full poll interval to wall time; for a fast backend
// whose run completes in ~0.2s that deflated rays_per_sec by >30%. 5ms caps the
// trailing quantization at a few ms while staying sleep-based (negligible CPU
// steal from trace workers). See task-fix-throughput-bench-honesty.
constexpr auto kBenchmarkPollInterval = std::chrono::milliseconds(5);
constexpr int kBenchmarkSingleRays = 2'000'000;
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

void PrintUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " -f <config_file> [options]\n"
            << "\n"
            << "Lumice — simulate ice halos by tracing rays through ice crystals.\n"
            << "\n"
            << "Options:\n"
            << "  -f <file>          Specify the configuration file (required)\n"
            << "  -o <dir>           Output directory for rendered images (default: current directory)\n"
            << "  --format <fmt>     Output image format: jpg or png (default: jpg)\n"
            << "  --quality <1-100>  JPEG quality (default: 95, ignored for PNG)\n"
            << "  --backend <name>   Trace backend: auto, cpu, metal, or cuda (default: auto).\n"
            << "                     'auto' and 'cpu' both select the CPU route today; 'metal'\n"
            << "                     falls back to CPU if unavailable. The LUMICE_TRACE_BACKEND\n"
            << "                     env var, if set, still overrides this (debug/CI only).\n"
            << "  --benchmark        Run a throughput benchmark and output [BENCHMARK] JSON. The legacy\n"
            << "                     CPU route runs a dual pass (single-worker + multi-worker → per-core\n"
            << "                     and parallel-efficiency data); a GPU route is single-engine, so it\n"
            << "                     runs one steady pass only (single/multi would not be parallel)\n"
            << "  -v                 Verbose output (trace level logging)\n"
            << "  -d                 Debug output (debug level logging)\n"
            << "  -h                 Show this help message and exit\n"
            << "\n"
            << "Examples:\n"
            << "  " << prog_name << " -f config.json\n"
            << "  " << prog_name << " -f config.json -o /tmp/output\n"
            << "  " << prog_name << " -f config.json --format png\n"
            << "  " << prog_name << " -f config.json --quality 80\n"
            << "  " << prog_name << " -f config.json --backend metal\n"
            << "  " << prog_name << " -f config.json --benchmark\n"
            << "  " << prog_name << " -f config.json -v\n";
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

std::filesystem::path FormatImagePath(const std::filesystem::path& output_dir, int renderer_id,
                                      std::string_view format) {
  std::ostringstream oss;
  oss << "img_" << std::setfill('0') << std::setw(2) << renderer_id << "." << format;
  return output_dir / oss.str();
}

void SaveRenderResults(LUMICE_Server* server, const std::filesystem::path& output_dir, std::string_view image_format,
                       int jpeg_quality) {
  LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1]{};
  if (LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) != LUMICE_OK) {
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

void PrintStats(LUMICE_Server* server) {
  LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1]{};
  if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) != LUMICE_OK) {
    return;
  }
  for (int i = 0; stats[i].sim_ray_num != 0; i++) {
    std::cout << "Stats: sim_rays=" << stats[i].sim_ray_num << ", crystals=" << stats[i].crystal_num << "\n";
  }
}

void RunBenchmarkPass(const std::string& config_str, int num_workers, const char* mode, int cores,
                      LUMICE_LogLevel log_level, int preferred_backend) {
  LUMICE_ServerConfig server_config{};
  server_config.num_workers = num_workers;
  server_config.preferred_backend = preferred_backend;
  auto* server = LUMICE_CreateServerEx(&server_config);
  LUMICE_SetLogLevel(server, log_level);

  if (LUMICE_CommitConfig(server, config_str.c_str()) != LUMICE_OK) {
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
    std::this_thread::sleep_for(kBenchmarkPollInterval);
    LUMICE_ServerState state{};
    LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1]{};
    if (LUMICE_QueryServerState(server, &state) != LUMICE_OK) {
      continue;
    }
    bool have_stats = LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) == LUMICE_OK;
    LUMICE_RayCount cur_rays = have_stats ? stats[0].sim_ray_num : 0;
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
        rays_per_sec = static_cast<double>(r_end) / active_sec;
        rate_basis = "active_short";
      } else {
        rays_per_sec = wall_sec > 0 ? static_cast<double>(r_end) / wall_sec : 0.0;
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
      if (drain_count_mode) {
        result["n_drains_in_window"] = n_drains_in_window;
        result["window_sec"] = std::round(window_sec * 1000.0) / 1000.0;
        result["window_rays"] = window_rays;
      }
      std::cout << "[BENCHMARK] " << result.dump() << "\n";
      break;
    }
  }

  LUMICE_DestroyServer(server);
}

}  // namespace


int main(int argc, char** argv) {
  std::filesystem::path config_filename;
  std::filesystem::path output_dir = ".";
  std::string image_format = "jpg";
  int jpeg_quality = kDefaultJpegQuality;
  bool benchmark_mode = false;
  int preferred_backend = LUMICE_BACKEND_CPU;
  auto log_level = LUMICE_LOG_INFO;

  for (int i = 1; i < argc; i++) {
    std::string_view arg = argv[i];
    if (arg == "-f") {
      if (++i >= argc) {
        std::cerr << "Error: -f requires an argument\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
      config_filename = argv[i];
    } else if (arg == "-o") {
      if (++i >= argc) {
        std::cerr << "Error: -o requires an argument\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
      output_dir = argv[i];
    } else if (arg == "--format") {
      if (++i >= argc) {
        std::cerr << "Error: --format requires an argument\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
      image_format = argv[i];
      if (image_format != "jpg" && image_format != "png") {
        std::cerr << "Error: --format must be 'jpg' or 'png', got '" << image_format << "'\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
    } else if (arg == "--quality") {
      if (++i >= argc) {
        std::cerr << "Error: --quality requires an argument\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
      try {
        jpeg_quality = std::stoi(argv[i]);
      } catch (const std::exception&) {
        std::cerr << "Error: --quality requires a numeric value, got '" << argv[i] << "'\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
      if (jpeg_quality < 1 || jpeg_quality > 100) {
        std::cerr << "Error: --quality must be between 1 and 100, got " << jpeg_quality << "\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
    } else if (arg == "--backend") {
      if (++i >= argc) {
        std::cerr << "Error: --backend requires an argument\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
      preferred_backend = ParseBackend(argv[i]);
      if (preferred_backend < 0) {
        std::cerr << "Error: --backend must be 'auto', 'cpu', 'metal', or 'cuda', got '" << argv[i] << "'\n\n";
        PrintUsage(argv[0]);
        return 1;
      }
    } else if (arg == "--benchmark") {
      benchmark_mode = true;
    } else if (arg == "-v") {
      log_level = LUMICE_LOG_VERBOSE;
    } else if (arg == "-d") {
      log_level = LUMICE_LOG_DEBUG;
    } else if (arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else {
      std::cerr << "Error: unknown option: " << arg << "\n\n";
      PrintUsage(argv[0]);
      return 1;
    }
  }

#ifdef _WIN32
  // Re-parse file paths from wide-char command line for full Unicode support.
  // argv[i] on Windows uses ANSI codepage, which loses non-ASCII characters.
  // Only path arguments (-f, -o) need wide-char re-parsing; ASCII-only args
  // (--format, --quality) are safe as-is.
  {
    int wargc = 0;
    wchar_t** wargv = CommandLineToArgvW(GetCommandLineW(), &wargc);
    if (wargv) {
      for (int i = 1; i < wargc; i++) {
        std::wstring_view warg = wargv[i];
        if (warg == L"-f" && i + 1 < wargc) {
          config_filename = wargv[++i];
        } else if (warg == L"-o" && i + 1 < wargc) {
          output_dir = wargv[++i];
        }
      }
      LocalFree(wargv);
    }
  }
#endif

  if (config_filename.empty()) {
    std::cerr << "Error: configuration file is required (-f <file>)\n\n";
    PrintUsage(argv[0]);
    return 1;
  }

  // Requested Metal but the machine can't provide it → fall back to CPU with a
  // visible notice rather than silently. (The core would fall back anyway; this
  // just makes the substitution explicit on the CLI.)
  if (preferred_backend == LUMICE_BACKEND_METAL && !LUMICE_IsBackendAvailable(LUMICE_BACKEND_METAL)) {
    std::cerr << "Warning: --backend metal requested but no Metal device is available; using CPU.\n";
    preferred_backend = LUMICE_BACKEND_CPU;
  }
  if (preferred_backend == LUMICE_BACKEND_CUDA && !LUMICE_IsBackendAvailable(LUMICE_BACKEND_CUDA)) {
    std::cerr << "Warning: --backend cuda requested but no eligible CUDA device is available; using CPU.\n";
    preferred_backend = LUMICE_BACKEND_CPU;
  }

  // Benchmark mode: dual-pass (single-worker + multi-worker)
  if (benchmark_mode) {
    std::ifstream config_file(config_filename);
    if (!config_file.is_open()) {
      std::cerr << "Error: cannot open config file: " << config_filename.u8string() << "\n";
      return 1;
    }
    nlohmann::json config_json;
    try {
      config_file >> config_json;
    } catch (const nlohmann::json::parse_error& e) {
      std::cerr << "Error: invalid JSON in config file: " << e.what() << "\n";
      return 1;
    }

    auto cores = static_cast<int>(std::thread::hardware_concurrency());

    // The GPU route is single-engine (worker_count=1, server.cpp) regardless of
    // num_workers. Its "single" (2M-ray, JIT-warmup-dominated) and "multi"
    // (full-ray, warm) passes are therefore NOT single-vs-parallel — the gap is
    // warmup + ray-count, not workers. So for the GPU route we run ONE steady pass
    // (kept labelled "multi" for output continuity) and skip the meaningless warmup
    // pass. Only the legacy CPU route keeps the genuine dual-pass: "single" = 1
    // worker (per-core efficiency), "multi" = PhysicalCoreCount() workers (real
    // parallelism). LUMICE_WillUseGpuRoute is env-aware (LUMICE_TRACE_BACKEND wins
    // over --backend), so this matches how bench_throughput.py selects a GPU run.
    bool gpu_route = LUMICE_WillUseGpuRoute(preferred_backend) != 0;

    if (!gpu_route) {
      // Pass 1: reduced rays, single worker (label="single") — CPU per-core efficiency.
      auto single_config = config_json;
      single_config["scene"]["ray_num"] = kBenchmarkSingleRays;
      RunBenchmarkPass(single_config.dump(), 1, "single", cores, log_level, preferred_backend);
    }

    // Steady pass (label="multi"): original ray count. CPU = PhysicalCoreCount()
    // workers (parallel); GPU = the single engine (the representative steady figure).
    int multi_workers = gpu_route ? 1 : lumice::PhysicalCoreCount();
    RunBenchmarkPass(config_json.dump(), multi_workers, "multi", cores, log_level, preferred_backend);

    return 0;
  }

  if (!std::filesystem::is_directory(output_dir)) {
    std::cerr << "Error: output directory does not exist: " << output_dir.u8string() << "\n";
    return 1;
  }

  LUMICE_ServerConfig server_config{};
  server_config.preferred_backend = preferred_backend;
  auto* server = LUMICE_CreateServerEx(&server_config);
  LUMICE_SetLogLevel(server, log_level);

  if (LUMICE_CommitConfigFromFile(server, config_filename.u8string().c_str()) != LUMICE_OK) {
    LUMICE_DestroyServer(server);
    return 1;
  }

  auto t_start = std::chrono::steady_clock::now();

  while (true) {
    std::this_thread::sleep_for(kPollInterval);
    SaveRenderResults(server, output_dir, image_format, jpeg_quality);
    PrintStats(server);

    LUMICE_ServerState state{};
    LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1]{};
    if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
      if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) == LUMICE_OK && stats[0].sim_ray_num > 0) {
        break;
      }
    }
  }

  // Final fetch after loop exit
  SaveRenderResults(server, output_dir, image_format, jpeg_quality);
  PrintStats(server);

  LUMICE_DestroyServer(server);
  return 0;
}
