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
            << "  --backend <name>   Trace backend: auto, cpu, or metal (default: auto).\n"
            << "                     'auto' and 'cpu' both select the CPU route today; 'metal'\n"
            << "                     falls back to CPU if unavailable. The LUMICE_TRACE_BACKEND\n"
            << "                     env var, if set, still overrides this (debug/CI only).\n"
            << "  --benchmark        Run dual-mode benchmark (single-worker + multi-worker) and output\n"
            << "                     two [BENCHMARK] JSON lines with per-core and parallel efficiency data\n"
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
  auto t_run_start = std::chrono::steady_clock::now();
  auto t_active_start = t_run_start;
  LUMICE_RayCount rays_at_active_start = 0;
  bool active_started = false;
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

    if (state == LUMICE_SERVER_IDLE && cur_rays > 0) {
      auto t_end = now;
      LUMICE_RayCount r_end = cur_rays;
      double wall_sec = std::chrono::duration<double>(t_end - t_run_start).count();
      double setup_sec = std::chrono::duration<double>(t_active_start - t_run_start).count();
      double active_sec = std::chrono::duration<double>(t_end - t_active_start).count();

      // Steady-window rate excludes setup and the first sampled chunk. Guard the
      // short-run degenerate case (run completed within the first one/two polls,
      // so r_end == rays_at_active_start or active window ~0) by falling back to
      // the active-window-from-first-rays rate, then whole-wall as last resort.
      double rays_per_sec = 0.0;
      const char* rate_basis = "wall_fallback";
      if (active_sec > 1e-4 && r_end > rays_at_active_start) {
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
        std::cerr << "Error: --backend must be 'auto', 'cpu', or 'metal', got '" << argv[i] << "'\n\n";
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
    // task-268.7: server is always single-engine; num_workers is ignored. The
    // "multi" pass remains as a higher ray-count comparison vs. the reduced
    // "single" pass — both run with one Simulator. The `multi_workers` label
    // is kept only for benchmark-output continuity.
    int multi_workers = lumice::PhysicalCoreCount();

    // Pass 1: reduced rays (label="single")
    auto single_config = config_json;
    single_config["scene"]["ray_num"] = kBenchmarkSingleRays;
    RunBenchmarkPass(single_config.dump(), 1, "single", cores, log_level, preferred_backend);

    // Pass 2: original ray count (label="multi"; worker count is still 1)
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
