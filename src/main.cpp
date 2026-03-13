#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>

#include "lumice.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace {

constexpr int kJpegQuality = 95;
constexpr auto kPollInterval = std::chrono::seconds(1);

void PrintUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " -f <config_file> [options]\n"
            << "\n"
            << "Lumice — simulate ice halos by tracing rays through ice crystals.\n"
            << "\n"
            << "Options:\n"
            << "  -f <file>    Specify the configuration file (required)\n"
            << "  -o <dir>     Output directory for rendered images (default: current directory)\n"
            << "  -v           Verbose output (trace level logging)\n"
            << "  -d           Debug output (debug level logging)\n"
            << "  -h           Show this help message and exit\n"
            << "\n"
            << "Examples:\n"
            << "  " << prog_name << " -f config.json\n"
            << "  " << prog_name << " -f config.json -o /tmp/output\n"
            << "  " << prog_name << " -f config.json -v\n";
}

std::string FormatImagePath(const std::filesystem::path& output_dir, int renderer_id) {
  std::ostringstream oss;
  oss << "img_" << std::setfill('0') << std::setw(2) << renderer_id << ".jpg";
  return (output_dir / oss.str()).string();
}

void SaveRenderResults(LUMICE_Server* server, const std::filesystem::path& output_dir) {
  LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1];
  if (LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) != LUMICE_OK) {
    return;
  }
  for (int i = 0; renders[i].img_buffer != nullptr; i++) {
    auto filepath = FormatImagePath(output_dir, renders[i].renderer_id);
    int ok = stbi_write_jpg(filepath.c_str(), renders[i].img_width, renders[i].img_height, 3, renders[i].img_buffer,
                            kJpegQuality);
    if (ok) {
      std::cout << "Saved: " << filepath << " (" << renders[i].img_width << "x" << renders[i].img_height << ")\n";
    } else {
      std::cerr << "Error: failed to write " << filepath << "\n";
    }
  }
}

void PrintStats(LUMICE_Server* server) {
  LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1];
  if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) != LUMICE_OK) {
    return;
  }
  for (int i = 0; stats[i].sim_ray_num != 0; i++) {
    std::cout << "Stats: sim_rays=" << stats[i].sim_ray_num << ", crystals=" << stats[i].crystal_num << "\n";
  }
}

}  // namespace


int main(int argc, char** argv) {
  std::string config_filename;
  std::filesystem::path output_dir = ".";
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
    } else if (arg == "-v") {
      log_level = LUMICE_LOG_TRACE;
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

  if (config_filename.empty()) {
    std::cerr << "Error: configuration file is required (-f <file>)\n\n";
    PrintUsage(argv[0]);
    return 1;
  }

  if (!std::filesystem::is_directory(output_dir)) {
    std::cerr << "Error: output directory does not exist: " << output_dir.string() << "\n";
    return 1;
  }

  auto* server = LUMICE_CreateServer();
  LUMICE_InitLogger(server);
  LUMICE_SetLogLevel(server, log_level);

  if (LUMICE_CommitConfigFromFile(server, config_filename.c_str()) != LUMICE_OK) {
    LUMICE_DestroyServer(server);
    return 1;
  }

  while (true) {
    std::this_thread::sleep_for(kPollInterval);

    SaveRenderResults(server, output_dir);
    PrintStats(server);

    LUMICE_ServerState state{};
    if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
      break;
    }
  }

  // Final fetch after loop exit
  SaveRenderResults(server, output_dir);
  PrintStats(server);

  LUMICE_DestroyServer(server);
  return 0;
}
