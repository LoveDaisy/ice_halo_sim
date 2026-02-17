#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "lumice.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

static const int kPollIntervalUs = 1000000;  // 1 second
static const int kJpegQuality = 95;

static void print_usage(const char* prog_name) {
  fprintf(stdout,
          "Usage: %s -f <config_file> [options]\n"
          "\n"
          "Lumice — simulate ice halos by tracing rays through ice crystals.\n"
          "\n"
          "Options:\n"
          "  -f <file>    Specify the configuration file (required)\n"
          "  -o <dir>     Output directory for rendered images (default: current directory)\n"
          "  -v           Verbose output (trace level logging)\n"
          "  -d           Debug output (debug level logging)\n"
          "  -h           Show this help message and exit\n"
          "\n"
          "Examples:\n"
          "  %s -f config.json\n"
          "  %s -f config.json -o /tmp/output\n"
          "  %s -f config.json -v\n",
          prog_name, prog_name, prog_name, prog_name);
}

static void save_render_results(LUMICE_Server* server, const char* output_dir) {
  LUMICE_RenderResult renders[LUMICE_MAX_RENDER_RESULTS + 1];
  if (LUMICE_GetRenderResults(server, renders, LUMICE_MAX_RENDER_RESULTS) == LUMICE_OK) {
    for (int i = 0; renders[i].img_buffer != NULL; i++) {
      char filepath[512];
      snprintf(filepath, sizeof(filepath), "%s/img_%02d.jpg", output_dir, renders[i].renderer_id);

      int ok = stbi_write_jpg(filepath, renders[i].img_width, renders[i].img_height, 3, renders[i].img_buffer,
                              kJpegQuality);
      if (ok) {
        printf("Saved: %s (%dx%d)\n", filepath, renders[i].img_width, renders[i].img_height);
      } else {
        fprintf(stderr, "Error: failed to write %s\n", filepath);
      }
    }
  }
}

static void print_stats(LUMICE_Server* server) {
  LUMICE_StatsResult stats[LUMICE_MAX_STATS_RESULTS + 1];
  if (LUMICE_GetStatsResults(server, stats, LUMICE_MAX_STATS_RESULTS) == LUMICE_OK) {
    for (int i = 0; stats[i].sim_ray_num != 0; i++) {
      printf("Stats: sim_rays=%lu, crystals=%lu\n", stats[i].sim_ray_num, stats[i].crystal_num);
    }
  }
}

int main(int argc, char** argv) {
  const char* config_filename = NULL;
  const char* output_dir = ".";
  LUMICE_LogLevel log_level = LUMICE_LOG_INFO;
  int opt;

  while ((opt = getopt(argc, argv, "f:o:vdh")) != -1) {
    switch (opt) {
      case 'f':
        config_filename = optarg;
        break;
      case 'o':
        output_dir = optarg;
        break;
      case 'v':
        log_level = LUMICE_LOG_TRACE;
        break;
      case 'd':
        log_level = LUMICE_LOG_DEBUG;
        break;
      case 'h':
        print_usage(argv[0]);
        return 0;
      default:
        print_usage(argv[0]);
        return 1;
    }
  }

  if (!config_filename) {
    fprintf(stderr, "Error: configuration file is required (-f <file>)\n\n");
    print_usage(argv[0]);
    return 1;
  }

  // Verify output directory exists
  struct stat st;
  if (stat(output_dir, &st) != 0 || !S_ISDIR(st.st_mode)) {
    fprintf(stderr, "Error: output directory does not exist: %s\n", output_dir);
    return 1;
  }

  LUMICE_Server* server = LUMICE_CreateServer();
  LUMICE_InitLogger(server);
  LUMICE_SetLogLevel(server, log_level);

  if (LUMICE_CommitConfigFromFile(server, config_filename) != LUMICE_OK) {
    LUMICE_DestroyServer(server);
    return 1;
  }

  while (1) {
    usleep(kPollIntervalUs);

    save_render_results(server, output_dir);
    print_stats(server);

    LUMICE_ServerState state;
    if (LUMICE_QueryServerState(server, &state) == LUMICE_OK && state == LUMICE_SERVER_IDLE) {
      break;
    }
  }

  // Final fetch after loop exit
  save_render_results(server, output_dir);
  print_stats(server);

  LUMICE_DestroyServer(server);
  return 0;
}
