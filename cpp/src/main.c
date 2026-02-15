#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "icehalo.h"

static const int kPollIntervalUs = 1000000;  // 1 second

static void print_usage(const char* prog_name) {
  fprintf(stdout,
          "Usage: %s -f <config_file> [options]\n"
          "\n"
          "Ice Halo Simulation — simulate ice halos by tracing rays through ice crystals.\n"
          "\n"
          "Options:\n"
          "  -f <file>    Specify the configuration file (required)\n"
          "  -v           Verbose output (trace level logging)\n"
          "  -d           Debug output (debug level logging)\n"
          "  -h           Show this help message and exit\n"
          "\n"
          "Examples:\n"
          "  %s -f config.json\n"
          "  %s -f config.json -v\n"
          "  %s -f config.json -d\n",
          prog_name, prog_name, prog_name, prog_name);
}

int main(int argc, char** argv) {
  const char* config_filename = NULL;
  HS_LogLevel log_level = HS_LOG_INFO;
  int opt;

  while ((opt = getopt(argc, argv, "f:vdh")) != -1) {
    switch (opt) {
      case 'f':
        config_filename = optarg;
        break;
      case 'v':
        log_level = HS_LOG_TRACE;
        break;
      case 'd':
        log_level = HS_LOG_DEBUG;
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

  HS_HaloSimServer* server = HS_CreateServer();
  HS_InitLogger(server);
  HS_SetLogLevel(server, log_level);

  if (HS_CommitConfigFromFile(server, config_filename) != 0) {
    HS_DestroyServer(server);
    return 1;
  }

  while (1) {
    usleep(kPollIntervalUs);

    HS_SimResult* result = NULL;
    for (result = HS_GetAllResults(server);  //
         HS_HasNextResult(result);           //
         result = HS_GetNextResult(result)) {
      HS_SimResultType res_type = HS_QueryResultType(result);
      switch (res_type) {
        case HS_RESULT_RENDER: {
          HS_RenderResult p = HS_GetRenderResult(result);
          printf("<render result>[%02d]: w: %d, h: %d, buff: %p\n",  //
                 p.renderer_id_, p.img_width_, p.img_height_, (const void*)p.img_buffer_);
        } break;
        case HS_RESULT_STATS: {
          HS_StatsResult p = HS_GetStatsResult(result);
          printf("<stats result>: sim_rays: %lu, crystals: %lu\n", p.sim_ray_num_, p.crystal_num_);
        } break;
        case HS_RESULT_NONE:
          break;
      }
    }
    HS_DeleteAllResults(result);

    HS_ServerState state = HS_QueryServerState(server);
    if (state == HS_SERVER_IDLE) {
      break;
    }
  }

  HS_DestroyServer(server);
  return 0;
}
