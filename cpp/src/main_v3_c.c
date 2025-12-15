#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "include/cserver.h"

int main(int argc, char** argv) {
  printf("argc: %d\n", argc);
  const char* config_filename = NULL;

  for (int i = 1; i < argc; i++) {
    printf("argv[%d]: %s\n", i, argv[i]);
    if (strcmp(argv[i], "-f") == 0) {
      config_filename = argv[i + 1];
    }
  }
  printf("config_filename: %s\n", config_filename);

  char buf[1024 * 1024];  // 1MB
  FILE* config_file = fopen(config_filename, "r");
  fread(buf, 1, 1024 * 1024, config_file);

  HS_HaloSimServer* server = HS_CreateServer();
  HS_CommitConfig(server, buf);

  sleep(1);
  while (1) {
    HS_SimResult* result = NULL;
    for (result = HS_GetAllResults(server);  //
         HS_HasNextResult(result);           //
         result = HS_GetNextResult(result)) {
      HS_SimResultType res_type = HS_QueryResultType(result);
      switch (res_type) {
        case HS_RESULT_RENDER: {
          HS_RenderResult p = HS_GetRenderResult(result);
          printf("<render result>[%02d]: w: %d, h: %d, buff: %p\n",  //
                 p.renderer_id_, p.img_width_, p.img_height_, p.img_buffer_);
        } break;
        case HS_RESULT_STATS: {
          HS_StatsResult p = HS_GetStatsResult(result);
          printf("<stats result>: sim_rays: %lu, crystals: %lu\n", p.sim_ray_num_, p.crystal_num_);
        } break;
        case HS_RESULT_NONE:
          printf("<none result>\n");
          break;
      }
    }
    HS_DeleteAllResults(result);

    HS_ServerState state = HS_QueryServerState(server);
    if (state == HS_SERVER_IDLE) {
      printf("server is idle! abort!\n");
      HS_StopServer(server);
      break;
    }
  }
  HS_DestroyServer(server);
  return 0;
}