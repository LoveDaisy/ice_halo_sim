#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context.h"
#include "simulation.h"
#include "render.h"
#include "mymath.h"

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::printf("USAGE: %s <config-file>\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();
  IceHalo::SimulationContextPtr sim_ctx = IceHalo::SimulationContext::CreateFromFile(argv[1]);
  IceHalo::Simulator simulator(sim_ctx);

  IceHalo::RenderContextPtr render_ctx = IceHalo::RenderContext::CreateFromFile(argv[1]);
  IceHalo::SpectrumRenderer renderer(render_ctx);

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000> > diff = t - start;
  std::printf("Initialization: %.2fms\n", diff.count());

  IceHalo::File file(render_ctx->GetImagePath().c_str());
  if (!file.Open(IceHalo::OpenMode::kWrite | IceHalo::OpenMode::kBinary)) {
    std::fprintf(stderr, "Cannot create output image file!\n");
    return -1;
  }
  file.Close();

  size_t total_ray_num = 0;
  auto flat_rgb_data = new uint8_t[3 * render_ctx->GetImageWidth() * render_ctx->GetImageHeight()];
  while (true) {
    for (auto wl : sim_ctx->GetWavelengths()) {
      std::printf("starting at wavelength: %.1f\n", wl.first);

      sim_ctx->SetCurrentWavelength(wl.first, wl.second);

      auto t0 = std::chrono::system_clock::now();
      simulator.Start();
      auto t1 = std::chrono::system_clock::now();
      diff = t1 - t0;
      std::printf("Ray tracing: %.2fms\n", diff.count());

      auto num = simulator.GetFinalRaySegments().size();
      auto* curr_data = new float[num * 4];
      auto* p = curr_data;
      for (const auto& r : simulator.GetFinalRaySegments()) {
        const auto axis_rot = r->root->main_axis_rot.val();
        assert(r->root);
        IceHalo::Math::RotateZBack(axis_rot, r->dir.val(), p);
        p[3] = r->w;
        p += 4;
      }
      renderer.LoadData(wl.first, wl.second, curr_data, num);
      delete[] curr_data;
    }

    renderer.RenderToRgb(flat_rgb_data);

    cv::Mat img(render_ctx->GetImageHeight(), render_ctx->GetImageWidth(), CV_8UC3, flat_rgb_data);
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    try {
      cv::imwrite(render_ctx->GetImagePath(), img);
    } catch (cv::Exception& ex) {
      std::fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
      break;
    }

    t = std::chrono::system_clock::now();
    total_ray_num += sim_ctx->GetTotalInitRays() * sim_ctx->GetWavelengths().size();
    diff = t - start;
    std::printf("=== Total %zu rays finished! ===\n", total_ray_num);
    std::printf("=== Spent %.3f sec!          ===\n", diff.count() / 1000);
  }

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  std::printf("Total: %.3fs\n", diff.count() / 1e3);

  delete[] flat_rgb_data;
  return 0;
}