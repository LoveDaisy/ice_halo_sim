#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context.h"
#include "render.h"
#include "simulation.h"

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::printf("USAGE: %s <config-file>\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();
  icehalo::ProjectContextPtr proj_ctx = icehalo::ProjectContext::CreateFromFile(argv[1]);
  icehalo::Simulator simulator(proj_ctx);
  icehalo::SpectrumRenderer renderer(proj_ctx);

  auto t = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000>> diff = t - start;
  std::printf("Initialization: %.2fms\n", diff.count());

  icehalo::File file(proj_ctx->GetDefaultImagePath().c_str());
  if (!file.Open(icehalo::FileOpenMode::kWrite)) {
    std::fprintf(stderr, "Cannot create output image file!\n");
    return -1;
  }
  file.Close();

  size_t total_ray_num = 0;
  while (true) {
    const auto& wavelengths = proj_ctx->wavelengths_;
    for (size_t i = 0; i < wavelengths.size(); i++) {
      std::printf("starting at wavelength: %d\n", wavelengths[i].wavelength);
      simulator.SetCurrentWavelengthIndex(i);

      auto t0 = std::chrono::system_clock::now();
      simulator.Run();
      auto t1 = std::chrono::system_clock::now();
      diff = t1 - t0;
      std::printf("Ray tracing: %.2fms\n", diff.count());

      renderer.LoadData(static_cast<float>(wavelengths[i].wavelength), wavelengths[i].weight,
                        simulator.GetSimulationRayData().CollectFinalRayData());
    }

    renderer.RenderToImage();

    cv::Mat img(proj_ctx->render_ctx_->GetImageHeight(), proj_ctx->render_ctx_->GetImageWidth(), CV_8UC3,
                renderer.GetImageBuffer());
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
    try {
      cv::imwrite(proj_ctx->GetDefaultImagePath(), img);
    } catch (cv::Exception& ex) {
      std::fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
      break;
    }

    t = std::chrono::system_clock::now();
    total_ray_num += proj_ctx->GetInitRayNum() * wavelengths.size();
    diff = t - start;
    std::printf("=== Total %zu rays finished! ===\n", total_ray_num);
    std::printf("=== Spent %.3f sec!          ===\n", diff.count() / 1000);
  }

  auto end = std::chrono::system_clock::now();
  diff = end - start;
  std::printf("Total: %.3fs\n", diff.count() / 1e3);

  return 0;
}