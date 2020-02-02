#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "context.h"
#include "mymath.h"
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
  if (!file.Open(icehalo::openmode::kWrite | icehalo::openmode::kBinary)) {
    std::fprintf(stderr, "Cannot create output image file!\n");
    return -1;
  }
  file.Close();

  size_t total_ray_num = 0;
  std::unique_ptr<uint8_t[]> flat_rgb_data{
    new uint8_t[3 * proj_ctx->render_ctx_.GetImageWidth() * proj_ctx->render_ctx_.GetImageHeight()]
  };
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

      const auto& ray_seg_set = simulator.GetFinalRaySegments();
      auto num = ray_seg_set.size();
      std::unique_ptr<float[]> curr_data{ new float[num * 4] };
      auto* p = curr_data.get();
      for (const auto& r : ray_seg_set) {
        assert(r->root_ctx);
        auto axis_rot = r->root_ctx->main_axis_rot.val();
        icehalo::math::RotateZBack(axis_rot, r->dir.val(), p);
        p[3] = r->w;
        p += 4;
      }
      renderer.LoadData(static_cast<float>(wavelengths[i].wavelength), wavelengths[i].weight, curr_data.get(), num);
    }

    renderer.RenderToRgb(flat_rgb_data.get());

    cv::Mat img(proj_ctx->render_ctx_.GetImageHeight(), proj_ctx->render_ctx_.GetImageWidth(), CV_8UC3,
                flat_rgb_data.get());
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