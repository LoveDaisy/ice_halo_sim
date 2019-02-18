#include <chrono>
#include <cstdio>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "render.h"
#include "context.h"

using namespace IceHalo;
using namespace cv;

int main(int argc, char* argv[]) {
  if (argc != 2) {
    printf("USAGE: %s config.json\n", argv[0]);
    return -1;
  }

  auto start = std::chrono::system_clock::now();
  auto ctx = RenderContext::CreateFromFile(argv[1]);
  ctx->LoadData();

  auto flatRgbData = new uint8_t[3 * ctx->GetImageWidth() * ctx->GetImageHeight()];
  ctx->RenderToRgb(flatRgbData);

  Mat img(ctx->GetImageHeight(), ctx->GetImageWidth(), CV_8UC3, flatRgbData);
  cvtColor(img, img, COLOR_RGB2BGR);
  try {
    imwrite(ctx->GetImagePath(), img);
  } catch (cv::Exception& ex) {
    fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
    return -1;
  }
  delete[] flatRgbData;

  auto t1 = std::chrono::system_clock::now();
  std::chrono::duration<float, std::ratio<1, 1000> > diff = t1 - start;
  printf("Total: %.2fms\n", diff.count());
}
