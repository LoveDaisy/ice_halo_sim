#include <chrono>
#include <cstdio>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "render.h"
#include "context.h"

using namespace IceHalo;
using namespace cv;

int main(int argc, char *argv[])
{
    if (argc < 3) {
        printf("USAGE: %s config.json data_file1 [data_file2 ...]\n", argv[0]);
        return -1;
    }

    auto start = std::chrono::system_clock::now();
    auto t0 = start;
    auto t1 = start;
    std::chrono::duration<double> diff;

    ContextParser *parser = ContextParser::createFileParser(argv[1]);
    RenderContext ctx;
    try {
        parser->parseRenderingSettings(ctx);
    } catch (std::invalid_argument &e) {
        fprintf(stderr, "Parsing error! Exit!\n Message: %s\n", e.what());
        return -1;
    }

    for (int fi = 2; fi < argc; fi++) {
        t0 = std::chrono::system_clock::now();
        auto num = ctx.loadDataFromFile(argv[fi]);
        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf(" Loading data (%d/%d): %.2fms; total %d pts\n", fi - 1, argc - 2, diff.count() * 1.0e3, num);
    }

    auto wlNum = ctx.getWavelengthNum();
    float *wlData = new float[wlNum];
    float *flatSpecData = new float[wlNum * ctx.getImageWidth() * ctx.getImageHeight()];
    ctx.copySpectrumData(wlData, flatSpecData);

    SpectrumRenderer render;
    uint8_t *flatRgbData = new uint8_t[3 * ctx.getImageWidth() * ctx.getImageHeight()];
    render.rgb(wlNum, wlData, ctx.getImageWidth() * ctx.getImageHeight(), flatSpecData, flatRgbData);
    delete[] wlData;
    delete[] flatSpecData;

    Mat img(ctx.getImageHeight(), ctx.getImageWidth(), CV_8UC3, flatRgbData);
    cvtColor(img, img, COLOR_RGB2BGR);
    try {
        imwrite("img.png", img);
    } catch (cv::Exception& ex) {
        fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
        return -1;
    }

    t1 = std::chrono::system_clock::now();
    diff = t1 - start;
    printf("Total: %.2fms\n", diff.count() * 1.0e3);
}