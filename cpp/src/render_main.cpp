#include <chrono>
#include <cstdio>
#include <vector>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "render.h"

using namespace IceHalo;
using namespace cv;

int main(int argc, char *argv[])
{
    if (argc < 2) {
        printf("USAGE: %s data_file1 [data_file2 ...]\n", argv[0]);
        return -1;
    }

    auto start = std::chrono::system_clock::now();
    auto t0 = start;
    auto t1 = start;
    std::chrono::duration<double> diff;

    EquiAreaCameraProjection proj;
    std::unordered_map<int, float*> specData;
    
    const uint32_t IMAGE_SIZE = 1801;
    float camRot[3] = { 90.0f, 0.0f, 0.0f };
    float fov = 55.0f;

    RenderContext ctx(IMAGE_SIZE, IMAGE_SIZE, 
        [&camRot, fov, &proj](uint64_t totalPts, float *tmpDir, int imgWid, int imgHei, int *tmpXY){
            proj.project(camRot, fov, totalPts, tmpDir, imgWid, imgHei, tmpXY);
        }
    );

    for (int fi = 1; fi < argc; fi++) {
        t0 = std::chrono::system_clock::now();
        ctx.loadDataFromFile(argv[fi]);
        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf(" Loading data (%d/%d): %.2fms\n", fi, argc - 1, diff.count() * 1.0e3);
    }

    auto wlNum = ctx.getWavelengthNum();
    float *wlData = new float[wlNum];
    float *flatSpecData = new float[wlNum * ctx.imgHei * ctx.imgWid];
    ctx.copySpectrumData(wlData, flatSpecData);

    SpectrumRenderer render;
    uint8_t *flatRgbData = new uint8_t[3 * ctx.imgHei * ctx.imgWid];
    render.rgb(wlNum, wlData, ctx.imgHei * ctx.imgWid, flatSpecData, flatRgbData);
    delete[] wlData;
    delete[] flatSpecData;

    Mat img(ctx.imgHei, ctx.imgWid, CV_8UC3, flatRgbData);
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