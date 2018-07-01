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

    SpectrumRenderer render;
    EquiAreaCameraProjection proj;
    std::unordered_map<int, float*> specData;
    
    const uint32_t BUFFER_SIZE = 1024 * 4;
    const uint32_t IMAGE_SIZE = 1801;
    float camRot[3] = { 90.0f, 0.0f, 0.0f };
    float fov = 55.0f;
    
    double totalW = 0;
    auto *readBuffer = new float[BUFFER_SIZE];
    for (int fi = 1; fi < argc; fi++) {
        const char* filename = argv[fi];
        std::FILE* fd = fopen(filename, "rb");
        auto readCount = fread(readBuffer, sizeof(float), 1, fd);
        if (readCount <= 0) {
            return -1;
        }

        auto wavelength = static_cast<int>(readBuffer[0]);
        if (wavelength < SpectrumRenderer::MIN_WL || wavelength > SpectrumRenderer::MAX_WL) {
            return -1;
        }
        printf("Wavelength: %d\n", wavelength);

        t0 = std::chrono::system_clock::now();
        std::vector<float> tmpData;
        while (true) {
            readCount = fread(readBuffer, sizeof(float), BUFFER_SIZE, fd);
            for (decltype(readCount) i = 0; i < readCount; i++) {
                tmpData.push_back(readBuffer[i]);
            }
            if (readCount < BUFFER_SIZE) {
                break;
            }
        }
        std::fclose(fd);

        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf(" Reading: %.2fms\n", diff.count() * 1.0e3);

        t0 = std::chrono::system_clock::now();
        auto totalPts = tmpData.size() / 4;
        auto *tmpDir = new float[totalPts * 3];
        auto *tmpW = new float[totalPts];
        for (decltype(totalPts) i = 0; i < totalPts; i++) {
            memcpy(tmpDir + i*3, tmpData.data() + i*4, 3 * sizeof(float));
            tmpW[i] = tmpData[i*4+3];
            totalW += tmpW[i];
        }

        auto *tmpXY = new int[totalPts * 2];
        proj.project(camRot, fov, totalPts, tmpDir, IMAGE_SIZE, IMAGE_SIZE, tmpXY);
        delete[] tmpDir;
        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf(" Projecting: %.2fms\n", diff.count() * 1.0e3);

        float *currentData = nullptr;
        auto it = specData.find(wavelength);
        if (it != specData.end()) {
            currentData = it->second;
        } else {
            currentData = new float[IMAGE_SIZE * IMAGE_SIZE];
            for (decltype(totalPts) i = 0; i < IMAGE_SIZE * IMAGE_SIZE; i++) {
                currentData[i] = 0;
            }
            specData[wavelength] = currentData;
        }
        printf("  map size: %lu\n", specData.size());

        t0 = std::chrono::system_clock::now();
        for (decltype(totalPts) i = 0; i < totalPts; i++) {
            int x = tmpXY[i * 2 + 0];
            int y = tmpXY[i * 2 + 1];
            if (x < 0 || x >= IMAGE_SIZE || y < 0 || y >= IMAGE_SIZE) {
                continue;
            }
            currentData[y * IMAGE_SIZE + x] += tmpW[i];
        }
        delete[] tmpXY;
        delete[] tmpW;
        t1 = std::chrono::system_clock::now();
        diff = t1 - t0;
        printf(" Adding: %.2fms\n", diff.count() * 1.0e3);
    }

    auto wlNum = specData.size();
    
    totalW /= wlNum;
    printf("Total w: %.4f\n", totalW);
    for (const auto &kv : specData) {
        for (decltype(wlNum) i = 0; i < IMAGE_SIZE * IMAGE_SIZE; i++) {
            kv.second[i] *= 8 * 4e3 / totalW;
        }
    }

    float *wlData = new float[wlNum];
    float *flatSpecData = new float[wlNum * IMAGE_SIZE * IMAGE_SIZE];
    uint8_t *flatRgbData = new uint8_t[3 * IMAGE_SIZE * IMAGE_SIZE];
    int k = 0;
    for (const auto &kv : specData) {
        wlData[k] = kv.first;
        memcpy(flatSpecData + k * IMAGE_SIZE * IMAGE_SIZE, kv.second, IMAGE_SIZE * IMAGE_SIZE * sizeof(float));
        k++;
    }
    render.rgb(wlNum, wlData, IMAGE_SIZE * IMAGE_SIZE, flatSpecData, flatRgbData);
    delete[] wlData;
    delete[] flatSpecData;

    Mat img(IMAGE_SIZE, IMAGE_SIZE, CV_8UC3, flatRgbData);
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