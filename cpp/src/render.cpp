#include "optics.h"
#include "render.h"
#include "mymath.h"


namespace IceHalo {

RenderContext::RenderContext(uint32_t imgHei, uint32_t imgWid, std::function<void(uint64_t, float*, int, int, int*)> proj) :
    imgHei(imgHei), imgWid(imgWid), totalW(0),
    proj(proj)
{ }


RenderContext::~RenderContext()
{
    for (const auto &kv : spectrumData) {
        delete[] kv.second;
    }
}


size_t RenderContext::getWavelengthNum() const
{
    return spectrumData.size();
}


void RenderContext::copySpectrumData(float *wavelengthData, float *spectrumData) const
{
    int k = 0;
    for (const auto &kv : this->spectrumData) {
        wavelengthData[k] = kv.first;
        memcpy(spectrumData + k * imgWid * imgHei, kv.second, imgWid * imgHei * sizeof(float));
        k++;
    }
    for (int i = 0; i < imgWid * imgHei * this->spectrumData.size(); i++) {
        spectrumData[i] *= 8 * 4e3 / totalW;
    }
}


int RenderContext::loadDataFromFile(const char* filename)
{
    const uint32_t BUFFER_SIZE = 1024 * 4;
    float* readBuffer = new float[BUFFER_SIZE];

    std::FILE* fd = fopen(filename, "rb");
    auto readCount = fread(readBuffer, sizeof(float), 1, fd);
    int totalCount = 0;
    if (readCount <= 0) {
        return -1;
    }

    auto wavelength = static_cast<int>(readBuffer[0]);
    if (wavelength < SpectrumRenderer::MIN_WL || wavelength > SpectrumRenderer::MAX_WL) {
        return -1;
    }

    std::vector<float> tmpData;
    while (true) {
        readCount = fread(readBuffer, sizeof(float), BUFFER_SIZE, fd);
        totalCount += readCount;
        for (decltype(readCount) i = 0; i < readCount; i++) {
            tmpData.push_back(readBuffer[i]);
        }
        if (readCount < BUFFER_SIZE) {
            break;
        }
    }
    std::fclose(fd);

    auto totalPts = tmpData.size() / 4;
    auto *tmpDir = new float[totalPts * 3];
    auto *tmpW = new float[totalPts];
    for (decltype(totalPts) i = 0; i < totalPts; i++) {
        memcpy(tmpDir + i*3, tmpData.data() + i*4, 3 * sizeof(float));
        tmpW[i] = tmpData[i*4+3];
        totalW += tmpW[i];
    }

    auto *tmpXY = new int[totalPts * 2];
    proj(totalPts, tmpDir, imgWid, imgHei, tmpXY);
    delete[] tmpDir;

    float *currentData = nullptr;
    auto it = spectrumData.find(wavelength);
    if (it != spectrumData.end()) {
        currentData = it->second;
    } else {
        currentData = new float[imgHei * imgWid];
        for (decltype(totalPts) i = 0; i < imgHei * imgWid; i++) {
            currentData[i] = 0;
        }
        spectrumData[wavelength] = currentData;
    }

    for (decltype(totalPts) i = 0; i < totalPts; i++) {
        int x = tmpXY[i * 2 + 0];
        int y = tmpXY[i * 2 + 1];
        if (x < 0 || x >= imgWid || y < 0 || y >= imgHei) {
            continue;
        }
        currentData[y * imgWid + x] += tmpW[i];
    }
    delete[] tmpXY;
    delete[] tmpW;
}


void EquiAreaCameraProjection::project(
        float *camRot,          // Camera rotation. [lon, lat, roll]
        float hov,              // Half field of view.
        uint64_t dataNumber,    // Data number
        float *dir,             // Ray directions, [x, y, z]
        int imgWid, int imgHei, // Image size
        int *imgXY              // Image coordinates
    )
{
    float imgR = std::fmax(imgWid, imgHei) / 2.0f;
    auto *dirCopy = new float[dataNumber * 3];
    float camRotCopy[3];
    memcpy(dirCopy, dir, sizeof(float) * 3 * dataNumber);
    memcpy(camRotCopy, camRot, sizeof(float) * 3);
    camRotCopy[0] *= -1;
    camRotCopy[1] *= -1;
    for (float &i : camRotCopy) {
        i *= Math::PI / 180.0f;
    }

    Math::rotateZ(camRotCopy, dirCopy, dataNumber);
    for (decltype(dataNumber) i = 0; i < dataNumber; i++) {
        float lon = std::atan2(dirCopy[i * 3 + 1], dirCopy[i * 3 + 0]);
        float lat = std::asin(dirCopy[i * 3 + 2] / Math::norm3(dirCopy + i * 3));
        float projR = imgR / 2.0f / std::sin(hov / 2.0f / 180.0f * Math::PI);
        float r = 2.0f * projR * std::sin((Math::PI / 2.0f - lat) / 2.0f);

        imgXY[i * 2 + 0] = static_cast<int>(r * std::cos(lon) + imgWid / 2.0f);
        imgXY[i * 2 + 1] = static_cast<int>(r * std::sin(lon) + imgHei / 2.0f);
    }

    delete[] dirCopy;
}


constexpr int SpectrumRenderer::MIN_WL;
constexpr int SpectrumRenderer::MAX_WL;
constexpr float SpectrumRenderer::_cmf_xyz_sum[];
constexpr float SpectrumRenderer::_W[];
constexpr float SpectrumRenderer::_mt[];
constexpr float SpectrumRenderer::_cmf_x[];
constexpr float SpectrumRenderer::_cmf_y[];
constexpr float SpectrumRenderer::_cmf_z[];

void SpectrumRenderer::rgb(int waveLengthNumber, float *waveLengths,
                           int dataNumber, float *specData, uint8_t *rgbData)
{
    for (int i = 0; i < dataNumber; i++) {
        /* Step 1. Spectrum to XYZ */
        float xyz[3] = { 0 };
        for (int j = 0; j < waveLengthNumber; j++) {
            // int startWl = std::max(static_cast<int>(waveLengths[j]), MIN_WL);
            // int endWl = std::min(static_cast<int>(waveLengths[j+1]), MAX_WL);
            // float startSpec = specData[j*dataNumber + i];
            // float endSpec = specData[(j+1)*dataNumber + i];

            // for (int k = startWl; k < endWl; k++) {
            //     float v = (k - startWl) * 1.0f / (endWl - startWl) * (endSpec - startSpec) + startSpec;
            //     xyz[0] += _cmf_x[k] * v;
            //     xyz[1] += _cmf_y[k] * v;
            //     xyz[2] += _cmf_z[k] * v;
            // }

            auto wl = static_cast<int>(waveLengths[j]);
            float v = wl >= MIN_WL && wl <= MAX_WL ? specData[j*dataNumber + i] : 0.0f;
            xyz[0] += _cmf_x[wl - MIN_WL] * v;
            xyz[1] += _cmf_y[wl - MIN_WL] * v;
            xyz[2] += _cmf_z[wl - MIN_WL] * v;
        }
        // for (int j = 0; j < 3; j++) {
        //     xyz[j] /= _cmf_xyz_sum[j];
        // }

        /* Step 2. XYZ to linear RGB */
        float gray[3];
        for (int j = 0; j < 3; j++) {
            gray[j] = _W[j] * xyz[1];
        }

        float r = 1.0f;
        for (int j = 0; j < 3; j++) {
            float a = 0, b = 0;
            for (int k = 0; k < 3; k++) {
                a += -gray[k] * _mt[j*3 + k];
                b += (xyz[k] - gray[k]) * _mt[j*3 + k];
            }
            if (a * b > 0 && a / b < r) {
                r = a / b;
            }
        }

        float rgb[3] = { 0 };
        for (int j = 0; j < 3; j++) {
            xyz[j] = (xyz[j] - gray[j]) * r + gray[j];
        }
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                rgb[j] += xyz[k] * _mt[j*3 + k];
            }
            rgb[j] = fmin(fmax(rgb[j], 0.0f), 1.0f);
        }

        /* Step 3. Convert linear sRGB to sRGB */
        for (int j = 0; j < 3; j++) {
            if (rgb[j] < 0.0031308) {
                rgb[j] *= 12.92f;
            } else {
                rgb[j] = static_cast<float>(1.055 * pow(rgb[j], 1.0/2.4) - 0.055);
            }
            rgbData[i * 3 + j] = static_cast<uint8_t>(rgb[j] * 255);
        }
    }
}

}   // namespace IceHalo