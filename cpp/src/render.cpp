#include "optics.h"
#include "render.h"
#include "mymath.h"


namespace IceHalo {

void EquiAreaCameraProjection::project(
        float *camRot,          // Camera rotation. [lon, lat, roll]
        float hov,              // Half field of view. For diagonal.
        int dataNumber,         // Data number
        float *dir,             // Ray directions, [x, y, z]
        int imgWid, int imgHei, // Image size
        int *imgXY              // Image coordinates
    )
{
    float imgR = std::sqrt(imgWid * imgWid * 1.0f + imgHei * imgHei * 1.0f) / 2.0f;
    auto *dirCopy = new float[dataNumber * 3];
    float camRotCopy[3];
    memcpy(dirCopy, dir, sizeof(float) * 3 * dataNumber);
    memcpy(camRotCopy, camRot, sizeof(float) * 3);
    camRotCopy[2] += 180;
    for (float &i : camRotCopy) {
        i *= Math::PI / 180.0f;
    }

    Math::rotateZ(camRotCopy, dirCopy, dataNumber);
    for (int i = 0; i < dataNumber; i++) {
        float lon = std::atan2(dirCopy[i * 3 + 1], dirCopy[i * 3 + 0]);
        float lat = std::asin(dirCopy[i * 3 + 2] / Math::norm3(dirCopy + i * 3));
        float projR = imgR / 2.0f / std::sin(hov / 360.0f * Math::PI);
        float r = 2.0f * projR * std::sin((Math::PI / 2.0f - lat) / 2.0f);

        imgXY[i * 2 + 0] = static_cast<int>(r * std::cos(lon) + imgWid / 2.0f);
        imgXY[i * 2 + 1] = static_cast<int>(r * std::sin(lon) + imgHei / 2.0f);
    }

    delete[] dirCopy;
}


constexpr int SpectrumRenderer::_cmf_min_wl;
constexpr int SpectrumRenderer::_cmf_max_wl;
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
            // int startWl = std::max(static_cast<int>(waveLengths[j]), _cmf_min_wl);
            // int endWl = std::min(static_cast<int>(waveLengths[j+1]), _cmf_max_wl);
            // float startSpec = specData[j*dataNumber + i];
            // float endSpec = specData[(j+1)*dataNumber + i];

            // for (int k = startWl; k < endWl; k++) {
            //     float v = (k - startWl) * 1.0f / (endWl - startWl) * (endSpec - startSpec) + startSpec;
            //     xyz[0] += _cmf_x[k] * v;
            //     xyz[1] += _cmf_y[k] * v;
            //     xyz[2] += _cmf_z[k] * v;
            // }

            auto wl = static_cast<int>(waveLengths[j]);
            float v = wl >= _cmf_min_wl && wl <= _cmf_max_wl ? specData[j*dataNumber + i] : 0.0f;
            xyz[0] += _cmf_x[wl - _cmf_min_wl] * v;
            xyz[1] += _cmf_y[wl - _cmf_min_wl] * v;
            xyz[2] += _cmf_z[wl - _cmf_min_wl] * v;
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