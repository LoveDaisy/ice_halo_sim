#ifndef SRC_CORE_RENDER_H_
#define SRC_CORE_RENDER_H_

#include <functional>
#include <unordered_map>

#include "context/context.h"
#include "core/enum_map.h"
#include "core/simulation.h"
#include "io/file.h"


namespace icehalo {

enum class VisibleRange { kUpper, kLower, kFront, kFull };


enum class LensType {
  kLinear,
  kEqualArea,
  kEquidistant,
  kDualEqualArea,
  kDualEquidistant,
};


void EqualAreaFishEye(const float* cam_rot,                                // Camera rotation. [lon, lat, roll]
                      float hov,                                           // Half field of view.
                      size_t data_number,                                  // Data number
                      const float* dir,                                    // Ray directions, [x, y, z]
                      int img_wid, int img_hei,                            // Image size
                      int* img_xy,                                         // Image coordinates
                      VisibleRange visible_range = VisibleRange::kUpper);  // Visible range


void EquidistantFishEye(const float* cam_rot,                                // Camera rotation. [lon, lat, roll]
                        float hov,                                           // Half field of view.
                        size_t data_number,                                  // Data number
                        const float* dir,                                    // Ray directions, [x, y, z]
                        int img_wid, int img_hei,                            // Image size
                        int* img_xy,                                         // Image coordinates
                        VisibleRange visible_range = VisibleRange::kUpper);  // Visible range


void DualEqualAreaFishEye(const float* cam_rot,                                // Not used
                          float hov,                                           // Not used
                          size_t data_number,                                  // Data number
                          const float* dir,                                    // Ray directions, [x, y, z]
                          int img_wid, int img_hei,                            // Image size
                          int* img_xy,                                         // Image coordinates
                          VisibleRange visible_range = VisibleRange::kUpper);  // Not used


void DualEquidistantFishEye(const float* cam_rot,                                // Not used
                            float hov,                                           // Not used
                            size_t data_number,                                  // Data number
                            const float* dir,                                    // Ray directions, [x, y, z]
                            int img_wid, int img_hei,                            // Image size
                            int* img_xy,                                         // Image coordinates
                            VisibleRange visible_range = VisibleRange::kUpper);  // Not used


void RectLinear(const float* cam_rot,                                // Camera rotation. [lon, lat, roll]
                float hov,                                           // Half field of view.
                size_t data_number,                                  // Data number
                const float* dir,                                    // Ray directions, [x, y, z]
                int img_wid, int img_hei,                            // Image size
                int* img_xy,                                         // Image coordinates
                VisibleRange visible_range = VisibleRange::kUpper);  // Visible range


using ProjectionFunction = std::function<void(const float* cam_rot,      // Camera rotation (lon, lat, roll), in degree.
                                              float hov,                 // Half field of view, in degree
                                              size_t data_number,        // Data number
                                              const float* dir,          // Ray directions, [x, y, z]
                                              int img_wid, int img_hei,  // Image size
                                              int* img_xy,               // Image coordinates
                                              VisibleRange visible_semi_sphere)>;  // Which semi-sphere can be visible


EnumMap<LensType, ProjectionFunction>& GetProjectionFunctions();


void SrgbGamma(float* linear_rgb);

constexpr int kMinWavelength = 360;
constexpr int kMaxWaveLength = 830;


class SpectrumRenderer {
 public:
  SpectrumRenderer();

  void SetCameraContext(CameraContextPtr cam_ctx);
  void SetRenderContext(RenderContextPtr render_ctx);

  void LoadRayData(const SimpleRayData& final_ray_data);
  void LoadRayDataFiles(const std::string& data_folder);
  void ClearRayData();

  void RenderToImage();
  uint8_t* GetImageBuffer() const;

 private:
  int LoadDataFromFile(File& file);
  void GatherSpectrumData(int* wl_data_out, float* sp_data_out);

  CameraContextPtr cam_ctx_;
  RenderContextPtr render_ctx_;
  std::unique_ptr<uint8_t[]> output_image_buffer_;
  std::unordered_map<int, std::unique_ptr<float[]>> spectrum_data_;
  std::unordered_map<int, std::unique_ptr<float[]>> spectrum_data_compensation_;
  float total_w_;
};

}  // namespace icehalo


#endif  // SRC_CORE_RENDER_H_
