#include "context/crystal_context.h"

#include <algorithm>
#include <limits>

#include "rapidjson/document.h"
#include "rapidjson/pointer.h"

namespace icehalo {

using rapidjson::Pointer;

CrystalContext::CrystalContext(CrystalPtrU g, AxisDistribution axis) : crystal(std::move(g)), axis(axis) {}


int CrystalContext::RandomSampleFace(const float* ray_dir) const {
  int total_faces = crystal->TotalFaces();
  std::unique_ptr<float[]> face_prob_buf{ new float[total_faces] };
  const auto* face_norm = crystal->GetFaceNorm();
  const auto* face_area = crystal->GetFaceArea();

  float sum = 0;
  for (int k = 0; k < total_faces; k++) {
    face_prob_buf[k] = 0;
    if (!std::isnan(face_norm[k * 3 + 0]) && face_area[k] > 0) {
      face_prob_buf[k] = std::max(-math::Dot3(face_norm + k * 3, ray_dir) * face_area[k], 0.0f);
      sum += face_prob_buf[k];
    }
  }
  for (int k = 0; k < total_faces; k++) {
    face_prob_buf[k] /= sum;
  }

  return math::RandomSampler::SampleInt(face_prob_buf.get(), total_faces);
}

}  // namespace icehalo
