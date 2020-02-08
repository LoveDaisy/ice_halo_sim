#include "file.h"
#include "gtest/gtest.h"
#include "obj_pool.h"
#include "optics.h"

extern std::string working_dir;

namespace {

class RaySegmentSerializationTest : public ::testing::Test {
 protected:
};

TEST_F(RaySegmentSerializationTest, SingleRaySegment) {
  auto ray_seg_pool = icehalo::RaySegmentPool::GetInstance();

  float pt[] = { -1.0f, 0.3f,  0.5f,     // For r0
                 0.2f,  -0.8f, 0.1f,     // For r1
                 0.7f,  0.7f,  1.0f };   // For r2
  float dir[] = { -0.8f, 0.9f,  0.1f,    // For r0
                  -0.9f, 0.9f,  1.0f,    // For r1
                  1.0f,  -1.0f, 0.2f };  // For r2
  float w[] = { 1.0f, 0.9f, 0.1f };
  int face_id[] = { 23, 5, 37 };
  auto r0 = ray_seg_pool->GetObject(pt + 0, dir + 0, w[0], face_id[0]);
  auto r1 = ray_seg_pool->GetObject(pt + 3, dir + 3, w[1], face_id[1]);
  auto r2 = ray_seg_pool->GetObject(pt + 6, dir + 6, w[2], face_id[2]);

  r0->next_reflect = r1;
  r0->next_refract = r2;
  r1->prev = r0;
  r2->prev = r0;

  std::printf("working_dir: %s\n", working_dir.c_str());
  icehalo::File file(working_dir.c_str(), "tmp.bin");
  file.Open(icehalo::openmode::kWrite);
  r0->Serialize(file, true);
  file.Close();

  ray_seg_pool->Clear();

  file.Open(icehalo::openmode::kRead);
  auto r_test = ray_seg_pool->GetObject();
  r_test->Deserialize(file, icehalo::endian::kUnknownEndian);

  ASSERT_EQ(r_test->next_reflect, reinterpret_cast<icehalo::RaySegment*>(1));
  ASSERT_EQ(r_test->next_refract, reinterpret_cast<icehalo::RaySegment*>(2));
  ASSERT_EQ(r_test->prev, reinterpret_cast<icehalo::RaySegment*>(0xffffffffffffffff));
  ASSERT_EQ(r_test->root_ctx, reinterpret_cast<icehalo::RayInfo*>(0xffffffffffffffff));
  ASSERT_EQ(r_test->w, w[0]);
  ASSERT_EQ(r_test->pt.x(), pt[0 * 3 + 0]);
  ASSERT_EQ(r_test->pt.y(), pt[0 * 3 + 1]);
  ASSERT_EQ(r_test->pt.z(), pt[0 * 3 + 2]);
  ASSERT_EQ(r_test->dir.x(), dir[0 * 3 + 0]);
  ASSERT_EQ(r_test->dir.y(), dir[0 * 3 + 1]);
  ASSERT_EQ(r_test->dir.z(), dir[0 * 3 + 2]);
  ASSERT_EQ(r_test->face_id, face_id[0]);
  ASSERT_EQ(r_test->state, icehalo::RaySegmentState::kOnGoing);
}

}  // namespace
