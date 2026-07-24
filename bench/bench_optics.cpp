#include <benchmark/benchmark.h>

#include <cstddef>

#include "config/sim_data.hpp"
#include "core/buffer.hpp"
#include "core/crystal.hpp"
#include "core/optics.hpp"
#include "core/raypath.hpp"

using namespace lumice;  // NOLINT(google-build-using-namespace) benchmark code

namespace {

// Prepare a prism crystal and ray buffers with realistic memory layout (AoS via RaySeg).
struct OpticsFixture {
  Crystal crystal;
  RayBuffer buf_in;
  RayBuffer buf_out;
  size_t ray_num;
  float refractive_index;

  explicit OpticsFixture(size_t n) : ray_num(n) {
    crystal = Crystal::CreatePrism(1.3f);
    buf_in = RayBuffer(n);
    buf_out = RayBuffer(n * 2);
    buf_in.size_ = n;
    buf_out.size_ = n * 2;
    refractive_index = crystal.GetRefractiveIndex(550.0f);

    // Seed every ray onto the upper basal face (cf_geom_ slot 0 on the
    // closed-form prism). Direction points into the crystal (negative face
    // normal); position is the face's first CCW corner. The upper basal is the
    // first present slot, so its polygon-face index (RaySeg::to_face_) is 0.
    // The Crystal no longer exposes per-triangle getters — read the flat POD
    // geometry directly (the benchmark only needs one realistic entry face).
    const CrystalGeom& g = crystal.CfGeom();
    const float* basal_n = g.face_normal;  // slot 0 normal
    const float* basal_v0 = g.face_vtx;    // slot 0, corner 0

    for (size_t i = 0; i < n; i++) {
      auto& r = buf_in[i];
      r.d_[0] = -basal_n[0];
      r.d_[1] = -basal_n[1];
      r.d_[2] = -basal_n[2];
      r.p_[0] = basal_v0[0];
      r.p_[1] = basal_v0[1];
      r.p_[2] = basal_v0[2];
      r.w_ = 1.0f;
      r.to_face_ = 0;
    }
  }
};

}  // namespace


static void BM_HitSurface(benchmark::State& state) {
  auto n = static_cast<size_t>(state.range(0));
  OpticsFixture fix(n);

  float_bf_t d_in{ fix.buf_in[0].d_, sizeof(RaySeg) };
  float_bf_t w_in{ &fix.buf_in[0].w_, sizeof(RaySeg) };
  id_bf_t to_face_in{ &fix.buf_in[0].to_face_, sizeof(RaySeg) };
  float_bf_t d_out{ fix.buf_out[0].d_, sizeof(RaySeg) };
  float_bf_t w_out{ &fix.buf_out[0].w_, sizeof(RaySeg) };

  for (auto _ : state) {
    HitSurface(fix.crystal, fix.refractive_index, n,  //
               d_in, w_in, to_face_in,                //
               d_out, w_out);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * n));
}
BENCHMARK(BM_HitSurface)->Arg(32)->Arg(256)->Arg(1024);


static void BM_Propagate(benchmark::State& state) {
  auto n = static_cast<size_t>(state.range(0));
  OpticsFixture fix(n);

  // Run HitSurface once to produce realistic Propagate inputs (reflected/refracted directions).
  {
    float_bf_t d_in{ fix.buf_in[0].d_, sizeof(RaySeg) };
    float_bf_t w_in{ &fix.buf_in[0].w_, sizeof(RaySeg) };
    id_bf_t to_face_in{ &fix.buf_in[0].to_face_, sizeof(RaySeg) };
    float_bf_t d_out{ fix.buf_out[0].d_, sizeof(RaySeg) };
    float_bf_t w_out{ &fix.buf_out[0].w_, sizeof(RaySeg) };
    HitSurface(fix.crystal, fix.refractive_index, n,  //
               d_in, w_in, to_face_in,                //
               d_out, w_out);
  }

  // Propagate: 2*n rays (reflection + refraction), step=2 means p_in is shared per pair.
  float_bf_t prop_d_in{ fix.buf_out[0].d_, sizeof(RaySeg) };
  float_bf_t prop_w_in{ &fix.buf_out[0].w_, sizeof(RaySeg) };
  float_bf_t prop_p_in{ fix.buf_in[0].p_, sizeof(RaySeg) };

  RayBuffer prop_out(n * 2);
  prop_out.size_ = n * 2;
  float_bf_t prop_p_out{ prop_out[0].p_, sizeof(RaySeg) };
  id_bf_t prop_to_face_out{ &prop_out[0].to_face_, sizeof(RaySeg) };
  id_bf_t prop_from_face_in{ &fix.buf_in[0].to_face_, sizeof(RaySeg) };

  for (auto _ : state) {
    Propagate(fix.crystal, n * 2, 2,            //
              prop_d_in, prop_p_in, prop_w_in,  //
              prop_from_face_in, prop_p_out, prop_to_face_out);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * n * 2));
}
BENCHMARK(BM_Propagate)->Arg(32)->Arg(256)->Arg(1024);
