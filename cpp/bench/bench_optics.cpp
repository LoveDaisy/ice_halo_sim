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

    const auto* face_norm = crystal.GetTriangleNormal();
    const auto* face_vtx = crystal.GetTriangleVtx();

    for (size_t i = 0; i < n; i++) {
      auto& r = buf_in[i];
      // Direction: pointing into the crystal (negative of face normal for the assigned face)
      int fid = static_cast<int>(i % 4);  // cycle through top-face triangles
      r.d_[0] = -face_norm[fid * 3 + 0];
      r.d_[1] = -face_norm[fid * 3 + 1];
      r.d_[2] = -face_norm[fid * 3 + 2];
      // Position: first vertex of the assigned face
      r.p_[0] = face_vtx[fid * 9 + 0];
      r.p_[1] = face_vtx[fid * 9 + 1];
      r.p_[2] = face_vtx[fid * 9 + 2];
      r.w_ = 1.0f;
      r.fid_ = fid;
    }
  }
};

}  // namespace


static void BM_HitSurface(benchmark::State& state) {
  auto n = static_cast<size_t>(state.range(0));
  OpticsFixture fix(n);

  float_bf_t d_in{ fix.buf_in[0].d_, sizeof(RaySeg) };
  float_bf_t w_in{ &fix.buf_in[0].w_, sizeof(RaySeg) };
  int_bf_t fid_in{ &fix.buf_in[0].fid_, sizeof(RaySeg) };
  float_bf_t d_out{ fix.buf_out[0].d_, sizeof(RaySeg) };
  float_bf_t w_out{ &fix.buf_out[0].w_, sizeof(RaySeg) };

  for (auto _ : state) {
    HitSurface(fix.crystal, fix.refractive_index, n,  //
               d_in, w_in, fid_in,                    //
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
    int_bf_t fid_in{ &fix.buf_in[0].fid_, sizeof(RaySeg) };
    float_bf_t d_out{ fix.buf_out[0].d_, sizeof(RaySeg) };
    float_bf_t w_out{ &fix.buf_out[0].w_, sizeof(RaySeg) };
    HitSurface(fix.crystal, fix.refractive_index, n,  //
               d_in, w_in, fid_in,                    //
               d_out, w_out);
  }

  // Propagate: 2*n rays (reflection + refraction), step=2 means p_in is shared per pair.
  float_bf_t prop_d_in{ fix.buf_out[0].d_, sizeof(RaySeg) };
  float_bf_t prop_w_in{ &fix.buf_out[0].w_, sizeof(RaySeg) };
  float_bf_t prop_p_in{ fix.buf_in[0].p_, sizeof(RaySeg) };

  RayBuffer prop_out(n * 2);
  prop_out.size_ = n * 2;
  float_bf_t prop_p_out{ prop_out[0].p_, sizeof(RaySeg) };
  int_bf_t prop_fid_out{ &prop_out[0].fid_, sizeof(RaySeg) };

  for (auto _ : state) {
    Propagate(fix.crystal, n * 2, 2,            //
              prop_d_in, prop_p_in, prop_w_in,  //
              prop_p_out, prop_fid_out);
  }
  state.SetItemsProcessed(static_cast<int64_t>(state.iterations() * n * 2));
}
BENCHMARK(BM_Propagate)->Arg(32)->Arg(256)->Arg(1024);
