// The analysis run's consumer: no image, a histogram of complete raypath
// chains (doc/raypath-analysis-panel.md §3). Every outgoing ray a batch
// delivers carries a chain id (SimData::outgoing_chain_id_); this consumer
// resolves it to a session-wide id through ChainIdMerger, decides whether the
// ray's exit direction is inside the requested ROI, and if so adds the ray's
// luminance Y · weight to that chain's bucket. A snapshot is the buckets
// sorted by energy.
//
// Y comes from the same authority RenderConsumer's colour-class lanes use
// (core/color_util.hpp SpectrumToYSingle) — no normalisation of its own, so
// a chain's energy here and its pixels' Y in a render of the same batches sum
// to the same number (test_raypath_histogram_consumer.cpp pins that).
//
// The record is BOUNDED, at two levels. The producer's interning table holds
// at most ChainIdInterningTable::kDefaultCapacity chains per worker and
// answers the rest with one sentinel id; this consumer holds at most
// kRaypathHistogramCapacity rows and, when a chain it has no row for arrives
// while full, replaces its lowest-energy row (Space-Saving, Metwally et al.
// 2005): the new row starts from the evicted row's energy and count — that is
// what makes the algorithm's guarantee hold — and records that inherited
// energy as its error_bound_, so a reader knows how much of a row may belong
// to some other chain. The sentinel's rays, which have no chain to be a row
// of, go to one "other" bucket that is never a row and never evicted; an
// evicted row's content is NOT moved there — it stays in the row that took
// the slot over, as that row's error — so every ray is in exactly one row or
// in other, and Σ energy over rows + other is the energy of every counted
// ray, at every capacity. The guarantee, for a capacity of k rows over a
// total energy E: every chain with energy > E/k has a row, and every row's
// error_bound_ <= E/k. Memory is O(k) per consumer,
// the rows are found in O(1) and the eviction in O(log k) amortised (a
// min-heap over the rows with lazy re-insertion, sized k), so neither grows
// with the ray count or the MS depth.
//
// Symmetry is a READ-time choice, not a recording-time one. The run records
// every chain at its finest (the simulator interns under kSymNone), and the
// three free functions at the bottom of this header — BuildRaypathReduceContext,
// ReduceRaypathHistogram, FormatRaypathChainDisplay — turn a finest result into
// what a reader asked for: chains reduced per layer under a P/B/D bit set,
// merged into one row per canonical chain, labelled in the one display format,
// each merged row's error_bound_ the Σ of its finest rows' — so a reduced row
// standing for an orbit of m finest chains is uncertain by at most m × E/k.
// The C API calls them on every read (c_api.cpp LUMICE_FrameGetRaypathAnalysis),
// which is what lets a GUI toggle P/B/D on a finished result without re-running.
#ifndef CONSUMER_RAYPATH_HISTOGRAM_H_
#define CONSUMER_RAYPATH_HISTOGRAM_H_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/chain_id_table.hpp"
#include "core/geo3d.hpp"
#include "core/shared/projection_shared.h"
#include "server/consumer.hpp"
#include "server/server.hpp"
#include "util/logger.hpp"

namespace lumice {

// RaypathRoiSpec (the request) is declared in server/server.hpp, next to
// RaypathRoiMode: it is also half of RaypathAnalysisRequest, which Server takes.

struct SceneConfig;

// Row capacity of one consumer (k in the class comment): the bound on the
// finest record's memory and on what a read-time reduction has to walk.
// Calibrated by measurement (doc/raypath-analysis-panel.md carries the
// table): equal to the producers' ChainIdInterningTable::kDefaultCapacity,
// which keeps the 22° reference scene (11.7k finest chains at 200k rays)
// exact end to end — the two bounds are only ever both exact or both not —
// and reads back in 15 ms on the 838k-chain two-layer scene that took 1.5 s
// unbounded; 32768 would double that for no exact row gained there.
constexpr size_t kRaypathHistogramCapacity = 16384;
// "The two bounds are only ever both exact or both not" (comment above) is a claim about this
// value tracking ChainIdInterningTable::kDefaultCapacity (K_trie); pin it so a change to either
// constant alone is a compile error rather than a silently-reintroduced mismatch.
static_assert(kRaypathHistogramCapacity == ChainIdInterningTable::kDefaultCapacity,
              "kRaypathHistogramCapacity (k) must track ChainIdInterningTable::kDefaultCapacity "
              "(K_trie) — see the comment above");

class RaypathHistogramConsumer : public IConsume {
 public:
  // `reduce_ctx` is published with every snapshot (RaypathHistogramResult::
  // reduce_ctx_) for the read-time reduction; the consumer itself never reads
  // it. The server builds it from the committed scene (BuildRaypathReduceContext
  // below); a test feeding synthetic batches may leave it empty, which reduces
  // every crystal with sigma_a = 0 / D off and labels every layer as
  // single-crystal. `capacity` is the row bound (kRaypathHistogramCapacity);
  // a test that wants to see an eviction sets it small, the product never
  // passes it.
  explicit RaypathHistogramConsumer(RaypathRoiSpec roi, RaypathReduceContext reduce_ctx = {},
                                    size_t capacity = kRaypathHistogramCapacity);

  void Consume(const SimData& data) override;
  void PrepareSnapshot() override;
  Result GetResult() const override;
  void Reset() override;

  const RaypathRoiSpec& Roi() const { return roi_; }
  size_t Capacity() const { return capacity_; }

 private:
  struct Entry {
    double energy_ = 0.0;
    size_t count_ = 0;
    std::vector<double> ring_energy_;  // kCone only, sized on first touch
    double error_bound_ = 0.0;         // see RaypathHistogramEntry::error_bound_
  };
  // One ray's contribution into `e` — the one accumulate path, shared by a
  // row and by the other bucket.
  void AddToEntry(Entry& e, double y, bool cone, int ring);
  // The row for `merged_id`: found; made, while there is room; or the
  // lowest-energy row taken over (Space-Saving), its content moved to other_.
  Entry& RowFor(uint32_t merged_id);
  // The row of lowest energy_ — the heap's top once stale items (a row whose
  // energy grew since it was pushed) have been re-pushed at their current
  // value. Every row is in the heap exactly once, so the heap never exceeds
  // live_.size() items.
  uint32_t PopMinRow();

  // Decision B in the design record: the frame test is the forward
  // projection plus the two display clips, on the ray's world direction.
  bool InFrame(float wx, float wy, float wz) const;
  // Membership is the dot-product threshold; `*out_ring` (written only when
  // true) is the angular-distance ring, floor(angle / ring width) clamped to
  // the last ring.
  bool ConeMembership(float wx, float wy, float wz, int* out_ring) const;

  RaypathRoiSpec roi_;
  RaypathReduceContext reduce_ctx_;

  // kInFrame, computed once from frame_config_ the way RenderConsumer does.
  Rotation rot_;
  lm_proj::ProjParams proj_params_{};
  float forward_[3]{ 0.0f, 0.0f, 0.0f };
  // kCone.
  float cos_radius_ = 1.0f;
  float ring_width_rad_ = 0.0f;

  ChainIdMerger merger_;
  const size_t capacity_;
  std::unordered_map<uint32_t, Entry> live_;
  // Min-heap over live_ by energy_ (ties by id, so two runs evict alike):
  // (energy as pushed, id). An item whose energy no longer matches its row is
  // stale, and is re-pushed at the current value when it surfaces.
  struct HeapItem {
    double energy_;
    uint32_t id_;
    bool operator>(const HeapItem& o) const { return energy_ != o.energy_ ? energy_ > o.energy_ : id_ > o.id_; }
  };
  std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<HeapItem>> min_heap_;
  // The bucket for what no row can hold (RaypathHistogramResult::other_*):
  // the sentinel's rays. A member, not an element of live_, so it can neither
  // be evicted nor found by RowFor.
  Entry other_;
  size_t truncated_chain_count_ = 0;
  std::vector<RaypathHistogramEntry> snapshot_entries_;
  double snapshot_max_row_error_ = 0.0;

  // One-shot diagnostics: the data contract is either honoured for the whole
  // session or broken on the first batch, so repeating the line buys nothing.
  bool logged_unresolved_ = false;
  bool logged_delta_contract_ = false;

  Logger logger_{ "RaypathHistogram" };
};

// ---- Read-time reduction ----------------------------------------------------

// The scene facts a reader needs (server.hpp RaypathReduceContext), from the
// scene a run traces: per crystal id the axis-derived D parameters, by the
// same two derivations FilterSpec::Create and the simulator's chain-id layer
// context make (detail::IsDApplicable / detail::ComputeSigmaA); per layer
// whether it holds more than one crystal.
RaypathReduceContext BuildRaypathReduceContext(const SceneConfig& scene);

// The display text of one chain — THE format a user sees, in the C API's
// `display` field and in the GUI's list (which prints it verbatim):
//   one layer, single-crystal layer:   "3-5"
//   one layer, multi-crystal layer:    "C1(3-5)"
//   several layers:                    "(3-5) -> (1-3)"  /  "C1(1-3) -> C4(3-5)"
// i.e. faces joined by "-"; a layer that holds more than one crystal names
// its crystal as "C<id>" (CrystalConfig::id_); with more than one layer every
// segment is parenthesised and layers are joined by " -> ", root first. The
// crystal prefix is decided per LAYER, by layer_multi_crystal[i] for chain[i]
// (server.hpp says why it is indexed that way); a layer past the vector's end
// is treated as single-crystal, and logged once, since it means the result
// and its context disagree about the scene. The arrow is ASCII on purpose:
// the GUI's body font has no U+2192 glyph, and the text is the same bytes in
// every consumer.
std::string FormatRaypathChainDisplay(const std::vector<RaypathChainSegment>& chain,
                                      const std::vector<bool>& layer_multi_crystal);

// A finest result under `symmetry` (a FilterConfig::kSym* bit set, 0..7): every
// entry's segments are reduced per layer with that layer's crystal's D
// parameters (ReduceRaypathByPeriod, the same rule a filter canonicalises
// under), entries that meet on one reduced chain are merged — energy_, count_
// ring_energy_ and error_bound_ summed — and each merged row is labelled by
// FormatRaypathChainDisplay and sorted as the recorded result is (energy
// descending, display ascending). Σ energy_ and Σ count_ are conserved
// exactly up to summation order; the row count never grows; the record-level
// bucket and truncation count (other_energy_, other_count_,
// truncated_chain_count_) are copied through unchanged, being about no chain,
// and max_row_error_ is the max over the MERGED rows. Pure: reads
// `finest`, returns a new result, and is what every C API read of the
// histogram goes through — symmetry 0 too, so the display text is the same
// authority's at every setting.
RaypathHistogramResult ReduceRaypathHistogram(const RaypathHistogramResult& finest, uint8_t symmetry);

// ReduceRaypathHistogram over a frame's recorded result, through the frame's per-symmetry cache
// (ResultFrame::raypath_reduce_cache_, a shared_ptr<ResultFrame::RaypathReduceCache> — one memo
// per possible kSym* bitmask, 8 slots; see that struct's own comment in server.hpp for why one
// slot is not enough): the reduction for `symmetry` is computed
// once and reused on every later call with the same symmetry, without evicting any other
// symmetry's memo. Null when the frame carries no analysis result. What both C API reads of the
// histogram call, so the row count and the rows of one (frame, symmetry) pair are one reduction,
// and provably the same one.
std::shared_ptr<const RaypathHistogramResult> ReducedRaypathHistogramOf(const ResultFrame& frame, uint8_t symmetry);

}  // namespace lumice

#endif  // CONSUMER_RAYPATH_HISTOGRAM_H_
