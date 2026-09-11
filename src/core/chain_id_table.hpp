// Interning table for cross-MS-layer raypath chains (raypath-analysis
// foundation; design in doc/raypath-analysis-panel.md §3.2).
//
// A "chain" is the complete raypath of one ray from the light source to the
// camera: one segment per MS layer it traversed, each segment being that
// layer's face sequence reduced under the crystal's symmetry. The engine does
// not carry the chain itself across layers (it is unbounded and fat); it
// carries a uint32 id and this table maps
//
//     (parent chain id, this layer's crystal id, this layer's reduced segment)
//         -> child chain id
//
// so the set of chains forms a trie whose root is id 0 ("no layer traversed
// yet") and whose every other node knows its parent. Printing a chain walks
// the parent pointers back to the root.
//
// Ids are dense: the k-th distinct key interned gets id k (k >= 1), which is
// what lets a consumer rebuild the trie incrementally from FlushDelta() — a
// delta's entries are exactly the ids in (last flushed, Size()], and every
// parent_id in it is either 0 or already delivered.
//
// One table per Simulator, i.e. per worker (server.cpp owns one Simulator per
// worker thread), and every access happens on that worker's own thread inside
// SimulateOneWavelength — nothing here is synchronised. Ids are therefore
// worker-local: two workers intern the same chain under different ids, and a
// consumer that merges deltas from several workers re-interns each delivered
// entry (parent remapped through what it already merged) into a table of its
// own. This class is that table too; there is no second implementation.
#ifndef SRC_CORE_CHAIN_ID_TABLE_H_
#define SRC_CORE_CHAIN_ID_TABLE_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/def.hpp"

namespace lumice {

struct ChainIdTableEntry {
  uint32_t id = 0;
  uint32_t parent_id = 0;
  // CrystalConfig::id_ (the user-visible crystal id), NOT the per-batch index
  // into SimData::crystals_: the former is a property of the scene and hence
  // the same on every worker and in every batch, the latter is not.
  IdType crystal_id = kInvalidId;
  // The face sequence of this layer's traversal after Crystal::ReduceRaypath.
  std::vector<IdType> segment;
};

class ChainIdInterningTable {
 public:
  static constexpr uint32_t kRootChainId = 0;

  ChainIdInterningTable();

  // Return the id of (parent_id, crystal_id, segment), assigning the next
  // dense id if the key is new. Never returns kRootChainId.
  uint32_t Intern(uint32_t parent_id, IdType crystal_id, std::vector<IdType> segment);

  // Number of interned chains, i.e. the largest id in use.
  size_t Size() const { return entries_.size() - 1; }

  // Entry for id in [1, Size()]. The root (id 0) has no entry.
  const ChainIdTableEntry& EntryAt(uint32_t id) const;

  // Entries interned since the previous FlushDelta() (or construction), in id
  // order. Calling it twice with nothing interned in between returns nothing
  // the second time.
  std::vector<ChainIdTableEntry> FlushDelta();

  // Drop every chain and rewind the flush cursor. Ids restart from 1.
  void Clear();

  // Human-readable chain, leaf first walked back to the root, e.g.
  // "crystal1(1-3-5)-crystal2(3-2)". The root itself formats as "".
  std::string Format(uint32_t id) const;

  // The same walk as Format(), structured: this chain's entries root-first,
  // one per MS layer the ray traversed, so result[i] is layer i+1 (1-indexed,
  // the order Format() prints). Empty for kRootChainId. Both walks go through
  // one private helper so the two can never disagree on the order.
  std::vector<ChainIdTableEntry> Segments(uint32_t id) const;

 private:
  // Ids from `id` back to (excluding) the root, leaf first.
  std::vector<uint32_t> PathToRoot(uint32_t id) const;

  struct Key {
    uint32_t parent_id;
    IdType crystal_id;
    std::vector<IdType> segment;
    bool operator==(const Key& o) const {
      return parent_id == o.parent_id && crystal_id == o.crystal_id && segment == o.segment;
    }
  };
  struct KeyHash {
    size_t operator()(const Key& k) const;
  };

  // entries_[id] for id >= 1; entries_[0] is an unused placeholder so the id
  // is the index and no arithmetic is needed anywhere.
  std::vector<ChainIdTableEntry> entries_;
  std::unordered_map<Key, uint32_t, KeyHash> index_;
  size_t flush_cursor_ = 1;
};

// The consumer-side merge of several producers' tables into one. Each
// Simulator (one per server worker) interns chains under its own dense ids,
// so the same chain arrives from two workers under two different local ids
// and, worse, one local id names different chains on different workers. The
// merger re-interns every delivered entry into its own table with the parent
// remapped through what it already merged from THAT producer; because
// Intern() keys on (parent, crystal_id, segment), two producers reporting one
// chain converge on one merged id.
//
// `producer_key` identifies the producer; the server tags every SimData with
// SimData::producer_effective_seed_ (the Simulator's effective seed, distinct
// per worker by construction — see the note at ServerImpl's worker loop). Two
// live producers sharing a key would silently fuse unrelated chains; the only
// cheap structural witness of that is a local id arriving out of order, which
// Absorb() reports (FlushDelta() hands ids out strictly ascending within one
// Run(), so a repeat or a step backwards means either two producers under one
// key or a producer that restarted without the consumer being Reset()).
//
// Not synchronised: the owning consumer is driven under the server's
// consumer mutex like every other IConsume.
class ChainIdMerger {
 public:
  // Resolve()'s answer for a (producer, local id) pair Absorb() never saw.
  static constexpr uint32_t kUnresolved = 0xFFFFFFFFu;

  struct AbsorbReport {
    // Entries whose parent had not been delivered by this producer before —
    // a broken delta contract; they are dropped and their local ids stay
    // unresolved.
    size_t orphaned = 0;
    // Entries whose local id was not greater than every id this producer
    // delivered so far (see the class comment). Still absorbed.
    size_t non_monotonic = 0;
  };

  // Merge one batch's delta from `producer_key`. Entries must be in the order
  // FlushDelta() emits them (ascending id, parents before children).
  AbsorbReport Absorb(uint32_t producer_key, const std::vector<ChainIdTableEntry>& delta);

  // Merged id of `local_id` as reported by `producer_key`; kRootChainId maps
  // to itself, anything never absorbed to kUnresolved.
  uint32_t Resolve(uint32_t producer_key, uint32_t local_id) const;

  const ChainIdInterningTable& Table() const { return table_; }

  // Forget every producer and every chain.
  void Clear();

 private:
  struct ProducerState {
    std::unordered_map<uint32_t, uint32_t> remap;  // local id -> merged id
    uint32_t max_local_id = ChainIdInterningTable::kRootChainId;
  };

  ChainIdInterningTable table_;
  std::unordered_map<uint32_t, ProducerState> producers_;
};

}  // namespace lumice

#endif  // SRC_CORE_CHAIN_ID_TABLE_H_
