#include "core/chain_id_table.hpp"

#include <utility>

namespace lumice {

size_t ChainIdInterningTable::KeyHash::operator()(const Key& k) const {
  // FNV-1a over the three fields. Only an index hint: equality is decided by
  // Key::operator==, so a collision costs a probe, never a wrong id.
  uint64_t h = 1469598103934665603ull;
  auto mix = [&h](uint64_t v) {
    h ^= v;
    h *= 1099511628211ull;
  };
  mix(k.parent_id);
  mix(k.crystal_id);
  mix(k.segment.size());
  for (auto fn : k.segment) {
    mix(fn);
  }
  return static_cast<size_t>(h);
}

ChainIdInterningTable::ChainIdInterningTable() {
  entries_.emplace_back();  // id 0 placeholder (the root has no entry)
}

uint32_t ChainIdInterningTable::Intern(uint32_t parent_id, IdType crystal_id, std::vector<IdType> segment) {
  Key key{ parent_id, crystal_id, std::move(segment) };
  auto it = index_.find(key);
  if (it != index_.end()) {
    return it->second;
  }
  auto id = static_cast<uint32_t>(entries_.size());
  ChainIdTableEntry e;
  e.id = id;
  e.parent_id = parent_id;
  e.crystal_id = crystal_id;
  e.segment = key.segment;
  entries_.push_back(std::move(e));
  index_.emplace(std::move(key), id);
  return id;
}

const ChainIdTableEntry& ChainIdInterningTable::EntryAt(uint32_t id) const {
  return entries_.at(id);
}

std::vector<ChainIdTableEntry> ChainIdInterningTable::FlushDelta() {
  std::vector<ChainIdTableEntry> out(entries_.begin() + static_cast<std::ptrdiff_t>(flush_cursor_), entries_.end());
  flush_cursor_ = entries_.size();
  return out;
}

void ChainIdInterningTable::Clear() {
  entries_.clear();
  entries_.emplace_back();
  index_.clear();
  flush_cursor_ = 1;
}

std::vector<uint32_t> ChainIdInterningTable::PathToRoot(uint32_t id) const {
  std::vector<uint32_t> path;
  for (uint32_t cur = id; cur != kRootChainId; cur = entries_.at(cur).parent_id) {
    path.push_back(cur);
  }
  return path;
}

std::vector<ChainIdTableEntry> ChainIdInterningTable::Segments(uint32_t id) const {
  const auto path = PathToRoot(id);
  std::vector<ChainIdTableEntry> out;
  out.reserve(path.size());
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    out.push_back(entries_[*it]);
  }
  return out;
}

std::string ChainIdInterningTable::Format(uint32_t id) const {
  // Collect leaf -> root, then emit root -> leaf.
  const auto path = PathToRoot(id);
  std::string out;
  for (auto it = path.rbegin(); it != path.rend(); ++it) {
    const auto& e = entries_[*it];
    if (!out.empty()) {
      out += '-';
    }
    out += "crystal";
    out += std::to_string(e.crystal_id);
    out += '(';
    for (size_t i = 0; i < e.segment.size(); i++) {
      if (i > 0) {
        out += '-';
      }
      out += std::to_string(e.segment[i]);
    }
    out += ')';
  }
  return out;
}

ChainIdMerger::AbsorbReport ChainIdMerger::Absorb(uint32_t producer_key, const std::vector<ChainIdTableEntry>& delta) {
  AbsorbReport report;
  auto& state = producers_[producer_key];
  for (const auto& e : delta) {
    uint32_t merged_parent = ChainIdInterningTable::kRootChainId;
    if (e.parent_id != ChainIdInterningTable::kRootChainId) {
      auto it = state.remap.find(e.parent_id);
      if (it == state.remap.end()) {
        report.orphaned++;
        continue;
      }
      merged_parent = it->second;
    }
    if (e.id <= state.max_local_id) {
      report.non_monotonic++;
    } else {
      state.max_local_id = e.id;
    }
    state.remap[e.id] = table_.Intern(merged_parent, e.crystal_id, e.segment);
  }
  return report;
}

uint32_t ChainIdMerger::Resolve(uint32_t producer_key, uint32_t local_id) const {
  if (local_id == ChainIdInterningTable::kRootChainId) {
    return ChainIdInterningTable::kRootChainId;
  }
  auto p = producers_.find(producer_key);
  if (p == producers_.end()) {
    return kUnresolved;
  }
  auto it = p->second.remap.find(local_id);
  return it == p->second.remap.end() ? kUnresolved : it->second;
}

void ChainIdMerger::Clear() {
  table_.Clear();
  producers_.clear();
}

}  // namespace lumice
