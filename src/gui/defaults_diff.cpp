#include "gui/defaults_diff.hpp"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <optional>
#include <string_view>
#include <utility>

#include "gui/file_io.hpp"
#include "gui/gui_logger.hpp"
#include "gui/user_defaults.hpp"

namespace lumice::gui {

namespace {

using nlohmann::json;

bool IsExcludedRootKey(std::string_view key) {
  for (const char* excluded : kDiffEngineExcludedRootKeys) {
    if (key == excluded) {
      return true;
    }
  }
  return false;
}

// Split "renderer.lens_type" into {"renderer", "lens_type"}.
//
// Deliberately NOT nlohmann's json_pointer: that form needs '/' separators plus ~0/~1 escaping,
// so every call site would have to convert and stay correct about escaping for keys that may
// grow. Walking a token vector has no escape rules to get wrong, and the panel already speaks
// dot-paths because that is what the user reads in the file.
std::vector<std::string> SplitKeyPath(const std::string& key_path) {
  std::vector<std::string> tokens;
  std::string current;
  for (const char c : key_path) {
    if (c == '.') {
      tokens.push_back(current);
      current.clear();
    } else {
      current.push_back(c);
    }
  }
  tokens.push_back(current);
  return tokens;
}

const json* FindByPath(const json& root, const std::vector<std::string>& tokens) {
  const json* node = &root;
  for (const auto& token : tokens) {
    if (!node->is_object()) {
      return nullptr;
    }
    const auto it = node->find(token);
    if (it == node->end()) {
      return nullptr;
    }
    node = &(*it);
  }
  return node;
}

void SetByPath(json& root, const std::vector<std::string>& tokens, const json& value) {
  json* node = &root;
  std::string walked;
  for (std::size_t i = 0; i + 1 < tokens.size(); ++i) {
    walked += (i == 0 ? "" : ".") + tokens[i];
    json& child = (*node)[tokens[i]];
    if (!child.is_object()) {
      // A scalar (or a stale array) sitting where an object belongs would make the assignment
      // below throw. The override file is hand-editable, so this is reachable; replacing the
      // node is the only way to honor the write the user just asked for.
      //
      // Replacing it DISCARDS whatever was there, which is a data loss — so it goes through the
      // one degradation channel rather than happening quietly. Silent discard is the failure
      // family this scrum's invariant I3 exists to close (the GUI dropping randomization data was
      // the same shape), and a user who hand-edited this file is exactly the user who would
      // otherwise never learn their edit was overwritten.
      //
      // Only when something was actually THERE, though: operator[] default-constructs a null for
      // a key the document does not have yet, and that is the ordinary path for every first write
      // to a nested key. Reporting it would fire a data-loss notice on a document that lost
      // nothing — and a warning that cries wolf on the common case is how the real one gets
      // dismissed unread.
      if (!child.is_null()) {
        NoteUserDefaultsDowngrade("'" + walked +
                                  "' in your personal defaults file held a value where a group of "
                                  "settings was expected, so it was replaced and its contents were lost.");
      }
      child = json::object();
    }
    node = &child;
  }
  (*node)[tokens.back()] = value;
}

// Erase `tokens` from `root`, then prune every parent object the erase left empty. Returns true
// when something was actually removed.
bool ErasePath(json& root, const std::vector<std::string>& tokens) {
  if (!root.is_object()) {
    return false;
  }
  const std::string& head = tokens.front();
  const auto it = root.find(head);
  if (it == root.end()) {
    return false;
  }
  if (tokens.size() == 1) {
    root.erase(it);
    return true;
  }
  const std::vector<std::string> rest(tokens.begin() + 1, tokens.end());
  if (!ErasePath(*it, rest)) {
    return false;
  }
  if (it->is_object() && it->empty()) {
    root.erase(it);
  }
  return true;
}

// The recursive comparison itself (plan §3.2 ②). Objects recurse over the UNION of both sides'
// keys, so a key present on only one side still produces a row (with json() — i.e. null — on the
// missing side). Everything else, arrays included, is a leaf and becomes exactly one row.
void WalkDiff(const std::string& key_prefix, const json& node_a, const json& node_b,
              std::vector<DefaultDiffRow>& out_rows) {
  if (node_a.is_object() && node_b.is_object()) {
    std::vector<std::string> keys;
    for (auto it = node_a.begin(); it != node_a.end(); ++it) {
      keys.push_back(it.key());
    }
    for (auto it = node_b.begin(); it != node_b.end(); ++it) {
      if (node_a.find(it.key()) == node_a.end()) {
        keys.push_back(it.key());
      }
    }
    for (const auto& key : keys) {
      const auto a_it = node_a.find(key);
      const auto b_it = node_b.find(key);
      WalkDiff(key_prefix + "." + key, a_it != node_a.end() ? *a_it : json(), b_it != node_b.end() ? *b_it : json(),
               out_rows);
    }
    return;
  }

  DefaultDiffRow row;
  row.key_path = key_prefix;
  row.current_value = node_a;
  row.default_value = node_b;
  out_rows.push_back(std::move(row));
}

// The effective defaults a NEW document would start from: factory values with the saved sparse
// override applied. Deliberately reuses ApplyUserDefaultsOverlay — the same function
// MakeNewDocumentState() applies — so the panel can never show a "default" the New path would not
// actually produce (invariant I1's failure mode is exactly two divergent resolution paths).
GuiState EffectiveDefaultState(const json& overlay_doc) {
  GuiState defaults{};
  ApplyUserDefaultsOverlay(defaults, overlay_doc);
  return defaults;
}

// Serialize `current` into `out`. False (with `out` untouched) when the serializer threw — see the
// callers for why that must not be turned into a partial write.
bool SerializeCurrentState(const GuiState& current, json& out) {
  try {
    out = json::parse(SerializeGuiStateJson(current));
  } catch (const std::exception& e) {
    // SerializeGuiStateJson builds its document programmatically, so this is not expected; log
    // rather than abort, because a panel that cannot serialize must still not take the app down.
    GUI_LOG_WARNING("[GUI] Defaults panel: could not serialize the current state ({}); nothing was changed", e.what());
    return false;
  }
  return true;
}

// The accepted-key copy itself, given an ALREADY serialized current document. Split out so
// SaveAcceptedDefaults can fail before it opens the file: a serialization failure must leave the
// override file untouched, not rewrite it unchanged.
void ApplyAcceptedKeys(json& doc, const std::vector<std::string>& accepted_key_paths, const json& current_json) {
  for (const auto& key_path : accepted_key_paths) {
    const auto tokens = SplitKeyPath(key_path);
    if (tokens.empty() || IsExcludedRootKey(tokens.front())) {
      // Defense in depth: the panel never offers an excluded key, but this is the only place
      // that turns a string into a written key, and a namespace-4 key path in the file would
      // be read back by MakeNewDocumentState.
      continue;
    }
    if (const json* value = FindByPath(current_json, tokens)) {
      SetByPath(doc, tokens, *value);
    } else {
      ErasePath(doc, tokens);
    }
  }
}

// Shared read-modify-write for all three disk-side wrappers. `mutate` receives the EXISTING document
// (never a fresh one), so a subtree this panel does not own — "presets" above all — survives every
// write by construction rather than by three call sites each remembering to preserve it.
bool UpdateOverlayDocument(const std::function<void(json&)>& mutate) {
  const auto dir = GetActiveUserConfigDir();
  if (!dir) {
    GUI_LOG_WARNING("[GUI] User defaults: no user-config directory available; nothing was saved");
    return false;
  }
  json doc = ReadOverlayJsonIfPresent(*dir);
  if (!doc.is_object()) {
    doc = json::object();
  }
  mutate(doc);
  return WriteUserDefaultsFile(*dir, doc);
}

}  // namespace

json ReadActiveOverlayDoc() {
  const auto dir = GetActiveUserConfigDir();
  if (!dir) {
    return json::object();
  }
  return ReadOverlayJsonIfPresent(*dir);
}

std::vector<DefaultDiffRow> BuildDefaultDiffRows(const GuiState& current) {
  return BuildDefaultDiffRows(current, ReadActiveOverlayDoc());
}

std::vector<DefaultDiffRow> BuildDefaultDiffRows(const GuiState& current, const nlohmann::json& overlay_doc) {
  json current_json;
  json default_json;
  try {
    current_json = json::parse(SerializeGuiStateJson(current));
    default_json = json::parse(SerializeGuiStateJson(EffectiveDefaultState(overlay_doc)));
  } catch (const std::exception& e) {
    // SerializeGuiStateJson builds its document programmatically, so this is not expected; log
    // rather than abort, because a panel that cannot list rows must still not take the app down.
    GUI_LOG_WARNING("[GUI] Defaults panel: could not serialize state for comparison ({})", e.what());
    return {};
  }

  std::vector<DefaultDiffRow> rows;
  for (auto it = current_json.begin(); it != current_json.end(); ++it) {
    if (IsExcludedRootKey(it.key())) {
      continue;
    }
    const auto default_it = default_json.find(it.key());
    WalkDiff(it.key(), *it, default_it != default_json.end() ? *default_it : json(), rows);
  }

  for (auto& row : rows) {
    row.has_saved_override = DocHasKeyPath(overlay_doc, row.key_path);
  }

  std::sort(rows.begin(), rows.end(),
            [](const DefaultDiffRow& a, const DefaultDiffRow& b) { return a.key_path < b.key_path; });
  return rows;
}

std::string FormatDiffValue(const nlohmann::json& value) {
  if (value.is_null()) {
    // Only reachable for a key one side does not have (an optional key such as
    // sun.custom_spectrum). "null" would read as a value; this reads as what it is.
    return "(absent)";
  }
  if (value.is_string()) {
    return value.get<std::string>();
  }
  if (value.is_boolean()) {
    return value.get<bool>() ? "true" : "false";
  }
  if (value.is_number_integer()) {
    return std::to_string(value.get<long long>());
  }
  if (value.is_number_float()) {
    // %.6g keeps 0.1 as "0.1" instead of "0.100000" while still separating values that differ in
    // the 6th significant digit. Display only — see the header note on comparison being raw.
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.6g", value.get<double>());
    return buffer;
  }
  if (value.is_array()) {
    // Element-wise rather than dump(): a color triple stored as float and widened to double comes
    // back out of dump() as "[0.800000011920929, 0.20000000298023224, ...]", which is unreadable
    // and — worse — invites the reader to believe the extra digits are the setting. Recursing
    // through this same function keeps one formatting rule for a value whether it stands alone or
    // sits in an array.
    std::string out = "[";
    for (std::size_t i = 0; i < value.size(); ++i) {
      if (i != 0) {
        out += ", ";
      }
      out += FormatDiffValue(value[i]);
    }
    out += "]";
    return out;
  }
  return value.dump();
}

bool RowNeedsAdoption(const DefaultDiffRow& row) {
  return row.current_value != row.default_value;
}

bool DocHasKeyPath(const nlohmann::json& doc, const std::string& key_path) {
  return FindByPath(doc, SplitKeyPath(key_path)) != nullptr;
}

bool ApplyAcceptedDefaultsToDoc(nlohmann::json& doc, const std::vector<std::string>& accepted_key_paths,
                                const GuiState& current) {
  json current_json;
  if (!SerializeCurrentState(current, current_json)) {
    return false;
  }
  ApplyAcceptedKeys(doc, accepted_key_paths, current_json);
  return true;
}

void ApplyRevertToDoc(nlohmann::json& doc, const std::string& key_path) {
  ErasePath(doc, SplitKeyPath(key_path));
}

void ApplyResetAllToDoc(nlohmann::json& doc) {
  json preserved = json::object();
  // Namespace 2 (the preset library) is a sibling section this panel does not edit. Keeping it
  // by name — rather than deleting the GuiState keys one by one — means a future GuiState key
  // is reset without anyone having to remember to add it here.
  const auto presets = doc.find("presets");
  if (presets != doc.end()) {
    preserved["presets"] = *presets;
  }
  doc = std::move(preserved);
}

bool SaveAcceptedDefaults(const std::vector<std::string>& accepted_key_paths, const GuiState& current) {
  json current_json;
  if (!SerializeCurrentState(current, current_json)) {
    // Deliberately before UpdateOverlayDocument: a failure here must leave the file untouched
    // rather than rewrite it without the keys it was asked to adopt.
    return false;
  }
  return UpdateOverlayDocument([&](json& doc) { ApplyAcceptedKeys(doc, accepted_key_paths, current_json); });
}

bool RevertOneDefault(const std::string& key_path) {
  return UpdateOverlayDocument([&](json& doc) { ApplyRevertToDoc(doc, key_path); });
}

bool ResetAllDefaults() {
  return UpdateOverlayDocument([](json& doc) { ApplyResetAllToDoc(doc); });
}

}  // namespace lumice::gui
