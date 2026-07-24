#ifndef SERVER_C_API_INTERNAL_H_
#define SERVER_C_API_INTERNAL_H_

// Internal (non-public) declarations for c_api.cpp implementation details that unit
// tests need to exercise directly. This is NOT part of the public C API surface
// (include/lumice.h): do not include it from src/gui/ or ship it to consumers.

#include <nlohmann/json.hpp>

#include "include/lumice.h"

// Serialize a LUMICE_Config struct into the core config JSON schema (the same shape
// core config/filter_config.cpp::to_json produces). Exposed here — rather than kept
// file-static in c_api.cpp — so tests can assert the emitted filter shape field by
// field, instead of only observing the LUMICE_CommitConfigStruct return code (whose
// lenient core from_json would mask emit-side field-name/condition errors).
//
// Throws std::invalid_argument if any filter has an unset/invalid LUMICE_FilterParam.type.
nlohmann::json ConfigToJson(const LUMICE_Config& c);

// Translate a single crystal's shape into the core crystal JSON schema, returning
// {"type": ..., "shape": {...}} WITHOUT the "id"/"axis" keys. This is the single
// mapping table shared by ConfigToJson (which adds id/axis around it) and
// LUMICE_GetCrystalMesh (which only needs type+shape to sample a preview mesh). Kept
// out of "id"/"axis" on purpose — those are meaningless for a stateless mesh preview;
// a dedicated test asserts they are absent so the boundary cannot silently drift.
nlohmann::json CrystalShapeToJson(const LUMICE_CrystalParam& cr);

#endif  // SERVER_C_API_INTERNAL_H_
