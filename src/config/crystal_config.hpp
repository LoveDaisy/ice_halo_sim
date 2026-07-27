#ifndef CONFIG_CRYSTAL_CONFIG_H_
#define CONFIG_CRYSTAL_CONFIG_H_

#include <memory>
#include <nlohmann/json.hpp>
#include <variant>

#include "core/crystal_kind.hpp"
#include "core/math.hpp"

namespace lumice {

//! @brief Index space for every randomizable crystal shape scalar.
//!
//! @details Slots are shared by both crystal types: a prism only owns
//!   kShapeScalarHeight + the six faces, a pyramid only owns the three
//!   pyramidal/prism heights + the six faces. Slots that do not apply to a type
//!   are simply never read for that type (CanonicalizeSyncGroups zeroes their
//!   sync group so an inapplicable declaration cannot leak into equality).
//!
//!   ⚠️ The order is deliberately chosen to be *verbatim the RNG draw order* in
//!   simulator.cpp's CrystalMaker: prism draws h_ then d_[0..5]; pyramid draws
//!   h_pyr_u_ -> h_prs_ -> h_pyr_l_ then d_[0..5]. That makes "a group's leader
//!   = its lowest-index member applicable to the crystal type" identical to "the
//!   member drawn first", so no second ordering definition is needed anywhere.
//!
//!   Note this is NOT the field declaration order of PyramidCrystalParam, and
//!   NOT the field order of the C API's LUMICE_CrystalParam
//!   (height/prism_h/upper_h/lower_h). The divergence is intentional; the C API
//!   mirror carries the same warning.
enum ShapeScalar : int {
  kShapeScalarHeight = 0,  // PrismCrystalParam::h_ — prism only
  kShapeScalarUpperH = 1,  // PyramidCrystalParam::h_pyr_u_ — pyramid only
  kShapeScalarPrismH = 2,  // PyramidCrystalParam::h_prs_ — pyramid only
  kShapeScalarLowerH = 3,  // PyramidCrystalParam::h_pyr_l_ — pyramid only
  kShapeScalarFace0 = 4,   // d_[0] — both types
  kShapeScalarFace1 = 5,
  kShapeScalarFace2 = 6,
  kShapeScalarFace3 = 7,
  kShapeScalarFace4 = 8,
  kShapeScalarFace5 = 9,
  kShapeScalarCount = 10,
};

//! @brief Shape-scalar sync groups: 0 = independent, 1..N = group id.
//!
//! @details Members of one group share a SINGLE random draw (the group's first
//!   applicable member consumes the RNG; the rest reuse its value without
//!   consuming anything). Zero-initialized = every scalar independent = the
//!   behavior before sync groups existed.
//!
//!   Deliberately a parallel array on the param structs rather than a field on
//!   Distribution: that type is shared with the orientation distributions and
//!   crosses the GPU device wire (pcg_shared.h), which is the wrong scope for a
//!   host-only shape-sampling concept.
//!
//!   Heights fold with std::abs while face distances stay signed (see
//!   CrystalMaker), so a group that mixes a height with a face distance shares
//!   the same *raw* draw but the height member consumes |v|. The mechanism does
//!   not forbid such a group; the asymmetry is documented, not validated away.

struct PrismCrystalParam {
  Distribution h_{ DistributionType::kNoRandom, 1.0f, 0.0f };  // Height, equal to c/a in HP2.0
  Distribution d_[6]{};                                        // Distance to center for prism faces
  int sync_group_[kShapeScalarCount]{};                        // Shape-scalar sync groups, 0 = independent
};

struct PyramidCrystalParam {
  Distribution h_prs_{};                                             // Prism height
  Distribution h_pyr_u_{ DistributionType::kNoRandom, 0.0f, 0.0f };  // Upper pyramidal relative height, from 0.0 to 1.0
  Distribution h_pyr_l_{ DistributionType::kNoRandom, 0.0f, 0.0f };  // Lower pyramidal relative height, from 0.0 to 1.0
  Distribution d_[6]{};                                              // Distance to center for prism faces
  int sync_group_[kShapeScalarCount]{};                              // Shape-scalar sync groups, 0 = independent
  float wedge_angle_u_ = 28.0f;  // Upper wedge angle (degrees). Default ≈ atan(√3/2 / 1.629), i.e. Miller {1,0,-1,1}
  float wedge_angle_l_ = 28.0f;  // Lower wedge angle (degrees)
};

//! @brief Rewrite sync_group_ into its canonical form. Three rules, all required:
//!   1. slots not applicable to this crystal type are zeroed;
//!   2. single-member groups are zeroed (a singleton group IS independence);
//!   3. surviving groups are renumbered 1..N by first appearance in ShapeScalar order.
//!
//! @details Canonical form is not cosmetic: config_compare.hpp's operator== is
//!   the re-simulation trigger predicate, so `[2,1,2,1,2,1]` and `[1,2,1,2,1,2]`
//!   — the same partition — must compare equal or a semantic no-op would kick
//!   off a full re-run. Applied on parse, on serialization, and inside
//!   operator== (on local copies).
void CanonicalizeSyncGroups(PrismCrystalParam& p);
void CanonicalizeSyncGroups(PyramidCrystalParam& p);

//! @brief Leader-normalize the distributions inside each sync group.
//!
//! @details The group's leader (lowest applicable ShapeScalar index, i.e. the
//!   member drawn first) owns the distribution; every other member's
//!   Distribution is overwritten with the leader's. A member that differed is
//!   overwritten anyway, but only after a LOG_WARNING — neither silent nor
//!   rejected.
//!
//!   No ordering precondition: this pass does NOT need CanonicalizeSyncGroups to
//!   have run first. Both the members it rewrites and the leaders it elects are
//!   scoped to slots this crystal type actually has, and that scoping is
//!   structural — an inapplicable slot names no field to read or donate, so it
//!   can be neither leader nor member no matter what its group number says. The
//!   other two canonical-form rules cannot move the outcome either: collapsing a
//!   singleton group turns one no-op (a lone member finds no earlier peer) into
//!   another (group 0 is skipped), and renumbering is a bijection on non-zero ids
//!   while this pass only ever compares ids for equality. Pinned by
//!   NormalizeAloneExcludesInapplicableSlotWithoutCanonicalizeFirst, which runs
//!   this pass on its own.
//!
//!   (An earlier revision of this comment claimed the opposite — that a slot
//!   rule 1 is about to zero could be elected leader and donate its distribution.
//!   It cannot: such a slot has no distribution to donate. Prefer
//!   PrepareSyncGroups anyway, because both passes are needed, not because one
//!   protects the other.)
void NormalizeSyncGroups(PrismCrystalParam& p);
void NormalizeSyncGroups(PyramidCrystalParam& p);

//! @brief Run both sync-group passes: canonicalize, then leader-normalize.
//!
//! @details The composed entry point every parse path should call, so that
//!   neither pass can be forgotten. Each answers a different question and both
//!   are required: canonical form is what makes operator== (the re-simulation
//!   trigger) see equal partitions as equal, leader normalization is what makes
//!   the group's members carry one distribution. Their relative order does not
//!   change the result (see NormalizeSyncGroups above); it is fixed here only so
//!   there is one entry point rather than a choice at every call site.
void PrepareSyncGroups(PrismCrystalParam& p);
void PrepareSyncGroups(PyramidCrystalParam& p);

//! @brief Does shape-scalar `slot` physically exist on this crystal type?
//!
//! @details The runtime query onto the ONE table that answers this — the same
//!   slot map CanonicalizeSyncGroups and NormalizeSyncGroups scope themselves
//!   by. It exists so consumers outside this TU (the C API, and through it the
//!   GUI, which may not include this header) can ask core rather than keep a
//!   private truth table that has to be edited in lockstep. Out-of-range slots
//!   answer false rather than trapping, so a caller iterating 0..N over a
//!   mismatched slot count degrades to "not applicable" instead of reading
//!   garbage.
//!
//!   O(1), no allocation, no side effects — safe to call per row per frame.
bool IsShapeScalarApplicable(CrystalKind kind, int slot);

//! @brief The JSON key under `shape.sync_group` that names shape-scalar `slot`.
//!
//! @details Returns a static string, or nullptr when the slot does not apply to
//!   `kind` (or is out of range). All six face slots share the single key
//!   "face_distance", whose value is a 6-element array — callers write it once,
//!   not once per face.
//!
//!   Same motive as IsShapeScalarApplicable: this used to be a `{key, slot}`
//!   literal table copied into three translation units (here, the C API, the GUI
//!   file IO) with nothing but a comment tying them together. Drift had no
//!   symptom — the layer that drifted would silently drop the field — so the
//!   copies are gone and every layer reads this.
const char* ShapeScalarSyncKeyName(CrystalKind kind, int slot);

using CrystalParam = std::variant<PrismCrystalParam, PyramidCrystalParam>;

// Per-param shape (de)serialization. Declared here (definitions in crystal_config.cpp)
// so consumers outside that TU — e.g. the C API mesh-preview path — can build a
// PrismCrystalParam/PyramidCrystalParam from a shape JSON via `get<>` (ADL).
void to_json(nlohmann::json& j, const PrismCrystalParam& p);
void from_json(const nlohmann::json& j, PrismCrystalParam& p);
void to_json(nlohmann::json& j, const PyramidCrystalParam& p);
void from_json(const nlohmann::json& j, PyramidCrystalParam& p);

struct CrystalConfig {
  IdType id_;
  CrystalParam param_;
  AxisDistribution axis_;
};

using CrystalConfigPtrU = std::unique_ptr<CrystalConfig>;
using CrystalConfigPtrS = std::shared_ptr<CrystalConfig>;

// convert to & from json object
void to_json(nlohmann::json& j, const CrystalConfig& c);
void from_json(const nlohmann::json& j, CrystalConfig& c);

}  // namespace lumice

#endif  // CONFIG_CRYSTAL_CONFIG_H_
