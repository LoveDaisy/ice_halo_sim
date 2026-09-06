# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Entries start at **v4.1.4**. Versions **v4.1.3 and earlier were written by hand at the time**,
before the criteria below existed; their granularity may differ from what follows, and they are
left as they were rather than rewritten to match.

## Maintaining this file

One entry per **user-perceptible** change, phrased as the effect on someone who upgrades — not
as a transcription of the commit or PR title. The conventional-commit prefix is a weak prior,
never the criterion: a `refactor:` that fixes a crash earns an entry, a `feat:` that only adds
an internal helper does not.

<details>
<summary>The full rule (what earns an entry, granularity, breaking changes, sourcing)</summary>

### What earns an entry

A change earns an entry if at least one of these is true:

- it changes what appears on screen, in a rendered image, in CLI/GUI output, or in a produced
  file (`.lmc`, exported JSON, an image);
- it changes the shape or behavior of the public C API (`src/include/lumice.h`) — a new
  function, a changed signature, an ABI-affecting struct change, a semantic change to an
  existing field;
- it changes what a config file (JSON) accepts, means, or defaults to;
- it changes what ships in a release artifact (binary linking, packaging, signing) in a way a
  user could notice — a crash that no longer happens counts, a CI speedup does not;
- it fixes a user-facing bug (crash, wrong pixels, a wrong number reported to the user), even
  if the underlying commit is titled `refactor:` / `build:` / `test:`.

**Excluded by default**: internal refactors with no observable behavior change, test-suite
changes, CI pipeline changes, documentation-only changes, code comments, dependency bumps with
no behavior change, and development/process tooling. The exclusion is a judgement that has to
be *made*, not a gap: when a title does not itself say whether the change crosses one of the
lines above — every title without a `feat`/`fix` prefix, and any ambiguous `feat`/`fix` title —
**read the diff before deciding**.

### Granularity

Default is one PR, one entry. **Merge** several PRs into one entry when they are steps of the
same user-visible change (a feature PR plus its same-cycle follow-up fixes, where the follow-up
has no independent user effect). **Split** one PR into several entries when it bundles
unrelated user-visible changes. The test is the same one as above: does splitting or merging
change what a reader needs to know about one perceptible behavior?

### Breaking changes

A change to a default value, to a config-file semantic, or to the C API's ABI or behavior gets
its own `### ⚠️ Breaking Changes` subsection inside that version, and states three things:
what a user saw before, what they see now, and what they need to do about it.

### Sourcing

`gh release view <tag> --json body` lists a version's PRs but is **not authoritative on its
own** — it has silently missed a merged PR at least once (v4.1.7 omits PR #24). Cross-check
against git:

```bash
git log <prev_tag>..<tag> --oneline --grep='^Merge pull request #'   # PRs merged via merge commit
git log <prev_tag>..<tag> --first-parent --no-merges                 # squashed PRs and direct commits
```

The second command is not redundant: dependabot bumps and admin-merged single-commit PRs land
with no merge commit to grep for, and direct-to-main commits appear in no PR list at all.

</details>

## [Unreleased]

### Added
- **`grid.outline` now draws**: the render config's celestial-outline flag draws a line
  along the horizon (altitude 0) in CLI/core renders, where for four years it was parsed,
  validated, serialized and then ignored. The line is placed from the same per-pixel inverse
  projection the render-domain mask is built from — not a second copy of that math — and is
  clipped to the hemisphere `visible` admits, so a `visible: upper` render shows the horizon as
  the edge of its sky. Its width follows the local degrees-per-pixel (the preview shader's own
  rule), so it stays a couple of pixels across the whole lens/FOV range, and its colour is the
  GUI overlay's horizon red blended in linear RGB before the sRGB transfer curve. **The default
  changed to off** — see Changed below.
- **GUI background colour reaches the picture**: the preview, the three PNG exports
  (screenshot / dual-fisheye / equirectangular) and the frame baked into a saved `.lmc` now all
  paint the configured background colour behind the halo. Previously only the CLI did, so the GUI
  colour picker moved and nothing on screen changed. The colour is composited additively in linear
  RGB before the sRGB transfer curve, which makes a pixel carrying no halo energy render as exactly
  the sRGB triple the picker showed; it is painted only where the lens actually images sky, so the
  black surround outside a fisheye's image circle stays black. Expect halo-against-sky contrast to
  drop against a bright background — that is what the sRGB curve does, and EV is its control.
- **Raypath-colour (composite) display honours the background colour**: with raypath
  colouring on, the picture now carries the same background as with it off. Previously the
  composite was baked server-side with a black surround, so toggling colouring changed the
  background out from under the user — the visible inconsistency this closes. The colour is added
  in linear RGB after all exposure handling and before the sRGB transfer curve, only on the pixels
  the lens actually images, so it agrees byte-for-byte with the mono path outside the halo and
  leaves the region outside a fisheye's image circle black.
- **One new C API setter** (ABI addition, non-breaking):
  `LUMICE_SetCompositeBackground(server, background_linear)` — a display-time push of the
  composite path's additive linear-RGB background, shaped exactly like `LUMICE_SetCompositeExposure`
  (no epoch bump, no accumulator reset, no re-simulation; the next acquired result frame re-bakes
  the composite). All-zero, the default, is an algebraic no-op, so a consumer that never calls it
  sees byte-identical composites.
- **One new C API pure function** (ABI addition, non-breaking):
  `LUMICE_XyzToSrgbUint8WithBackground(xyz_in, out, pixel_count, intensity_scale, background_linear)`
  — the existing `LUMICE_XyzToSrgbUint8` with an additive linear-RGB background composited before
  the final clamp and gamma, for a consumer baking a frame that has to match what the renderer put
  on screen. The inverse sRGB transfer curve a caller needs to convert a picker colour into that
  `background_linear` argument (or into `LUMICE_RenderParam::background`) stays a C++-only inline
  function, `lumice::SrgbToLinearRgb` in `src/util/color_space.hpp` — no new public C API for it.
- **`ev_mode`** -- a first-class choice between the two exposure anchors, reaching core
  `RenderConfig`, the C API, `.lmc` documents and CLI JSON. `"relative"` (the DEFAULT) anchors
  the image to its own P99, which is what the GUI preview has always displayed: the picture
  keeps its look as `ray_num` grows, and correspondingly the config alone does not determine
  output brightness. `"absolute"` anchors to the energy the light source EMITTED, so two renders
  at the same EV are comparable -- the behavior the entry below introduced unconditionally.
  A config with no `ev_mode` key, and one with a misspelled value, both render `relative`.
  **This replaces that entry's default rather than adding to it**: the CLI was unconditionally
  absolute for one release cycle, so an existing config re-rendered now switches to the P99
  self-anchor unless it states `"ev_mode": "absolute"`. Which anchor an image was made with is
  no longer inferable from the tool version; it is in the document.
  On the composite (raypath-colour) path the same field selects between the participating-pixel
  self-anchor and the mono path's absolute scale -- the very same scalar, shared rather than
  re-derived, so mono and composite stay comparable at one EV.
  **ABI**: `LUMICE_RenderParam` gains a trailing `int ev_mode` (`LUMICE_EV_MODE_RELATIVE` = 0,
  `LUMICE_EV_MODE_ABSOLUTE` = 1) and `LUMICE_API_VERSION` moves 415 -> 416. The field is
  APPENDED, so every existing field keeps its offset, but `sizeof` grows: a caller that was not
  recompiled hands the API a shorter struct. Recompile against the new header. RELATIVE == 0
  keeps the documented default reachable from a zero-initialized struct.
- **`LUMICE_RawXyzResult.emitted_energy`** (C API): the total spectral energy the light
  source emitted into a snapshot -- the quantity the renderer now normalizes by. Raw total,
  unlike the neighbouring `snapshot_intensity`, which is a per-pixel figure; and a different
  measurement, not a rescaling of it, since one counts what went in and the other what
  landed. A consumer reproduces the renderer's own scale as
  `intensity_factor * kNormScale * total_pixels / emitted_energy`. The field occupies
  alignment padding that already existed before `epoch`, so `sizeof(LUMICE_RawXyzResult)`
  stays 64 bytes and every existing field keeps its offset -- a caller compiled against the
  old header is unaffected, and one recompiled against the new one gains a field without
  relinking anything else.

### Changed
- **`render[].grid.outline` now defaults to `false`**. It defaulted to `true` for as long
  as it existed, which cost nothing while nothing drew it; now that it draws, leaving it on would
  put a horizon line into every existing config that never asked for one. Add `"horizon": true` to
  a renderer to get the line back. Turning an annotation on for every render is a product decision
  nobody has made, so the default states the one thing that is certain: draw it when asked.
- **`render[].grid.central` / `grid.elevation` documented as not rendered**. Both keys are
  still parsed, validated and round-tripped, and no code draws either — they are now labelled that
  way in `doc/configuration.md` (and `_zh`) instead of sitting in the same table as the keys that
  do something, and the shipped `examples/config_example.json` no longer demonstrates a 22 deg
  circle that never appears in the output.
- **Breaking behavior change (CLI image brightness): display normalization is now absolute.**
  The renderer divides by the energy the light source EMITTED, where it used to divide by the
  energy that LANDED on a pixel. The old denominator moved with the scene -- add a filter, or
  point a narrower lens at the sky, and the image was silently re-brightened by exactly the
  amount that had been removed, so two renders at the same EV could not be compared. The new
  one is fixed by the source and the ray budget alone.
  **Re-running an existing config produces a darker image**, by exactly the fraction of
  emitted energy that reached the frame: negligible for a full-sphere view (~0.98, under 0.03
  stop), around 0.4-0.6 for a 90-120 degree lens (~1 stop), and as low as 0.18 for a narrow
  lens behind a filter (~2.5 stop). Raise EV to taste; the darkening is the change working,
  not a regression. Cross-lens comparability is a separate matter and is not claimed here:
  two projections still differ by a per-projection solid-angle constant.
  The GUI is unaffected -- its display path normalizes through its own auto-EV anchor and
  never used this scale. `kNormScale` is unchanged at 0.08: it was calibrated on full-sphere
  views, which is where the two denominators nearly coincide.
- **Breaking behavior change (CLI image brightness): display normalization now supports an
  absolute anchor, selected by the `ev_mode` entry above (default remains `relative`, i.e.
  unchanged pixels for an existing config).** The renderer divides by the energy the light
  source EMITTED, where it used to divide by the energy that LANDED on a pixel. The old denominator
  moved with the scene -- add a filter, or point a narrower lens at the sky, and the image
  was silently re-brightened by exactly the amount that had been removed, so two renders at
  the same EV could not be compared. The new one is fixed by the source and the ray budget
  alone.
  **Re-running an existing config in `"absolute"` mode produces a darker image**, by exactly
  `landed_fraction` (energy landed / energy emitted) -- a per-scene constant, independent of
  `ray_num`. The general law: `new_scale / old_scale ≡ landed_fraction`, and a caller can
  compute their own scene's shift from two `LUMICE_RawXyzResult` fields
  (`snapshot_intensity`, `emitted_energy`) without re-deriving anything. Measured spans across
  this feature's calibration corpus, in stops (`log2(landed_fraction)`):
  full-sphere view, no filter: **-0.038 .. -0.002**; narrow lens (90-120 degree), no filter:
  **-0.67 .. -1.25**; filter active / high `ms_prob`: **-0.97 .. -10.47**. The darkening is the
  change working, not a regression -- raise EV to taste. Cross-lens comparability is a
  separate matter and is not claimed here, by design: two projections still differ by a
  per-projection solid-angle constant, and fixing that would re-weight every pixel by its own
  solid angle and change each image's appearance, which is out of scope (see
  `doc/ev-pipeline-architecture.md` §7.3).
  The GUI is unaffected in either mode's pixels for an existing document -- see the `ev_mode`
  entry above for what changed in the GUI (a Mode control, not a pixel default). `kNormScale`
  is unchanged at 0.08: it was calibrated on full-sphere views, which is where the two
  denominators nearly coincide (`landed_fraction` median 0.980; re-deriving it would move
  full-sphere scenes by only +0.029 stop, at the cost of re-shooting every reference image).
  For an illuminant spectrum the emitted energy is charged at the band expectation of the
  SPD rather than at the weight of the wavelength each batch happens to draw, so the same
  config renders at the same brightness at every seed.
  Separately, an undersampled scene darkens as `ray_num` grows under **either** denominator
  (measured N-scaling slope: -1.026 landed-weight, -1.027 emitted-energy) -- an honest
  Monte-Carlo estimator property, not something this change introduces or fixes. See
  `doc/ev-pipeline-architecture.md` §7.4 before reporting it as a regression.

### Removed
- **Breaking ABI #4**: `LUMICE_RenderParam::opacity` removed, and `render[].opacity` is no longer
  parsed from JSON (`RenderConfig::opacity_` deleted). The field had no drawing consumer anywhere
  in the tree since the first commit — setting it changed nothing — and it has no counterpart in
  the GUI, so nothing was ever going to grow into it. `LUMICE_API_VERSION` is bumped to 416.
  Drop the assignment from C/FFI callers; old JSON configs keep loading (unknown keys are
  ignored). Note this is a DIFFERENT field from `LUMICE_GridLine::opacity`, which is untouched.
  The GUI's mirror of the same field (`renderer.opacity`, editable in the Settings panel and
  persisted in `.lmc`) is removed with it; a `.lmc` written by an older build still opens.

## [4.4.3] - 2026-08-30

### Added
- **The background photo can be panned and zoomed** (#275). A reference photograph no longer has
  to be pre-cropped to match the simulated frame: three new document fields drive an offset and a
  scale, reachable both as sliders and as a direct gesture on the canvas (Alt/Option + drag to
  pan, Alt/Option + wheel to zoom — the same modifier on every platform). The transform is folded
  into the image's UV mapping, so it costs nothing at render time and round-trips through `.lmc`.
- **Crystals can be excluded from a run without deleting them** (#276). Each crystal card gains a
  participation toggle, which is what people were doing by hand by dragging a population's weight
  to zero — a lossy workaround, since it destroyed the weight they wanted back. The flag
  round-trips through `.lmc` and is carried by Duplicate.
- **Crystals can be given a name** (#282). The crystal card's identity is now an editable name
  rather than a position in the list, so a scene with several populations stays readable after
  they are reordered, and the Colors panel refers to them by that name.
- `LUMICE_IsDApplicable` (C API, ABI addition): asks whether a crystal's axis distribution admits
  the D symmetry short-cut, from the same core predicate the simulator uses. Added so the GUI's
  hint and the engine's behaviour cannot drift apart; see the roll fix below (#278).

### Changed
- **The GUI has a designed visual language** (#271, #279). It shipped with Dear ImGui's stock dark
  theme and the built-in 13-pixel bitmap font, neither of which anyone had chosen. There is now a
  proportional font (Roboto Medium, embedded at build time — no system font dependency), a
  deliberate palette, a quantized spacing and corner-radius rhythm, and three semantic colours
  (good / warning / destructive) used consistently instead of per-call-site literals. This changes
  the look of every panel; nothing about what the controls do changes.
- **The Overlay auxiliary-line controls are a table** (#277, #280). The stack of per-line control
  groups became six aligned columns with a per-row expander for the angular-distance settings, so
  the same properties on different lines read across rather than having to be found in each group.
  Drag fields show a resize cursor while they are being dragged.
- **The row-end label column is aligned** (#281). Labels that follow a control on the same row were
  each spaced by their own call site, so they started at slightly different x positions down a
  panel. They now share one owner and one gap value.
- **Windows release binaries link the C runtime statically** (#269). `CMP0091` was declared but not
  in effect, so the published Windows build linked the MSVC runtime dynamically and failed to start
  on a machine without the matching redistributable installed. It is now genuinely static, as the
  release was always meant to be.

### ⚠️ Breaking Changes
- **A config that omits a required field is now rejected instead of quietly filled in** (#265).
  Several loader paths used to record an error and then carry on with a value they picked for you:
  a crystal missing `id` or `type`, a colour class missing `color`. These now reject the offending
  entry rather than guessing. Two other cases still load, but say out loud what was chosen —
  `light_source.spectrum` and `lens.type` when absent, and a shape distribution object missing
  `type`.
  **What to do**: a config that loaded before and now reports a rejected crystal or colour class
  was already not describing what you meant — the missing key is named in the message; add it.
  Colour-class `z_order` is now kept compact at the source, so a class dropped for a missing colour
  no longer leaves a hole in the ordering.
- **Exporting over an existing config file asks first** (#265). Export used to overwrite silently;
  it now names the file and waits for a confirmation.
  **What to do**: nothing, unless you script the GUI's export — an unattended run that relied on
  the silent overwrite will now stop at the prompt.

### Fixed
- **A full-sphere render ignored the crystal's roll** (#278). The full-sphere fast path took a
  rotational-symmetry short-cut that is only valid when the roll distribution really is a uniform
  full turn, but the condition it tested looked only at the axis direction and was structurally
  blind to roll — so a scene with a constrained roll silently rendered as if it were unconstrained.
  The condition now covers roll, and it lives in core as the single owner both the engine and the
  GUI hint read (`LUMICE_IsDApplicable`, above).
  Related, and the reason it went unnoticed: a non-linear slider dragged to its end point landed a
  hair short of the exact boundary, so "uniform over a full turn" was true at one pixel of travel
  and false at the next, switching sampling paths with nothing on screen to say so. Slider end
  points now snap to the exact boundary.
- **A scaled-down background photo aliased** (#274). The photo was uploaded without mipmaps and
  sampled bilinearly, so shrinking it to fit the viewport undersampled the image and produced
  crawling detail. It now uses mipmaps with trilinear filtering.

## [4.4.2] - 2026-08-11

### Added
- **Sampling density in the status bar** (#237). The status bar now reports how many distinct
  crystal geometries and how many orientations a run has drawn, next to the ray count — the two
  numbers that say whether a randomized scene is actually being sampled or is repeating one draw.
- **`LUMICE_GetDrainStatus`** (C API, ABI addition) (#251). Reports whether the consumer has
  drained everything the simulation produced, so a caller can tell "the run finished" from "the
  run finished and every batch has reached you". The GUI uses it to decide when a preview may
  self-pause; without it, a completed run could sit with unconsumed batches while the display
  called itself done.

### Changed
- **A `.lmc` load that fails leaves the open document alone** (#262). A failed load used to leave
  the application holding a partly-replaced document.
- **A blank filter row states nothing, rather than matching everything** (#262). An OR row with no
  raypath entered was lowered into a match-all clause, which made the whole filter match every ray
  — the opposite of what an empty row reads as. Blank rows are now dropped when the filter is
  built. Note the two are genuinely different: an empty raypath *array* in a core config still
  means "every ray", and that is unchanged.
- **The GUI's diagnostics reach the log before the window does** (#252). The GLFW / OpenGL / font
  initialisation messages are emitted through the logger, and the log sinks are installed before
  those stages run — so a launch that dies during startup now leaves its reason in the log panel
  and in the file sink instead of on a console that the Windows build has already closed.
- **`LUMICE_IsLegalFace`, `LUMICE_IsShapeScalarApplicable` and `LUMICE_ShapeScalarSyncKeyName`
  reject an unrecognised crystal kind** (#252) instead of treating it as `LUMICE_CRYSTAL_PYRAMID`.
  They answer their own negative (`0`, `0`, `NULL`). A caller passing one of the two documented
  enumerators is unaffected.

### ⚠️ Breaking Changes
- **The result-reading API is now an acquire/release handle** (#250). The six getters that returned
  borrowed pointers into the server's mutable cache — `LUMICE_GetRenderResults`,
  `LUMICE_GetRawXyzResults`, `LUMICE_GetCompositeResults`, `LUMICE_GetRawXyzAndCompositeResults`,
  `LUMICE_GetStatsResults`, `LUMICE_GetCachedStats` — are **removed**. `LUMICE_API_VERSION` moves
  414 → 415.
  **Before**: you called a getter, got a pointer, and had to finish reading it before the next
  snapshot overwrote the buffer underneath you — a contract nothing enforced, and which was
  violated twice in this codebase's own history, the second time confirmed by AddressSanitizer as a
  use-after-free.
  **Now**: `LUMICE_AcquireResultFrame` gives you a reference-counted, immutable frame; you read it
  through `LUMICE_FrameGetRender` / `_FrameGetRawXyz` / `_FrameGetComposite` / `_FrameGetStats`, and
  return it with `LUMICE_ReleaseResultFrame`. The frame stays valid for as long as you hold it, and
  a later snapshot publishes a new frame rather than rewriting yours.
  **What to do**: wrap each former getter call in an acquire/release pair and read through the
  corresponding `LUMICE_FrameGet*`. Release every frame you acquire — a held frame keeps its buffers
  alive.
- **`scene.scattering[].prob` is now required** (#257). It used to default silently. A layer that
  omits it is rejected at parse time.
  **What to do**: write the key. A single-layer scene that relied on the default should state
  `"prob": 1.0`.
- **An object-shaped axis slot must state its `type`** (#257). `{"mean": ..., "std": ...}` with no
  `type` used to be filled in with a per-slot default, differently for `zenith`, `azimuth` and
  `roll`. It is now rejected.
  **What to do**: add `"type": "gauss"` (or `uniform`, `zigzag`, `laplacian`) to the slot. A bare
  number (`"zenith": 30`) is still a fixed value and is unaffected.
  Documented alongside it, unchanged in behaviour but previously unwritten: an *absent* `axis`
  object means one fixed orientation, whereas an `axis` that omits `azimuth` / `roll` means those
  rotate freely over the full turn. The two "missing" cases are not interchangeable.

### Fixed
- **A finite low-ray simulation could finish without ever showing a picture** (#240). The preview's
  quality gate held frames back until enough rays had accumulated; a run whose whole budget was
  below that bar reached COMPLETED with the gate never satisfied, so the preview stayed empty for a
  run that had in fact finished. The final frame of a completed generation now goes to screen
  regardless.
- **The status bar showed the previous run's statistics after a restart** (#244). The server's
  cached statistics survived Stop, so a freshly restarted run displayed the old numbers — plausible
  ones, which is why it was hard to notice. Statistics now carry the generation they belong to and
  are ignored when that generation is stale.
- **The preview could show old pixels stamped with the new generation** (#246). During a restart
  window, a texture payload could be materialised from content produced before the restart and then
  labelled with the current generation. The payload is now materialised only once its content is
  known to belong to the generation it claims.
- **Revert discarded view changes it did not count as changes** (#254). Revert and the predicate
  that decides what counts as a modification were working from two different field lists, so Revert
  threw away renderer edits that had never made the document look modified. Both now read one list.
- **Preview drag moved by a fixed number of pixels regardless of lens** (#258). Dragging the
  preview panned by a constant, so the same gesture moved a narrow linear view by a sliver and a
  full-sky fisheye across the horizon. The gain now follows the lens's angular resolution at the
  cursor.
- **A settings slider did not commit when the drag ended off the control** (#262).
- **A core config with a scattering layer missing `proportion` loaded at ~1% of its intended
  share** (#255). The GUI filled the absent key with `1.0` while core's own parser seeds `100.0` on
  the same 0–100 scale, so a layer that should have been an equal partner was loaded at a hundredth
  of its weight. (Since #257 the key is required outright; this fixes what a file written before
  that loads as.)
- **Closed-form pyramid crystals could be structurally invalid** (#256, #260). Introduced in 4.4.0,
  the closed-form pyramid mis-assigned faces at a collapsed apex (every cone face was recorded at
  the apex point, whether or not its plane passed through it), and could lose its basal face or
  silently collapse near the apex; a `prism_h == 0` solid was not capped. Its tolerances also
  carried a unit-of-measure assumption, so the same shape scaled up or down was judged differently.
  Tolerances are now purely scale-relative, and vertex merging has a lateral ruler rather than a
  single absolute one.

## [4.4.1] - 2026-07-29

### Added
- **Shape-scalar sync groups** (#228). Shape scalars can be declared to share one random draw per
  crystal instance, so a randomized crystal can stay symmetric — the six `face_distance` values
  drawing together rather than independently, for example, which is the difference between a
  randomly sized hexagon and a randomly lopsided one. Declared per crystal in the config and
  editable in the crystal modal's property table.
- **A user-level defaults layer for the GUI** (#231, #234). Personal preferences — a favourite
  lens, a retuned axis preset — can be saved to a `user_defaults.json` in the OS user-config
  directory and are then applied to every new document on that machine, without turning one
  person's habit into everyone's factory default. It is a **GUI-only file**: it is never read by
  the CLI and never sits next to a scene config. The `Settings` panel edits it as a table of keys
  with their current, origin and source values, alongside an editable axis-preset library, and
  writes the file exactly once, when Save is pressed — closing any other way discards the edits.
  Two new command-line switches control it: `--user-config <dir>` points at a specific directory,
  `--no-user-config` ignores the layer entirely.
- **Orientation count reported as its own statistic** (#232). See the ABI note below.
- **Five C API queries for the crystal shape/axis JSON schema** (ABI additions) (#228, #230):
  `LUMICE_ShapeScalarSyncKeyName`, `LUMICE_ShapeWedgeAngleKeyName`, `LUMICE_ShapeIndicesKeyName`,
  `LUMICE_AxisScalarKeyName` and `LUMICE_IsShapeScalarApplicable`. The key spellings and the
  which-scalar-applies-to-which-kind table had been written out once in core and again in the GUI;
  they are now asked for rather than repeated, so the two cannot drift.

### Changed
- **GUI startup and OpenGL diagnostics go through the logger** (#227). Messages that used to be
  written straight to stderr now reach the GUI's log panel and its file sink like everything else.
  On Windows this is the difference between visible and invisible: the GUI closes its console at
  startup, so those messages previously went nowhere at all.

### ⚠️ Breaking Changes
- **`LUMICE_Config` is removed; configuration goes through the opaque `LUMICE_Scene`** (#224).
  `LUMICE_API_VERSION` moves 410 → 414 over this release.
  **Removed**: the `LUMICE_Config` value struct itself, `LUMICE_ParseConfigString`,
  `LUMICE_ParseConfigFile`, `LUMICE_ConfigToJson`, `LUMICE_CommitConfig`,
  `LUMICE_CommitConfigFromFile`, `LUMICE_CommitConfigStruct`, `LUMICE_ConfigCreateColorClasses`,
  `LUMICE_ConfigReleaseColorClasses`, `LUMICE_ConfigReleaseCompositions`, and the C++ RAII header
  `lumice_config_scope.hpp`. There is no shim and no alias.
  **Before**: a ~118 KB value struct passed by value, with inline fixed-capacity arrays that put a
  hard ABI ceiling on how many crystals or filter clauses a scene could hold, and hand-managed
  side allocations for colour classes and filter compositions.
  **Now**: build a scene with `LUMICE_SceneCreate` and the `LUMICE_SceneAdd*` / `LUMICE_SceneSet*`
  calls (or read one with `LUMICE_SceneFromJson` / `_FromJsonFile`), then hand it to
  `LUMICE_CommitScene`. `LUMICE_SceneToJson` serialises it back; `LUMICE_SceneClone` and
  `LUMICE_SceneDestroy` manage its lifetime. Capacity ceilings are enforced by the build API rather
  than baked into a struct layout.
  **What to do**: replace `LUMICE_CommitConfig(server, json)` with
  `LUMICE_SceneFromJson` + `LUMICE_CommitScene` + `LUMICE_SceneDestroy`; replace struct-filling code
  with the corresponding `LUMICE_SceneAdd*` calls. The colour-class and composition release calls
  disappear with the struct that owned them.
- **`LUMICE_CrystalParam` gains a trailing `int sync_group[LUMICE_SHAPE_SCALAR_COUNT]`** (#228),
  alongside ten new `LUMICE_SHAPE_SCALAR_*` index constants. This is an append — every existing
  field keeps its offset — but `sizeof` grows, so a caller that was not recompiled hands the API a
  shorter struct. Recompile against the new header. All-zero means "no grouping", the previous
  behaviour.
- **`LUMICE_StatsResult` gains a trailing `orientation_num`** (#232), the count of distinct crystal
  orientations sampled. `sizeof` grows from 24 to 32 bytes, so a caller passing an array of the old
  struct to `LUMICE_GetStatsResults` would have each element written past its end. Recompile against
  the new header.
  Why it is a separate number and not a rescaling of `crystal_num`: shape and axis are drawn
  independently, and on the commonest halo setup — a fixed shape under a random axis — the geometry
  count is 1 no matter how richly the orientation was sampled. Expect it to be far larger than
  `crystal_num`, and do not compare it across backends: the CPU and GPU routes sample at different
  densities, which is exactly what the number exposes.
- **`crystal_num` / `Stats: crystals=N` redefined** (#229; no ABI change — same field name and
  type): the value is now **how many distinct crystal geometries the run actually sampled**, not
  how many crystal objects it built. A scene with no random shape distributions reports exactly its
  (scattering layer × entry) count regardless of `ray_num`; give a shape a distribution and the
  count rises with the geometries actually drawn. Previously the value tracked the batch schedule
  instead of the scene — sweeping the dispatch grain alone moved it by two orders of magnitude, and
  a randomized scene reported the same number as its fixed-shape twin.
  **Expect the reported number to drop sharply** for fixed-shape scenes (e.g. 785 → 5 on a
  5-population 20k-ray scene, or 60 → 5 on the default multi-worker CLI); that is the fix, not a
  regression. The value is now independent of `num_workers` and of the dispatch grain — the
  fixed-shape part of a scene is counted once from the committed config rather than once per worker
  per batch — but remains non-comparable across backends (CPU samples per ray-group, the GPU
  K-shape clock is off by default). See `doc/c_api.md` and the contract block on
  `TraceBackend::GetLastBatchStochasticCrystalSampleCount`.
  **What to do**: nothing in code — the field name and type are unchanged. Do not compare a
  `crystals=N` recorded before this release with one recorded after; they count different things.

## [4.4.0] - 2026-07-25

### Added
- **Crystal shape randomization, end to end** (#221, #223). Every shape scalar — `height`,
  `prism_h`, `upper_h`, `lower_h` and each of the six `face_distance` values — can now carry a
  distribution instead of a single number, so a population can vary in size and proportion the way
  real crystals do rather than being 10 000 copies of one solid. The crystal modal presents all of
  them in one uniform property table (value, distribution type, spread), so the fields read the
  same way whichever one is being randomized.
- **A GPU K-shape pool, opt in through `scene.geom_clock`** (#213, #217, #218). The GPU backends
  built one crystal shape per batch, which caps how many distinct geometries a randomized scene can
  reach. `geom_clock` sets how many rays share one sampled shape: `0` (the default) keeps today's
  behaviour, and a positive K builds a pool of shapes per batch instead. Accepted in the config as
  `scene.geom_clock`, and on the C API as a field on the same record. Out-of-range values are
  rejected at parse time rather than clamped, so a sweep cannot quietly run at a K it was not
  asked for. Affects the Metal and CUDA backends only.
- `LUMICE_API_VERSION` (C API): the header now states its own ABI version, encoded as
  `major*100 + minor`, so an integration can `static_assert` against it rather than inferring
  compatibility from a release number.

### Changed
- **Crystal geometry is computed in closed form** (#214, #215, #216). The geometry pipeline used to
  derive planes, reconstruct vertices, group them into faces, triangulate, and then recover the
  face grouping and face numbering back out of the triangles — rebuilding, with numeric tolerances,
  information it had known exactly one step earlier. Each of those rebuilds needed a tolerance, and
  each tolerance had produced its own defect (missing faces on extreme wedges, wrong face numbers,
  degenerate solids). The hexagonal crystal family is now generated directly from its planes as a
  closed-form representation. Same crystals, same config; what changes is that a shape near a
  degenerate boundary is now decided by an exact construction rather than by whether a residual
  landed on one side of an epsilon.
- **Randomized-geometry runs are substantially faster** (#219). The triangle mesh is gone from the
  hot path: entry-face sampling now consumes the closed-form polygon faces directly, and a triangle
  mesh is built only on demand for export. Measured on the same machine and session, at 64 rays per
  sampled shape: Metal 13.7 → 18.1 M rays/s (+32%), CUDA 19.0 → 60.5 M rays/s (3.19×). Scenes with
  fixed shapes are unchanged — they barely construct geometry, so there was nothing there to cut.
- **A negative `face_distance` is a valid value** (#206). It used to be folded to its absolute
  value, which silently turned a negative draw from a distribution into a positive one and hid the
  whole negative domain from the geometry pipeline. A negative distance shifts that plane past the
  origin and still describes a valid convex crystal as long as each opposite pair sums positive.
  Heights are still folded — a negative height has no meaning independent of orientation.

### ⚠️ Breaking Changes
- **`LUMICE_AxisDist` is replaced by `LUMICE_Distribution`, and the crystal shape scalars become
  distributions** (#221). `LUMICE_CrystalParam`'s layout changes; callers must recompile.
  **Before**: `LUMICE_AxisDist { int type; float mean; float std; }` on the three axis slots, and
  plain `float` for `height`, `prism_h`, `upper_h`, `lower_h` and `face_distance[6]`.
  **Now**: one `LUMICE_Distribution { int type; float center; float spread; }` used for all of
  them. The fields are renamed because their meaning is per type — `center`/`spread` hold a mean and
  a standard deviation for `GAUSS`, an interval midpoint and full width for `UNIFORM`, an amplitude
  for `ZIGZAG`, a scale for `LAPLACIAN` — and a new `LUMICE_DIST_NO_RANDOM` type carries a fixed
  value with the spread unused.
  **What to do**: recompile; rename `mean`/`std` to `center`/`spread`; wrap each former scalar shape
  field as `LUMICE_Distribution{ LUMICE_DIST_NO_RANDOM, value, 0 }` to keep the old behaviour.
  `upper_wedge_angle` / `lower_wedge_angle` stay bare floats.
- **`LUMICE_GetCrystalMesh` no longer takes a server or a JSON string** (#214).
  **Before**: `LUMICE_GetCrystalMesh(LUMICE_Server*, const char* crystal_json, LUMICE_CrystalMesh*)`.
  **Now**: `LUMICE_GetCrystalMesh(const LUMICE_CrystalParam*, unsigned long long sample_seed,
  LUMICE_CrystalMesh*)` — it builds the mesh from the parameters you already hold, with no server
  and no round trip through JSON, and takes a seed so a randomized crystal's mesh is reproducible.
  A crystal the geometry gate rejects yields an empty but valid mesh rather than an error.
  **What to do**: drop the server argument, pass the `LUMICE_CrystalParam` you already built
  instead of serialising it, and pass a seed (any fixed value reproduces one draw). Callers that
  only had the JSON must parse it into a `LUMICE_CrystalParam` themselves, or go through
  `LUMICE_SceneFromJson` — see the v4.4.1 entry.

### Fixed
- **A randomized `face_distance` could crash the simulator** (#206). Vertex merging used a fixed
  absolute tolerance, and on a hexagonal corner where four planes nearly coincide — common for a
  `face_distance` drawn around gauss(1, 0.5) — the per-triple residuals landed right at that
  threshold, so one geometric corner survived as several distinct vertices. The resulting
  over-vertexed solid left face records partly initialised and the process segfaulted (measured 4
  crashes in 20 runs on the reported config; 0 in 40 after the fix). The tolerance is now relative
  to the crystal's own scale, and any mesh that still fails a closed-manifold check is replaced by
  an empty crystal that contributes no rays instead of a corrupt one.
- **A pyramid with a randomized `face_distance` crashed on Metal** (#208). Face records were
  allocated at one stride and then counted at a smaller one, so the crystal's copy and move
  constructors computed their offsets from the smaller count and pointed into the wrong part of the
  allocation — a wrong face normal on the CPU, and a wild read during upload on Metal. Allocation
  and count are now derived from the same single pass.
- **The CUDA backend ignored crystal-shape randomization** (#209). Its RNG was reset on every
  session start, collapsing the per-batch shape stream to a constant prefix, and the device-side
  geometry pool was reused across sessions even for a stochastic scene. A randomized run therefore
  produced results bit-identical to a fixed-shape control — the randomization was silently doing
  nothing. Deterministic scenes keep their build-once fast path.
- **Filters collapsed under strong randomization on the GPU backends** (#220). The face-numbering
  table was shared across crystal instances, so with a pool of differing shapes a filter read face
  numbers belonging to a different crystal and matched nothing. Each instance now carries its own,
  on both Metal and CUDA. An all-degenerate crystal pool is tolerated rather than asserted on.

## [4.3.6] - 2026-07-16

### Changed
- **Filter clauses can go far larger** (#202). The OR-clause / AND-term ceiling for a raypath
  filter's boolean expression — 16 / 8 on the host API, 8 on the GPU backends — is raised to 4096
  for plain filtering (a scene that only narrows which rays render, with no coloring involved). A
  scene that wants a large sum-of-products expression over many raypaths no longer has to fit it
  under the old ceiling. The 64-class limit on `raypath_color`'s component mask is unchanged by
  design — widening it would be an always-on cost paid by every scene, for a "more than 64
  distinguishable colors" need nobody has asked for.
- **A GPU color scene that exceeds an internal cap now says so in the GUI** (#204). Three
  GPU-only limits — symmetry groups per slot, OR-summands per color predicate, and the number of
  color classes — used to degrade silently, visible only in the log. All three now surface as a
  warning in the GUI the moment a scene crosses them.

### ⚠️ Breaking Changes
- **`raypath_color`'s default composite mode changes from `dominant` to `painter`** (#199).
  `dominant` picks one winning color class per pixel by brightness — a discontinuous choice that
  flickers between classes wherever two classes' brightness is close, and can render a dim class's
  contribution as pure black wherever a brighter class overlaps it. `painter` is redesigned as a
  proper Porter-Duff alpha-over blend (`alpha = min(brightness, 1)`), which composites overlapping
  classes smoothly instead of a hard winner-take-all.
  **What to do**: a scene that depends on the old winner-take-all look should set `"mode":
  "dominant"` explicitly; `dominant` itself is unchanged, and so is `additive`.
- **`LUMICE_ComplexComposition` becomes a pointer-and-count** (#202), following the
  `raypath_color` precedent from 4.3.5: the fixed-size sub-clause array is replaced by a
  heap-allocated buffer managed through `Create`/`Release` calls, which is also what makes the
  4096-clause ceiling above possible without inflating every config on the stack.
  **What to do**: recompile against the new header; build a Complex filter's sub-clauses through
  the new `Create`/`Release` API instead of a fixed-size array.

### Fixed
- **Changing the view, lens projection, or hemisphere no longer restarts the simulation** (#195).
  Dragging the preview, switching between linear/fisheye/rectangular/globe, or flipping the
  upper/lower-hemisphere crop are purely how the GUI reprojects an already-rendered frame — the
  simulator always produces the same full-sky data — but a regression counted them as
  "configuration changed," which popped the changed-config prompt and, in infinite-rays mode,
  restarted the run on every commit interval.
- **The Metal preview could look frozen while dragging a slider** (#197). Every commit rebuilt the
  Metal pipeline state from scratch (~100-150ms), which took longer than the 70ms commit interval,
  so a continuous drag kept restarting the simulator before its first batch of rays ever landed —
  the ray count stayed at zero for the whole gesture. Metal's pipeline objects are now cached for
  the process's lifetime, and a commit is deferred until the previous run's first batch has
  actually produced rays (or a 250ms timeout, so a heavy scene doesn't stall the UI indefinitely).
- **GPU-rendered colors could bleed into the next batch** (#198). On Metal and CUDA, a per-ray
  color-class mask carried from one multi-scattering layer to the next was supposed to be cleared
  at the start of every batch, but the clear was accidentally gated behind a test-only flag that
  production rendering never set. A scene large enough to need more than one dispatch batch could
  therefore have an earlier batch's colors leak into a later one's first layer, producing a
  visibly different dominant color between the CPU and GPU backends on the same scene (observed up
  to 7× off on one color class).

## [4.3.5] - 2026-07-14

### Added
- **Per-raypath color classification** (#182, #184, #188, #190). Rays can now be classified into
  named color classes and rendered with a distinct color per class, driven by a new
  `raypath_color` config array and a GUI "Colors" window. Each class is defined by one or more
  placement-scoped predicates — a single component, an OR of several, an AND across layers, or an
  entire crystal via a `filter`-less reference — the same reference language `scene.scattering`
  entries already use. Three composite modes are available (`dominant`, `additive`, `painter` —
  see the default-mode change in 4.3.6), each with its own z-order and show/hide/solo visibility.
  Matching respects the same physical symmetry (P/B/D) as an ordinary raypath filter, so a color
  class covers a whole symmetry-equivalent family of raypaths rather than one literal orientation.
  All three backends (CPU, Metal, CUDA) produce matching classifications and composites. Brightness
  is read from one shared exposure anchor across every class — never normalized per class, which
  would misrepresent relative brightness — with the colored composite boosted by one stop relative
  to the full-spectrum image, since a single hue reads dimmer than a full spectrum at the same
  radiance.
  New C API: `LUMICE_ColorPredicate` (carries its own P/B/D symmetry), `LUMICE_SetRaypathColors` —
  a display-time setter, so recoloring or reordering classes doesn't restart the simulation — and
  `LUMICE_ConfigCreateColorClasses` / `_ReleaseColorClasses` (see the ABI note below).
- **The GUI filter editor accepts the full filter grammar** (#177). The old editor could only
  author a single raypath or an entry/exit pair; a filter mixing raypath types, ANDed conditions,
  or several OR'd alternatives had to be hand-written in JSON. It is now a sum-of-products editor —
  a list of OR'd rows, each an AND of factors (`3-5 & entry:2`) — that round-trips every shape the
  config format accepts, including the `;` shorthand for several raypaths in one row.

### ⚠️ Breaking Changes
- **`LUMICE_Config` gains a `raypath_color` field** (#184). Because a color-class list can be
  arbitrarily long, it is not stored inline — doing so once inflated `sizeof(LUMICE_Config)` to
  467 KB during development, enough to overflow a small worker-thread stack. It is heap-allocated
  instead, through `LUMICE_ConfigCreateColorClasses` / `LUMICE_ConfigReleaseColorClasses`, keeping
  `sizeof(LUMICE_Config)` at 113 KB.
  **What to do**: recompile against the new header. A config built through the struct API that
  wants color classes now calls the `Create`/`Release` pair; one that never sets `raypath_color`
  is unaffected.

### Fixed
- **A scene mixing a plate-axis crystal with any non-plate crystal rendered up to 20× slower**
  (#178). The per-distribution orientation-sampling lookup table was memoized by a single-entry,
  most-recent-only cache; a worker alternating between two crystals with different axis
  distributions evicted and rebuilt the table on every single crystal — 250,000 times over in the
  reported scene. It is now a shared, build-once cache keyed by distribution, used by all three
  backends.
- **The crystal preview thumbnail mislabeled faces and kept the wrong pose after switching
  crystals** (#179). Face-normal vectors were left out of the preview's coordinate-frame
  conversion, offsetting every front/back face decision by 90° from the label positions; and the
  preview kept the previously-edited crystal's camera pose when a different crystal card was
  opened, instead of resetting to that crystal's default view.
- **A field being edited in the crystal/filter/axis modal could leak into a different entry**
  (#180). Clicking a different crystal card while a text field was mid-edit relied on Dear ImGui's
  per-widget-ID state, and different entries reused the same widget IDs — so an uncommitted edit
  (a filter expression, a height) could replay into the newly opened entry instead of being
  discarded.

## [4.3.4] - 2026-07-06

### Added
- **GUI custom discrete-spectrum editor** (#168, #173). The Sun panel's Spectrum dropdown gains a
  `Custom…` entry that opens a wavelength/weight table editor (add/remove rows, a preset seed, and
  a Reset button back to that seed), so a discrete custom spectrum — previously only reachable by
  hand-editing the config JSON — can be built and saved from the GUI. Round-trips through `.lmc`
  files and core JSON configs. Incidentally fixes a pre-existing bug where importing a legacy
  hand-written discrete-spectrum config silently dropped the spectrum on load.

### Changed
- **Multi-scattering layer `prob` footguns are now guarded in the GUI** (#169). Setting the
  *last* layer's continuation probability above 0 silently discarded every ray that "continued"
  past it — there is no next layer to receive them — and setting a *middle* layer's probability to
  exactly 0 silently starved every layer after it. The last layer's slider now locks at 0 (with a
  warning if a loaded config set it otherwise, so you can still change it back), a zero
  middle-layer probability shows a warning instead of passing silently, and adding a new layer
  promotes the old last layer's probability to a sane default (0.8) instead of leaving it at the
  near-zero value a final layer would have had.
- **Filters with multiple OR'd raypaths or several entry/exit conditions no longer stall the GUI
  while dragging** (#172). Programmatic filter commits had two paths — a fast typed-struct path
  and a slow, string-based JSON path — and any filter beyond a single plain raypath fell onto the
  slow path, which could not tell the GUI a lightweight update was possible; every slider drag
  forced a full filter rebuild that stalled the live preview. All GUI filter commits now go
  through the fast path.
- **Switching between the CPU and GPU backend and clicking Run no longer leaves the previous
  backend's stale frame on screen** (#172).

### ⚠️ Breaking Changes
- **`ray_num` now means the total ray count across all wavelengths of a discrete spectrum, not
  the count per wavelength** (#168). A discrete-spectrum scene now traces
  `ceil(ray_num / N_wavelengths)` rays per wavelength; a single-wavelength or illuminant-spectrum
  scene is unaffected (identity transform).
  **What to do**: a hand-written discrete-spectrum config should multiply its existing `ray_num`
  by its wavelength count to trace the same number of rays per wavelength as before.
- **The C API's Complex filter now uses a flat reference encoding** (#172), changing
  `LUMICE_FilterParam` into a 5-arm tagged union with its Complex arm's sub-clauses stored in an
  independent pool.
  **What to do**: recompile against the new header. A Complex filter built through the struct API
  is now constructed through the new sub-clause pool rather than nesting `LUMICE_FilterParam`
  values directly.

### Fixed
- **Crystal orientations near the poles could render with an incorrect color tint, and GPU
  renders of near-pole-heavy scenes were slower than necessary** (#171). Near-pole orientation
  sampling on the GPU backends rejected 15-86% of its proposals depending on the distribution
  (worst on a uniform distribution), and an independent RNG-stream collision between wavelength
  and orientation sampling tinted Laplacian-distributed light pillars green. Both are replaced by
  an exact area-measure importance sampler that accepts ~99% of its proposals and uses an
  independent RNG stream per axis.
- **A crystal with a downward-pointing axis, sampled near the poles, could render as if it
  pointed up** (#174). The near-pole sampler folded southern-hemisphere draws into the northern
  hemisphere on the assumption that up and down were interchangeable, which is wrong for a crystal
  whose orientation distribution is not itself up/down-symmetric. Orientation sampling near and
  away from the poles is now unified into one exact sampler that preserves which hemisphere a draw
  belongs to; common configurations (a symmetric distribution with full-circle azimuth) render
  identically to before.

## [4.3.3] - 2026-07-03

### Changed
- **The GPU backends accelerate all 11 render projections, not just 2** (#162). A CLI render
  using a projection beyond the two the GPU backends originally supported silently fell back to
  the CPU backend — correct output, but none of the GPU speedup. Every projection's forward math
  is now one shared implementation used by the CPU, Metal, and CUDA backends alike, so
  cross-backend results cannot drift and every projection reaches the GPU.
- **The GPU backends stay fast at high preview resolutions** (#159). The GPU accumulator used to
  be read back to the host on every simulation batch, which at the GUI's real 2048×1024 preview
  size dominated the cost of a light scene. Readback is now decoupled onto its own cadence instead
  of the trace clock. Measured on the same scene at 2048×1024: 1.4× (RTX 4060 Ti), 2.7× (GTX 1070
  Ti), and ~3× (Apple Silicon) faster than before, and Metal's throughput is now nearly flat across
  resolutions instead of falling off at higher ones.

### Fixed
- **The GUI preview could get stuck showing "Simulating" after a GPU run had actually finished**
  (#167). The simulation's completion state was tracked in two duplicated, edge-triggered places
  fed by torn reads of a shared mutable struct; a narrow race could leave the GUI never observing
  the transition to done. Completion is now read from one authoritative, versioned backend state.
- **The GUI preview's orientation didn't match a CLI render of the same config** (#165, #166).
  Three independent issues compounded: the globe lens was mirrored left-right, the dual-fisheye
  and rectangular lenses were flipped top-to-bottom, and azimuth's left/right handedness was
  inconsistent between lens types. All three are now unified to a single convention — right =
  increasing azimuth — matching the GUI's existing behavior and everyday expectation.
  **Note for existing renders**: a CLI render using a single-lens projection (`linear` or a single
  fisheye) now comes out mirrored left-right compared to before, since that was the half of the
  inconsistency that lived on the CLI side; renders using `rectangular`, `dual_fisheye`, or `globe`
  were already on the corrected convention and are unchanged.
- **A GPU session could silently under-sample its rays after tracing more than about 4.3 billion
  of them** (#164). The per-ray random-number stream was seeded from a ray index truncated to 32
  bits on the GPU backends; once a session's cumulative ray count wrapped past 2³², two different
  rays could draw from the identical stream.
- **A GPU-rendered scene could report `crystals=0` in the CLI's stats line** (#158). The
  diagnostic crystal count wasn't tracked on the exit-seam path Metal and CUDA use; it now reports
  each backend's own notion of "how many crystals were involved" — deliberately not the same
  number as the CPU backend, which counts per-batch instances where the GPU exit-seam counts
  distinct crystal settings in the final layer.
- **The `--benchmark` throughput number for a GPU backend could be wrong by up to 5×** (#160,
  #161). The benchmark's timing window was tied to a coarse-grained progress counter that only
  advanced once per readback drain; a short run could complete almost entirely inside what the
  benchmark counted as "setup," under-reporting the true rate. It now runs a fixed number of
  drains and measures only the steady-state ones in between.

## [4.3.2] - 2026-07-01

### Added
- **CUDA GPU trace backend** (#147, #148, #152, #153, #154, #155, #157). Alongside the Metal
  backend introduced in 4.3.0, ray tracing can now also run on the GPU via CUDA (NVIDIA GPUs, on
  Windows and Linux). Enable it with `--backend cuda`, or via the GUI's GPU checkbox, which now
  reads "Use Metal GPU" on a Mac and routes to CUDA on an NVIDIA GPU elsewhere (a new
  `LUMICE_BACKEND_CUDA` constant, added backward-compatibly to the existing backend-selection
  API). Falls back to the CPU backend automatically, with a diagnostic message, on a GPU that's
  missing, has too old a compute capability, or is otherwise unusable. Windows release builds now
  bundle the CUDA runtime (`cudart64_*.dll`) needed to run it. Measured throughput on a consumer
  GPU (RTX 4060 Ti) reaches roughly 114M rays/s on a light scene, close to that hardware's
  intrinsic kernel rate.

### ⚠️ Breaking Changes
- **Ray-count fields widen to a dedicated 64-bit type** (#149). `LUMICE_StatsResult`'s ray-count
  fields and the config's `ray_num` change from `unsigned long` to a new `LUMICE_RayCount`
  (`unsigned long long`, statically asserted to be at least 64 bits everywhere).
  **What to do**: recompile against the new header. On every platform except Windows this is
  source-compatible; the point of the change is Windows, where `unsigned long` is only 32 bits.

### Fixed
- **The GUI's ray-count display could roll over to zero past about 4.3 billion rays, on
  Windows** (#149). `unsigned long` is 32 bits under Windows' data model (64 bits on Linux/macOS),
  so a long-running session's ray counter silently wrapped. See the ABI change above.
- **A rendered halo could show an incorrect stray bright spot 10-18° off the expected band, on
  Windows only** (#156). When the platform's random-number generator happened to draw exactly
  `0.0` — which only MSVC's implementation does, roughly once every 17 million draws — an
  entry-face sampling routine left its output at whatever the caller had last set it to,
  occasionally picking a face that faces away from the light. That ray then took a spurious
  total-internal-reflection path instead of being absorbed, producing an off-band, out-of-place
  bright pixel.

## [4.3.1] - 2026-06-24

### Added
- **`--backend {auto,cpu,metal}` CLI flag** (#141). The GPU backend introduced in 4.3.0 could
  previously only be selected through an environment variable or the GUI checkbox; the CLI now has
  a first-class selector. `LUMICE_TRACE_BACKEND` remains as a loud, logged override for debugging
  and CI.

### Changed
- **Overlay auxiliary lines are more complete and consistent on screen** (#142). The globe
  projection now draws zenith/nadir markers, which it previously omitted; fisheye lenses no longer
  clip markers that fall in the outer black border; grid line width stays a constant ~2px on
  screen regardless of zoom instead of thickening as you zoom in; the grid's density now adapts
  across seven steps as the field of view changes instead of thinning to one or two lines at high
  zoom; and axis/coordinate labels are placed by a single rewritten algorithm that no longer
  clusters at the projection's edges or drops labels entirely on the globe and dual-fisheye
  lenses.

### Fixed
- **The Metal backend could crash the app on some macOS versions** (#139). macOS 26.5 shipped a
  broken runtime shader compiler that silently produced an empty, non-functional shader library
  instead of an error; combined with assertions that Release builds compile out, this reached an
  unguarded Metal call and aborted the process. Shaders are now precompiled at build time and
  loaded as binary data, bypassing the runtime compiler entirely (the old runtime-compile path
  stays as a fallback on older macOS); and a broken compile can no longer crash the app — it falls
  back to the CPU backend with a log message instead.
- **A raypath filter or hit-path longer than 15 hits could crash the CPU backend, and was
  silently truncated even when it didn't crash** (#140). Recording a ray's hit sequence past its
  inline buffer's capacity required duplicating an overflow allocation that one code path forgot
  to duplicate, leaving a dangling reference; and the hit-path record used to visualize a raypath
  was capped at 15 hits regardless of the configured `max_hits`, silently dropping anything beyond
  that. Both are fixed; the recorded path now always covers the full configured hit count.

## [4.3.0] - 2026-06-21

### Added
- **Metal GPU trace backend** (#121, #122, #129, #131, #138). Ray tracing can now run on the GPU
  via Metal, on any Mac with a Metal-capable GPU; the CPU backend remains the default everywhere.
  Enable it with the `LUMICE_TRACE_BACKEND=metal` environment variable, or in the GUI via a "Use
  Metal GPU" checkbox that only appears when a Metal device is actually available at runtime (not
  merely compiled in) — a Mac without a usable GPU, or running under a VM/remote desktop without
  one, never sees a checkbox that would crash it. Measured end-to-end throughput on Apple Silicon
  is several times the CPU backend's, both for CLI rendering and the GUI's live preview; results
  are statistically equivalent to the CPU backend across every tested scene. New C API:
  `LUMICE_SetPreferredBackend` with `LUMICE_BACKEND_CPU`/`LUMICE_BACKEND_METAL` constants, and
  `LUMICE_IsBackendAvailable` to query at runtime whether a given backend can actually be used.

### Changed
- **Multi-scattering and filtered scenes trace faster on the CPU** (#119). Reworking the per-ray
  hit recorder into a small-buffer-optimized layout (paths up to 16 hits stored inline, longer
  ones spilling to a per-batch arena) cut its memory footprint; measured +57.8% multi-worker
  throughput on a filter-plus-multi-scattering scene, no change on a simple single-crystal scene.
- **The default worker count is now the number of physical CPU cores, not
  `hardware_concurrency - 2`** (#120). The simulator is memory-bound with a dedicated consumer
  thread; the old default packed workers onto SMT/hyperthread siblings, contending for cache with
  each other and starving the consumer — measured ~48% throughput loss on a common 8-core/16-thread
  desktop. `num_workers` set explicitly is unaffected.

### Fixed
- **Some extreme crystal geometries rendered visibly wrong halo patterns** (#133, #135, #137). A
  very flat pyramid (wedge angle near 90°) could grow a fictitious flat face where its cone should
  have come to a point, which fed downstream face-numbering and made physically-distinct raypaths
  (e.g. a straight-through pair and a genuinely refracted pair) render identically; a related but
  independent face-numbering bug could make an entire raypath family go completely dark. Both
  traced to the same class of cause — a face-grouping tolerance too coarse to tell two
  nearly-parallel faces apart — fixed by picking each triangle's best-matching face instead of its
  first adequate match, plus a guard against a zero-thickness crystal producing a NaN normal.
- **The application could hang indefinitely when stopping a simulation** (#125). A worker-thread
  shutdown signal and the condition variable it was supposed to wake could race: if the last
  worker's "I'm done" notification landed in the narrow window while the stopping thread was about
  to go to sleep, the notification was lost and the wait never woke up.
- **The GUI could silently drop a complex filter on import**, rendering the scene as if no filter
  were set at all (#131).

## [4.1.3] - 2026-03-17

### Fixed
- GUI no longer stutters during long simulations (moved server polling to background thread)
- Unicode superscript characters replaced with ASCII in ray number display

## [4.1.2] - 2026-03-16

### Fixed
- Windows exe now statically links GCC runtime (no more `libgcc_s_seh-1.dll` / `libwinpthread-1.dll` missing errors)
- Release workflow `if` conditions fixed for macOS signing secrets

## [4.1.1] - 2026-03-16

### Fixed
- macOS deployment target set to 13.0 (was defaulting to runner OS version 15.0)
- macOS release binaries now code-signed and notarized (Developer ID + Apple notarization)
- `version.py` encoding fix for Windows (`utf-8` explicit)
- Git LFS checkout enabled for CI/Release GUI builds and E2E tests

## [4.1.0] - 2026-03-15

### Added
- Application icon for macOS and Windows (generated from source PNG)
- macOS `.app` bundle with `Info.plist` and icon (Finder-friendly)
- Windows GUI subsystem (hidden console) with embedded icon resource
- Icon generation script (`scripts/generate_icons.sh`)
- Windows CI and Release support (GitHub Actions)
- Preview aspect ratio presets, image export, background overlay, ray number formatting

## [4.0.0] - 2026-03-12

### Added
- GUI application (Dear ImGui + GLFW + OpenGL) with crystal preview, simulation control, and render preview
- `.lmc` binary file format for GUI project save/load
- GUI automated tests (ImGui Test Engine) with visual regression
- GitHub Actions CI pipeline (Ubuntu x64/ARM64, macOS ARM64, Windows x64)
- Release workflow with automated packaging on tag push
- Linux OpenGL support for GUI build
- `version.py` script for version consistency checking and management
- `.editorconfig`, PR/Issue templates, Dependabot configuration
- `CONTRIBUTING.md` with development workflow documentation

### Changed
- Project version updated from 2.1.1 to 4.0.0 (aligning with git tags)

### Fixed
- `asin()` NaN causing white pixel artifacts on ARM64
- Crystal preview coordinate transform (Core Z-up to screen Y-up)
- Various GUI interaction and rendering fixes

## [3.4.1] - 2026-03-09

### Added
- Fisheye projection support (equidistant, equisolid, stereographic, orthographic)
- E2E test infrastructure under `test/e2e/`

### Fixed
- `asin()` input clamping to prevent NaN pixel artifacts
- FOV consistency: corrected f→fov conversion per projection model

## [3.4.0] - 2026-03-02

### Changed
- Simplified JSON configuration format: flattened scene/light/project structure
- Scattering config restructured from parallel arrays to entry-based format
- `ray_num` uses `"infinite"` string instead of `-1`
- Removed unused `StreetLightParam` and `view.distance` config

## [3.3.0] - 2026-02-24

### Added
- Performance optimization: deterministic crystal caching and buffer reuse
- `BM_SimLoop` single-thread benchmark

### Changed
- Deleted `matlab/` directory, promoted `cpp/` contents to root
- Dual-language README (English + Chinese)

### Fixed
- `CommitConfig` race condition causing benchmark SIGSEGV
- Documentation corrections across `doc/` and README

## [3.2.0] - 2026-02-17

### Changed
- Project renamed from IceHalo to **Lumice** (namespace, CMake, C API, documentation)
- C API prefix changed from `HS_*` to `LUMICE_*`, header from `icehalo.h` to `lumice.h`
- Replaced raw `new` with `std::make_unique` throughout codebase
- Refactored logging system to instance-level logger control via spdlog

## [3.1.0] - 2026-02-10

### Changed
- Eliminated `FOR_TEST`, `RANDOM_SEED`, `MULTI_THREAD` compile switches
- Unified test infrastructure (removed `icehalo_test_lib`)
- Migrated dependencies to CPM.cmake with URL-based downloads

### Removed
- Deprecated benchmark files

## [3.0.0] - 2025-12-19

### Added
- C API documentation and developer guide
- Doxygen configuration for API documentation

### Changed
- Major codebase modernization: CMake build system cleanup, dependency management
- V3 rewrite with clean namespace structure

## [2.3.0] - 2021-09-29

### Changed
- Server architecture improvements
- Configuration parsing enhancements

## [2.2.0] - 2021-03-26

### Changed
- Rendering pipeline updates

## [2.1.2] - 2020-02-16

### Fixed
- Minor bug fixes

## [2.1.1] - 2020-01-27

### Fixed
- Minor bug fixes and stability improvements

## [2.1] - 2019-03-25

### Added
- Multi-scattering support improvements

## [2.0] - 2019-03-11

### Added
- Multi-scattering simulation support
- Server/client architecture

### Changed
- Major architecture rewrite

## [1.2] - 2019-03-02

### Added
- Natural color rendering based on spectrum simulation

## [1.1] - 2019-02-19

### Added
- Configuration file support

## [1.0] - 2019-02-16

### Added
- Initial release
- Basic ice crystal halo simulation
- Support for common crystal types (hexagonal prism, plate, column)

[Unreleased]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.4.3...HEAD
[4.4.3]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.4.2...v4.4.3
[4.4.2]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.4.1...v4.4.2
[4.4.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.4.0...v4.4.1
[4.4.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.3.6...v4.4.0
[4.3.6]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.3.5...v4.3.6
[4.3.5]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.3.4...v4.3.5
[4.3.4]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.3.3...v4.3.4
[4.3.3]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.3.2...v4.3.3
[4.3.2]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.3.1...v4.3.2
[4.3.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.3.0...v4.3.1
[4.3.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.8...v4.3.0
[4.2.8]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.7...v4.2.8
[4.2.7]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.6...v4.2.7
[4.2.6]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.5...v4.2.6
[4.2.5]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.4...v4.2.5
[4.2.4]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.3...v4.2.4
[4.2.3]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.2...v4.2.3
[4.2.2]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.1...v4.2.2
[4.2.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.2.0...v4.2.1
[4.2.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.14...v4.2.0
[4.1.14]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.13...v4.1.14
[4.1.13]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.12...v4.1.13
[4.1.12]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.11...v4.1.12
[4.1.11]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.10...v4.1.11
[4.1.10]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.9...v4.1.10
[4.1.9]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.8...v4.1.9
[4.1.8]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.7...v4.1.8
[4.1.7]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.6...v4.1.7
[4.1.6]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.5...v4.1.6
[4.1.5]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.4...v4.1.5
[4.1.4]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.3...v4.1.4
[4.1.3]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.2...v4.1.3
[4.1.2]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.1...v4.1.2
[4.1.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.1.0...v4.1.1
[4.1.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.0.0...v4.1.0
[4.0.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v3.4.1...v4.0.0
[3.4.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v3.4.0...v3.4.1
[3.4.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v3.3.0...v3.4.0
[3.3.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v3.2.0...v3.3.0
[3.2.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v3.1.0...v3.2.0
[3.1.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v3.0.0...v3.1.0
[3.0.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v2.3.0...v3.0.0
[2.3.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v2.2.0...v2.3.0
[2.2.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v2.1.2...v2.2.0
[2.1.2]: https://github.com/LoveDaisy/ice_halo_sim/compare/v2.1.1...v2.1.2
[2.1.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v2.1...v2.1.1
[2.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v2.0...v2.1
[2.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v1.2...v2.0
[1.2]: https://github.com/LoveDaisy/ice_halo_sim/compare/v1.1...v1.2
[1.1]: https://github.com/LoveDaisy/ice_halo_sim/compare/v1.0...v1.1
[1.0]: https://github.com/LoveDaisy/ice_halo_sim/releases/tag/v1.0
