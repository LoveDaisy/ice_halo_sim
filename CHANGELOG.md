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

**When entries are written**: at release time, in one batch, for the whole span since the
previous tag. This file records changes between tags only — there is no accumulator section for
work merged since the last release, and a PR does not add its own entry when it lands. The
release chore enumerates every PR in the span mechanically (the Sourcing rule below), decides
per PR whether it earns an entry, and writes the version's section in one pass; a section that
grew one PR at a time was measured to end up half-full, which reads as complete and is worse
than empty. The steps are in `CONTRIBUTING.md`, "Release Process".

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

### Which section an entry goes in

`Fixed` is for a change to **what the program does** with the same input: it used to hang,
crash, drop rays, show wrong pixels, report a wrong number, or fall back to a default, and now
it does not. `Changed` is for everything that was already working and is now different or
better — faster, clearer, differently shaped output.

Two boundary forms have each been mis-sorted once during backfill, so they are stated here
rather than re-decided per version:

- **A guard or warning added in the editor, with the engine untouched**, is `Changed`, not
  `Fixed`. The tell is the question *does an unchanged config produce different output?* If a
  change only closes off a way to ask the engine for something useless — nothing the program
  computed was ever wrong, and every pre-existing config still renders byte-identically — it is
  `Changed`, however much the entry's prose talks about what used to go wrong silently. (Worked
  example: PR #169 has the GUI lock the last multi-scattering layer's probability and warn about
  a zero middle layer; its description states the core rendering semantics are unchanged.)
- **A user-visible fault that a redesign incidentally removes** is `Fixed`, and gets its own
  bullet there, even when the redesign itself is an `Added` entry. Keep the two claims
  separate: the feature is what you can now do, the fix is what no longer breaks.

A bullet that bundles several sub-claims across this line should be split — unless the split
would separate parts of one visual change a reader perceives as a single thing, in which case
keep it whole and let the dominant claim pick the section.

One change is described in one section, once. When a change earns a `Breaking Changes` entry,
that entry is the whole of it — do not also narrate it under `Added` or `Changed`. (This is a different question from
the Granularity section's split-by-PR rule above: that one decides how many *entries* one PR
becomes, this one decides which *section* one entry's sub-claims land in.)

### Breaking changes

A change to a default value, to a config-file semantic, or to the C API's ABI or behavior gets
its own `### ⚠️ Breaking Changes` subsection inside that version, and states three things:
what a user saw before, what they see now, and what they need to do about it.

The qualifier is that something a user already has must behave differently: an existing config,
`.lmc`, script, or compiled consumer. A default that only applies to newly created things — a
GUI document started from scratch, a fresh config — breaks nothing, because nothing existing
changes; it is `Changed`. The check is the entry's own third clause: if what a user needs to do
is *nothing*, it does not belong in this subsection.

### Sourcing

`gh release view <tag> --json body` lists a version's PRs but is **not authoritative on its
own** — it has silently missed a merged PR at least once (v4.1.7 omits PR #24). Cross-check
against git:

```bash
git log <prev_tag>..<tag> --oneline --grep='^Merge pull request #'   # PRs merged via merge commit
git log <prev_tag>..<tag> --first-parent --no-merges                 # squashed PRs and direct commits
git diff <prev_tag>..<tag> -- src/include/lumice.h | grep LUMICE_API_VERSION   # C API moved → Breaking candidates
```

The second command is not redundant: dependabot bumps and admin-merged single-commit PRs land
with no merge commit to grep for, and direct-to-main commits appear in no PR list at all.
The third is the mechanical trigger for the `Breaking Changes` subsection: if the version
constant moved, at least one PR in the span changed the C API and its entry has to say what
a compiled consumer sees. (Before the release is cut, `<tag>` does not exist yet — use the
branch, e.g. `v4.5.0..main`.)

A PR's own description is not authoritative on whether it breaks the C API. Diff the header
across each version boundary instead, to see *what* moved rather than only that it did:

```bash
git diff <prev_tag> <tag> -- src/include/lumice.h
```

This is not a belt-and-braces check. Over the v4.1.4–v4.1.14 backfill it surfaced six ABI
changes no PR description mentioned — a value inserted mid-enum, a signature that gained a
parameter, three structs that grew fields, and an identifier whose meaning was replaced — one of
them in a PR that called itself "backward compatible", which was true of the JSON config layer
it was thinking of and false of the C struct beside it.

</details>

## [4.5.1] - 2026-09-10

### Added
- **Print mode: ink laid on paper, for a halo on a light background** (#337, #343). A white or
  pale background used to erase the halo entirely. Light is *added* to the background and clamps
  at white, so with a white sky every pixel was at the ceiling before any ray energy arrived —
  while the grid and overlay lines, which are blended rather than added, stayed perfectly visible,
  so the result read as a failed simulation rather than a colour choice. No foreground colour can
  fix that under an additive operator, so the fix is a second operator. `render.tone` chooses it:
  `screen` (the default, and byte-for-byte what every existing config rendered) or `print`, under
  which accumulated radiance becomes a neutral ink density laid on `render.paper` (a new field,
  default white): `D = 11·log10(1+e)`, `out = paper·10^(-D)`, so a white page can go dark. Print is
  greyscale by construction — hue is given up on purpose, since "ink absorbs the complementary
  colour" would make white-light features such as parhelic circles vanish on white paper — so a
  print reads like a drawing and its arcs are told apart by position and shape. In the GUI the
  Display panel's `Mode` switch selects it and the ground-colour row becomes `Paper Color`. Because
  ink owns the colour channel under `print`, four things that colour pixels are mutually exclusive
  with it and say so instead of silently doing nothing — the background photo overlay, raypath
  colouring, `ray_color`, and per-family annotation colours: the CLI logs one warning per field it
  will not read, and the GUI greys the control with the reason. Overlay text labels are ink too
  (#343 closed the one path — the GUI's ImGui-drawn labels — the first cut missed, so under Print
  every label was still drawn in its family's hue beside a black line). A contrast-headroom notice
  guards the degenerate end of *each* operator through one shared predicate: a `screen` background
  within a few 8-bit levels of white, or a `print` paper within a few of black, both render a blank
  picture from a configuration that looks reasonable — the CLI warns at commit, the GUI shows an
  inline notice with a one-click repair (`Switch to Print` / `Reset paper to white`). `.lmc` and
  exported JSON round-trip both fields.
  **ABI**: `LUMICE_RenderParam` gains trailing `tone` / `paper[3]` (`LUMICE_API_VERSION` 426 → 427,
  appended so every existing field keeps its offset; `sizeof` grows, recompile against the new
  header). A zero-initialized `tone` is `LUMICE_TONE_SCREEN`, the operator this API has always
  used — but `paper`'s JSON default is white while a zeroed struct names black paper, which under
  `print` is an all-black page. Set it, or go through JSON.
- **An eyedropper for Sky Color that samples the background photo** (#334). Next to the Sky Color
  swatch, a pipette enters a picking mode in which the preview shows the photo alone — render,
  auxiliary lines and lens border are suppressed, so what you see is what you get — and a colour
  chip follows the cursor until you click. Sampling the *sky* of the photo is the point: the photo
  blend is `background·(1−α) + render·α`, so the closer `background` is to the photo's own sky
  colour, the less the photo is washed out and the more the halo comes through as pure increment.
  The button is greyed, with the reason, while the photo is hidden or none is loaded.
- **A config can now ask for a grid family's numbers without its lines.** (#323) The parallels, the
  meridians and the sun's angular-distance circles each gain a line switch of their own —
  `grid.elevation_line`, `grid.longitude_line`, `grid.angular_dist_line`, all defaulting to true, so
  every existing config renders exactly as before. Previously only the horizon could be asked for
  text without a line; for the other three, "do not draw me" could only be said by removing their
  angles, which removed their labels too. The GUI's exported config follows suit: turning grid or
  circle **lines** off while leaving their **labels** on used to drop the numbers from the CLI
  render with no warning, and now exports exactly what the preview shows.
- **`LUMICE_RenderParam` gains `elevation_line` / `longitude_line` / `angular_dist_line`** (#323)
  (`LUMICE_API_VERSION` 425 → 426, appended at the end of the struct — recompile against the new
  header). Note the defaults run the other way from every other annotation flag: the JSON default is
  *on*, so a zero-initialized struct asks for no lines even when it carries a full angle list. Set
  the three fields, or go through JSON.
- **`LUMICE_ConvertMillerIndexToWedgeAngle` — one place to ask what Miller indices mean.** (#336)
  New C API function returning the wedge angle *and* a verdict on the indices themselves: valid, "no
  cone this side", "not enough indices yet" (so a UI can call it on every keystroke without owning a
  rule for when a row is finished), or invalid, with the offending slot named where one slot is at
  fault. No `LUMICE_API_VERSION` change — a new function, no struct or ABI change.
- **The wedge-angle dropdown takes custom Miller indices.** (#336) Below the four built-in presets
  the Crystal editor's Upper A / Lower A dropdowns now carry a row of index boxes in the same
  `{h,k,i,l}` notation the preset labels use, so a face the table does not offer — `{3,0,-3,1}`, say
  — can be asked for by name instead of converted to degrees by hand. The angle updates as you type
  and is only written when you press Apply, so a triple being typed through never lands
  half-finished. `i` is shown, not typed: it is `-(h+k)` by definition, and deriving it means the
  four numbers can never contradict each other. Indices that name no buildable face are refused with
  the reason spelled out and Apply greyed — a non-zero `k` (a second-order pyramidal face this
  crystal model cannot express), a negative index, an h:l ratio outside the buildable range, and `h
  = 0`, which says "no pyramidal cap on this side" and belongs in the pyramid height rather than in
  an angle. Nothing is stored: the dropdown writes the angle and no Miller indices enter the
  document.
- **The Settings panel keeps the wedge angles you use.** (#336) A new region under Settings >
  Presets saves crystal faces by their Miller indices, and everything saved there is offered in the
  Crystal editor's Upper A / Lower A dropdowns from then on, across restarts. The add row is the
  same `{h,k,i,l}` control the dropdown carries, refusing the same triples for the same stated
  reasons. Entries are stored as indices rather than as degrees, so a saved face keeps naming the
  same face if the ice constants are ever corrected; a preset has no name of its own for the same
  reason — the indices are the name. The four built-in presets are unaffected and cannot be deleted:
  your list is added to them, and a triple they already cover is not saved twice. Nothing is written
  until you press Save, and closing the panel discards the change. A file edited by hand can hold a
  face that names no buildable cone; the panel shows that row with a warning and a live delete
  button rather than hiding it, and loading one says which entries it dropped and why.

### Changed
- **The CLI no longer rounds every render up to a whole second** (#324). The output loop used to
  wait out its 1 s poll interval *before* checking whether the simulation had finished, so a
  100-ray render and a 6,000,000-ray one both took 1.05 s and every render's wall time was a
  whole-second staircase. It now checks first and waits only if there is something to wait for;
  a 20k-ray render measures 1.02 s → 0.04 s, and the results file and stats are written no more
  often than before. (The test suite does not get proportionally faster — under a parallel run the
  waits already overlapped — so this is a latency change for a person at the terminal, not a CI
  one.)
- **The Display panel's Rendering rows are regrouped and renamed** (#338). The rows now read
  `Resolution` ‖ `EV Anchor` / `EV` / its read-out ‖ `Mode` / ground colour / warning, so the EV
  read-out sits under the value it reports instead of two rows away. The ground-colour row is one
  control with two faces — `Sky Color` (with the eyedropper) under Screen, `Paper Color` under
  Print — because the renderer reads only one of the two fields in each mode, so one of two
  permanently shown controls was always dead; both fields keep their own values, `.lmc` keys,
  Settings rows and Revert baseline, and switching modes back and forth changes neither. Three
  labels: `Tone` → `Mode`, the `ev_mode` selector's `Mode` → `EV Anchor` (the same word as
  `anchor_l99_sky`), `Paper` → `Paper Color`.
- **Importing a core/CLI config into the GUI now warns about the keys the GUI deliberately does
  not support** (#328). A non-zero `sun.azimuth`, `render[0].lens_shift` or `scene.geom_clock` in
  an imported JSON used to be dropped without a word — a hand-written `azimuth: 30` rotated the
  whole picture by 30° in the CLI and by nothing in the GUI, with no hint why. All three now
  raise the same import-warning popup the reader already uses for a complex filter it cannot hold.
  This is a boundary, not a gap: the GUI fixes the sun's azimuth at 0 and the CLI keeps the
  freedom; the change is that the boundary now speaks.
- **The "layer produces no rays" notice reads the weights that actually reach the engine** (#320).
  It used to fire only when every crystal in a layer was excluded; a layer whose crystals were all
  included with `Weight` 0 is the same engine configuration byte for byte and was not reported.
  The notice now covers both, and names both controls.
- **`upper_indices` / `lower_indices` are now judged rather than partly ignored.** (#336) These
  arrays used to be read only when they held exactly three entries, and only entries 0 and 2 were
  looked at, so three kinds of mistake passed silently: a four-index array — `[1,0,-1,1]`, the
  notation users actually write — fell through entirely and left the default 28° looking like a
  stated value; a non-zero second index was dropped, rendering `[1,1,2]` as `[1,0,2]`'s 46.756°
  instead of the 31.545° it names (a shape this crystal model cannot build at all); and a negative
  index produced a negative angle whose cone then vanished from the render with nothing said. Each
  of these now leaves the wedge angle at its default **and logs a warning naming the array and the
  reason**. Applies equally to the CLI, the C API and the GUI's own reader, which had each kept a
  copy of this conversion. In the GUI the report is also an import-warning popup rather than a line
  in the log panel alone, matching every other downgrade the document reader performs: the angle a
  refusal leaves behind is a default the document never stated, and nothing on screen distinguishes
  it from one the document did state.

### ⚠️ Breaking Changes
- **`LUMICE_ComputeAnnotationOverlay` is removed; `LUMICE_ComputeAnnotationAnchors` replaces it**
  (#342, `LUMICE_API_VERSION` 427 → 428). Gone with it: `LUMICE_ReleaseAnnotationOverlay`, the
  `LUMICE_AnnotationOverlay` struct (its `drawable` / `horizon` / `elevation` / `longitude` /
  `angular_dist` masks and the `zenith_*` / `nadir_*` fields), and the `zenith_nadir` and
  `want_labels` fields of `LUMICE_AnnotationRequest` — that struct's layout changes, so every
  caller recompiles. The old call rasterized the auxiliary lines into width×height masks on the
  CPU and was documented as "not a per-frame call"; its one interactive consumer, the GUI preview,
  answered by freezing every line for the duration of a camera drag (the `Fixed` entry below). The
  lines are level sets of three world-space angle fields, which any renderer can evaluate from the
  direction it already has, so the new call answers only what a renderer cannot derive locally —
  where along each curve its label sits, and where each named marker lands — in tens of
  microseconds, with nothing in it proportional to the canvas, and is designed to be called every
  frame. `LUMICE_AnnotationLabel`, `LUMICE_AnnotationMarkerPoint`, `LUMICE_AnnotationView`, the
  `LUMICE_ANNOTATION_*` constants and the error contract are unchanged; the CLI renderer is
  unaffected, since it composites through core in-process and bakes the same masks it always did.
  **What to do**: a C/FFI caller that drew the masks draws the curves itself from the same three
  angle fields (the preview shader's `blendAnnotationColor` and core's `annotation_overlay.cpp` are
  the two reference evaluations) and takes text and marker positions from
  `LUMICE_ComputeAnnotationAnchors` / `LUMICE_ReleaseAnnotationAnchors`; ask for the poles through
  `marker_ids` (`LUMICE_ANNOTATION_MARKER_ZENITH` / `_NADIR`) rather than the removed switch.
  No JSON, `.lmc` or rendered-image change.
- **`[0,0,l]` now means "no pyramidal cap on this side", not 28°.** (#336) ⚠️ Behaviour change for
  existing configs: a crystal whose `upper_indices` or `lower_indices` starts with 0 (and states no
  explicit `upper_wedge_angle` / `lower_wedge_angle`) previously rendered a 28° cone on that side
  and now renders a plain prism end. 28° was never a stated value — it was the field's default
  showing through — and every other part of the engine already read a leading 0 as "no cone". Add an
  explicit `upper_wedge_angle: 28.0` to keep the old picture.

### Fixed
- **The preview's auxiliary lines follow the camera every frame again** (#342). Since 4.5.0 the
  horizon, elevation/longitude grid and sun angular-distance circles were rasterized by core into
  a mask and refreshed only after the view had held still, so during a drag or zoom they froze
  where they were and jumped into place on release — while the lens border, which never took that
  path, kept tracking. They are now evaluated per fragment in the preview shader from the same
  level-set definition the CLI rasterizes, so they move with the picture; measured over twelve
  consecutive frames of view change, the old path was up to 342 px off, the new one matches the
  analytic position. The line profile is the same hard set the CLI draws (no antialiasing on
  either side), so the picture at rest is unchanged.
- **A config exported from a dual-fisheye view now draws its discs the size the screen shows**
  (#342). The GUI's export wrote `overlap: 0.0872` — the value it uses to *sample its own source
  texture*, not a property of the picture — and the CLI drew each disc 4% smaller than the preview
  did. The export now writes `0`; a JSON file already saved keeps its old value and renders as
  before, and re-exporting from the GUI picks up the fix.
- **On CUDA, a run whose first layer has every weight at zero no longer poisons the GPU backend
  for the rest of the run** (#320). The reported shape — run once with all population weights at
  0, give one a weight, and get ten-plus seconds of garishly over-saturated colour — was the GPU
  backend silently dropping to the CPU: an all-zero layer skipped the per-crystal loop that
  records the kernel timing events, the unchecked timer reads then left a sticky CUDA error behind,
  and the next run's first kernel-launch check read that error and blamed the launch. The
  fallback then kept the GPU's batch size, so every CPU batch carried a single wavelength, which is
  where the colour came from. All three links are fixed. Should the backend genuinely stop
  mid-run, the GUI now says so in a warning instead of only getting slower, batches queued at the
  GPU's size are dropped rather than traced one wavelength at a time, and the CPU continues at its
  own batch size.
- **Deleting an entry or a layer while the editor is open no longer rewrites a different entry**
  (#319). In the default Immediate editing mode the Edit Entry window binds its target by index,
  and deleting an entry *above* it left the index in range but pointing at the next entry down,
  whose crystal and filter were then overwritten with the deleted entry's contents once per frame
  — the beta report of "the card below gets closed" was that card's contents being replaced and
  its label renumbered. The binding now follows its entry down on a delete above it, closes on a
  delete of the entry itself, and the same holds for deleting a layer above the bound one.
- **A malformed raypath row in a `.lmc` is refused instead of silently reinterpreted** (#329). A
  row like `3--5` (or `-3-5`, `3-5-`, or one naming a face that does not exist) used to load as
  `{3, 5}` — a plausible path the user never wrote — because the load path never ran the syntax
  check the editor runs. The earlier 4.1.14 / 4.2.0 editors let such a row be typed and saved, so
  this reaches real files. The row is now dropped on load, logged, and reported in the same popup
  the open dialog already uses for other downgrades.
- **A `Run` you press is never dropped by the auto-commit backpressure gate** (#325). The gate
  that keeps the 70 ms auto-commit from re-submitting while a run's first batch is still in
  flight applied to the `Run` button too, so pressing it within 500 ms of the previous run, before
  that batch had landed, discarded the whole commit — "I pressed Run and nothing happened". A
  user-initiated run is exempt; the gate still governs automatic commits.
- **Under Print, the photo drag and zoom gestures no longer act on a photo that is not on
  screen** (#338). Print suppresses the background photo overlay, but the gesture handlers still
  reported the photo as visible, so dragging the preview moved an image the frame did not show.
  One predicate now decides both whether the photo is drawn and whether it can be handled.
- **The wedge-angle preset dropdown named crystals it did not draw.** (#336) Its four entries
  carried hand-transcribed angles with the Miller ratio inverted, unchanged since they were first
  written: `{2,0,-2,1}` set 47.300° where that face is at **14.886°**, `{1,0,-1,2}` set 14.700°
  where it is at **46.756°**, and `{1,0,-1,1}` set 28.000° for **27.996°**. Picking one built a
  crystal that was not the one the label named. The angles are now computed from the indices, so the
  label and the number cannot disagree again. ⚠️ Every config saved by picking one of these presets
  keeps its stored angle — the old number is still there and still renders what it always did;
  re-pick the preset to take the corrected value.
- **`{1,0,-1,0}` has left the preset list; `{1,0,-1,3}` (57.912°) takes its place.** (#336)
  `{10-10}` is a prism face and has no wedge angle at all, so its 90.000° was not a rounding error
  but a category one. What it produced was a plain prism — reachable, then and now, by setting the
  pyramid height to 0, which is where "no cone on this side" belongs.

## [4.5.0] - 2026-09-06

### Added
- **Lens border ring for fisheye projections** (#283). Overlay gains a `Lens Border` row (colour +
  toggle, off by default) that draws the projection's own image-circle boundary — equal-area,
  equidistant, orthographic and all four dual-fisheye variants — so a halo that does not fill the
  frame no longer looks indistinguishable from the black surround outside the lens's valid domain.
  `linear`, single-lens `fisheye_stereographic`, `rectangular` and `globe` are unaffected: the first
  three already fill the frame at every reachable field of view, and `globe` is excluded as a
  product choice despite having a bounded image circle of its own.
- **GUI background colour reaches the picture** (#286). The preview, the three PNG exports
  (screenshot / dual-fisheye / equirectangular) and the frame baked into a saved `.lmc` now all
  paint the configured background colour behind the halo — previously only the CLI did, so the GUI
  colour picker moved and nothing on screen changed. The colour is composited additively in linear
  RGB before the sRGB transfer curve, which makes a pixel carrying no halo energy render as exactly
  the sRGB triple the picker showed; it is painted only where the lens actually images sky, so the
  black surround outside a fisheye's image circle stays black.
- **Raypath-colour (composite) display honours the background colour** (#286). With raypath
  colouring on, the picture now carries the same background as with it off — previously the
  composite was baked server-side with a black surround, so toggling colouring changed the
  background out from under the user.
- **Two new C API entry points for the background colour** (#286, ABI additions, non-breaking):
  `LUMICE_SetCompositeBackground(server, background_linear)`, a display-time push shaped exactly
  like `LUMICE_SetCompositeExposure` (no epoch bump, no re-simulation; all-zero is an algebraic
  no-op, so a caller that never calls it sees byte-identical composites), and
  `LUMICE_XyzToSrgbUint8WithBackground(...)`, the existing `LUMICE_XyzToSrgbUint8` with an additive
  linear-RGB background composited before the final clamp and gamma.
- **`ev_mode`** (#287, #299) — a first-class choice between two exposure anchors, reaching core
  `RenderConfig`, the C API, `.lmc` documents and CLI JSON. `"absolute"` anchors to the energy the
  light source EMITTED, so two renders at the same EV are directly comparable. `"relative"` (the
  DEFAULT) anchors to the P99 radiance of a fixed, full-sky reference buffer built for the scene —
  the same anchor the GUI preview has always used — so it does not depend on which view, lens,
  `visible` clip or output resolution a particular render asks for; `ray_num` still co-determines
  it, since the anchor is a statistic over the accumulated simulation. A config with no `ev_mode`
  key, or a misspelled value, renders `relative`. On the composite (raypath-colour) path the same
  field selects between the participating-pixel self-anchor and the mono path's absolute scale,
  sharing one scalar rather than re-deriving it.
  **ABI**: `LUMICE_RenderParam` gains a trailing `int ev_mode` (`LUMICE_EV_MODE_RELATIVE` = 0,
  `LUMICE_EV_MODE_ABSOLUTE` = 1); `LUMICE_API_VERSION` moves 415 -> 416 (the same step also removes
  `opacity`, below). See Breaking Changes below for what selecting each mode does to an existing
  config's brightness.
  The GUI's own auto-EV anchor calculation moves into core alongside this, as two new pure C API
  functions with no ABI-breaking half: `LUMICE_ComputeP99Y` (the P99 statistic itself, with an
  optional coarse-grid downsample) and `LUMICE_ComputeEvAuto` (the stops-and-clamp arithmetic on
  top of it) — so the GUI and any other consumer share one implementation of the anchor instead of
  each carrying their own copy.
- **`LUMICE_RawXyzResult.emitted_energy`** (#287, C API, ABI addition): the total spectral energy
  the light source emitted into a snapshot — the quantity `absolute` mode normalizes by. A raw
  total, unlike the neighbouring per-pixel `snapshot_intensity`, and a different measurement, not a
  rescaling of it, since one counts what went in and the other what landed. A consumer reproduces
  the renderer's own absolute scale as `intensity_factor * kNormScale * total_pixels /
  emitted_energy`. Occupies pre-existing alignment padding, so `sizeof(LUMICE_RawXyzResult)` is
  unchanged.
- **Core-side annotation layer**: the CLI renderer can now draw every auxiliary line and marker the
  GUI preview draws, from one shared implementation instead of two (#292, #305). A `render[]` entry
  can ask for parallels (`grid.elevation`), meridians (`grid.longitude`, new), circles of angular
  distance from the sun (`grid.angular_dist`, renamed from `grid.central` — see Breaking Changes), a
  front-hemisphere clip (`front`, new), text labels for each of those families
  (`horizon_label` / `grid_label` / `angular_dist_label`, new), and — generalizing the existing
  zenith/nadir ring into a named list — up to six sky-direction markers (`grid.markers`: zenith,
  nadir, sun, subsun, anthelion, antisolar), each independently switched, coloured and drawn, with a
  matching `[Look At ▾]` view preset in the GUI's View group. All of this previously existed in the
  GUI only, or not at all in this generalized form; an exported config now reproduces it. New C API:
  `LUMICE_ComputeAnnotationOverlay` / `LUMICE_ReleaseAnnotationOverlay` (pure, side-effect-free
  geometry and label anchors for one view), `LUMICE_ResolveAnnotationMarkerDirection` (a named
  marker as a world direction, for pointing the camera at it rather than finding its canvas
  position), and `LUMICE_ResolveSunHorizonDirection` (the sun's azimuth carried down to the
  horizon, for the same View-preset use).
  **ABI**: a series of `LUMICE_RenderParam` field additions across this arc — `elevation_grid`
  becoming rendered, `longitude_grid`, `zenith_nadir`, `front`, the three `*_label` switches, and
  `markers` / `markers_count` / `markers_opacity` / `markers_radius_px` — every one appended so
  existing fields keep their offsets, and every one opt-in: a zero-initialized struct draws none of
  them, matching what a config with no `grid` object already rendered. `zenith_nadir` and the
  `.lmc` `overlay_zenith_nadir_*` keys keep working unchanged; where both `zenith_nadir` and a
  non-empty `markers` list are present, `markers` wins. `LUMICE_API_VERSION` moves 416 -> 421 for
  the `render[]` fields above (#292); the marker generalization (#305) later resumes this same
  counter at 423 (after the two `LUMICE_RawXyzResult` additions below take it there), moving it
  423 -> 424 for the underlying named-direction table (`LUMICE_AnnotationMarkerPoint` and the
  `LUMICE_ANNOTATION_MARKER_*` ids, all appended at each struct's physical end) and 424 -> 425 for
  `markers[]` itself — 425 in total, matching `HEAD`.
- **`LUMICE_RawXyzResult.anchor_l99_sky` and `.axis_solid_angle`** (#299, C API, ABI additions):
  the two quantities a consumer needs to reproduce the `relative`-mode scale itself —
  `anchor_l99_sky` is the P99 radiance of the fixed full-sky reference buffer (identical on every
  row of one call), and `axis_solid_angle` is the solid angle this renderer's own on-axis pixel
  subtends: `scale = intensity_factor * TargetWhiteToLinear(135) / (axis_solid_angle *
  anchor_l99_sky)`. `anchor_l99_sky` grows the struct (64 -> 72 bytes) — a caller that was not
  recompiled hands the API a shorter buffer; `axis_solid_angle` then lands in the padding that
  growth leaves behind, so it does not grow the struct a second time. `LUMICE_API_VERSION` moves
  421 -> 422 for the first field, 422 -> 423 for the second (the annotation-layer entry above
  continues the count from here to 425).
- **CLI `--workers N`**, and a matching GUI personal default (#314). Both front ends previously left
  worker count at its zero-initialized automatic default; `--workers` now lets a user pick it
  explicitly (rejected outright, not silently clamped, if it is not a positive integer), and the
  GUI Settings panel can save a preferred value the same way it saves other personal defaults. See
  Breaking Changes below for the new automatic default this doesn't override.
- **GUI "Use GPU" can be saved as a personal default** (#311). Previously the checkbox reset to
  legacy CPU on every launch even after an explicit choice; it can now be saved via Settings like
  other personal defaults. The factory default is unchanged (CPU), and a saved GPU preference on a
  machine with no GPU backend available is silently ignored by the existing runtime fallback.

### Changed
- **Cylinder crystal height can go as low as `1e-4`**, down from `0.01` (#301) — the GUI slider's
  floor was far above core's actual `h > 1e-5` acceptance, so this range was reachable by editing a
  config by hand but not from the slider. Several sliders' display formats are also corrected
  (`axis` fields, `sun.diameter`) where the digit shown did not change finely enough to track the
  slider's own step size, making the control look frozen mid-drag.
- **Legacy CPU backend throughput improved via reduced allocation churn** (#303). A per-batch
  working buffer was reallocated on every batch; it is now reused for the worker's lifetime,
  measured to cut large allocations from roughly 17,000 to about a dozen in one run. Measured
  throughput gain ranges from +2.4% to +159.5% across two machines, worker counts and scenarios
  (largest at 16 workers on a 16-core machine); peak memory dropped 57-61% in the same runs. Output
  is bit-identical — no `GetUniform()` call sites were added or removed.
- **The renderer-count-exceeded error names the actual count and the limit** (#294). Loading a
  fifth `render[]` entry used to fail with a bare `error code 3`; the CLI now reports `config has 5
  "render" entries, exceeding the limit of 4`. The limit itself (4) is unchanged.

### ⚠️ Breaking Changes
- **`LUMICE_RenderParam::opacity` is removed** (#286). `render[].opacity` is no longer parsed from
  JSON either. The field had no drawing consumer anywhere in the tree since the first commit —
  setting it changed nothing — and had no GUI counterpart. `LUMICE_API_VERSION` moves 415 -> 416,
  the same step that adds `ev_mode` above.
  **What to do**: drop the assignment from C/FFI callers and recompile; an existing JSON config
  that still sets `"opacity"` keeps loading (unknown keys are ignored). This is a different field
  from `LUMICE_GridLine::opacity`, which is untouched. The GUI's own mirror of the field
  (`renderer.opacity`, in Settings and in `.lmc`) is removed with it — a `.lmc` written by an older
  build still opens.
- **Display normalization gains an absolute anchor, and the CLI's own denominator changes to use
  it by default** (#287). The renderer's scale used to divide by the energy that LANDED on a
  pixel — a quantity that moves with the scene (add a filter, or narrow the lens, and the image
  silently re-brightened by whatever was removed), which made two renders at the same EV
  incomparable. The `ev_mode` field (see Added) now picks the denominator: `absolute` divides by
  the energy the source EMITTED, fixed by the light source and the ray budget alone; `relative`
  (the default) is described under Added, and its own migration is covered by the entry below.
  **What to do**: rendering with `ev_mode: absolute` produces a darker image than the old CLI
  behavior, by exactly `landed_fraction` (energy landed / energy emitted), a per-scene constant
  independent of `ray_num` — negligible for a full-sphere view (-0.038 .. -0.002 stop), around a
  stop for a 90-120 degree lens (-0.67 .. -1.25), and up to -10.47 stops behind a filter or at high
  `ms_prob`. Raise EV to taste; a caller can also compute the exact shift from
  `LUMICE_RawXyzResult`'s `snapshot_intensity` and `emitted_energy` without re-deriving anything.
  Cross-lens comparability is not claimed: two projections still differ by a per-projection
  solid-angle constant. `kNormScale` (0.08) is unchanged — full-sphere `landed_fraction` medians
  0.980, so re-deriving it would move full-sphere scenes by only +0.029 stop against the cost of
  every reference image. Separately, an undersampled scene darkens as `ray_num` grows under either
  denominator (measured slope ~ -1.03 for both) — an honest Monte-Carlo estimator property, not
  something this change introduces; see `doc/ev-pipeline-architecture.md` §7.4 before reporting it
  as a regression.
- **`relative` mode now anchors to the same fixed reference buffer the GUI has always used,
  instead of to the render's own output** (#299). The CLI's `relative` implementation (above)
  anchored to the P99 of the view actually being rendered — dependent on its lens, `visible` clip
  and output resolution — where the GUI anchored to a full-sky buffer built independently of any
  of those. This was a bug in the CLI implementation rather than a second definition of
  `relative`; the two anchors now agree, algebraically, on one formula both front ends evaluate.
  **What to do**: every existing `ev_mode: relative` config (the default) re-renders at a
  different brightness — measured across a calibration corpus, from -2.02 to +2.55 stops. Two of
  the shifts are intentional and worth knowing rather than raising EV to undo: a narrow field of
  view no longer auto-brightens relative to a wide one, and changing `sim_resolution` (or,
  interactively in the GUI, resizing the preview) no longer changes the picture's brightness.
  `absolute` mode is unaffected — it was already anchored independently of the render's own output.
- **`render[].grid.central` is renamed to `render[].grid.angular_dist`** (#292), and the matching
  `LUMICE_RenderParam` fields to `angular_dist` / `angular_dist_count`. The old name never said
  what the number is (the angular distance from the sun). `grid.central` keeps loading as an alias
  forever (the new key wins if both are present), so no existing JSON config is rejected.
  `LUMICE_API_VERSION` moves 416 -> 417 — a source-compatibility break only: `sizeof` and every
  field's offset are unchanged, so a caller that does not name the field directly is unaffected.
  **What to do**: a C/FFI caller naming the struct field, or using a designated initializer for it,
  fails to compile until renamed to `angular_dist[_count]`; recompile. No JSON change is required
  unless you want to adopt the new key name.
  Separately, and with no ABI half: a config carrying `grid.central` (now `grid.angular_dist`) or
  `grid.elevation` entries used to have them parsed, validated and round-tripped without ever being
  drawn — the CLI draws both now (see the annotation-layer entry under Added), so a config that
  already set either one renders differently from before.
- **Automatic CPU worker count is capped at 10** (#314), down from the machine's full physical
  core count. Measured across two machines (16-core and 12-core) and two scenarios, throughput
  peaked at 10 workers regardless of core count, so adding more only bought scheduling overhead —
  as much as +61.6% throughput recovered by capping on the 16-core machine.
  **What to do**: nothing, unless you were relying on this project defaulting to more than 10
  workers on a many-core machine — pass `--workers N` (CLI) or set the personal default (GUI, see
  Added) to opt back into a higher count; an explicit value is honoured verbatim and is not subject
  to the cap.

### Fixed
- **A crash on selecting one of the last three lens types** (#289). `kLensTypeJsonNames[]` had only
  8 entries for 11 `LensType` values, with no `static_assert` to catch the mismatch, so serializing
  a document with `Fisheye Orthographic`, `Dual Fisheye Orthographic` or `Globe` selected read past
  the array — a SIGSEGV in the observed build, and on another build potentially a silently
  corrupted lens type written to the saved file. Reachable via Save, opening Settings, or saving a
  user default. The array is now complete and `static_assert`-guarded, and an unrecognized stored
  value falls back to `linear` with a visible notice instead of silently defaulting.
- **A raypath like `3-5,1-2` silently computed the wrong path instead of being rejected** (#284).
  `,` was the raypath separator before `-` replaced it years ago, and survived undocumented as a
  read-compatibility fallback; typing it in the GUI's raypath field normalized `3-5,1-2` into
  `3-5-1-2` — a single four-face path where the user meant two separate three-face paths — with no
  validation error, since every face number involved happened to be legal. The editor now rejects
  raypath text containing `,` with a message naming the correct separators (`-` within a path, `;`
  between paths); a `.lmc` written before this fix is migrated automatically on load (comma -> `-`
  within each raypath token; `entry:`/`exit:` face-list commas are untouched), with a log entry per
  document that needed it.
- **An exported config could describe a different picture than the one on screen** (#288). The
  GUI's export path shared its config-building code with the path that commits a scene to
  simulate, but only `intensity_factor` and `ev_mode` were actually filled in for export — lens
  type, field of view, view pose, `visible`, resolution, background colour and the horizon toggle
  were all left at their internal simulation defaults instead of the values on screen, and in the
  horizon case this meant an exported document could draw a line the user had explicitly turned
  off. The export path is now filled in field by field from the same GUI state the preview reads
  (three necessary conversions — camera roll, background colour space, resolution — are documented
  at their call sites), checked by a new same-document, two-renderer-path comparison test
  (`test/gui/parity/test_gui_cli_export_parity.cpp`). A config front-clipped (`front`) had no core
  equivalent at the time this shipped and is rejected on export with a warning rather than being
  silently mis-encoded; #292 (below) later gives `front` a real core field, replacing that
  placeholder. A GUI ray-tint control that never had any effect (changing it re-simulated but
  altered no pixel) is removed as part of the same cleanup.
- **GUI preview brightness for non-equal-area projections did not match the CLI** (#290, #296). The CLI
  bins per-pixel energy directly onto its target lens (a genuine camera measurement, `~ L*Ω_p`);
  the GUI's preview shader resamples a fixed equal-area texture without multiplying by the target
  lens's own per-pixel solid angle, so for any lens whose solid angle is not constant across the
  frame — every projection except the equal-area family — the two disagreed by up to 0.79x at
  frame centre and 1.35x at the edge (measured, `linear`, 160 degree FOV). The preview shader now
  applies that lens's relative illumination, so non-equal-area GUI previews gain the natural
  vignetting the CLI already had; equal-area previews are pixel-identical (their solid angle is
  constant, so there was nothing to add). A same-cycle regression this introduced — the saved-`.lmc`
  and composite (raypath-colour) preview paths correctly showed the new vignetting live but lost it
  after Save -> Open, because those paths bake an 8-bit texture that never carried it — is fixed
  together with it: `.lmc` and the composite bake now store plain radiance (no background, no
  vignetting), and both are applied uniformly at display time, so what a document shows after
  reopening matches what it showed when saved. A `.lmc` file saved before this fix keeps rendering
  without the vignetting on reopen, since the background colour it would need to subtract back out
  was already irreversibly baked into its pixels.
- **Fisheye renders were clipped to a 90-degree hemisphere regardless of the lens's own field of
  view** (#291). Core computed content only for angles up to 90 degrees from the view axis for four
  of the five single-lens fisheye variants, while the GUI preview's own domain check allowed up to
  180 degrees; a lens configured for a wider field of view showed correct sky in the GUI preview and
  a black ring past the 90-degree radius in the CLI. Core's domain is now widened per lens to match:
  equal-area and equidistant to the full 180 degrees, stereographic to 179.5 degrees (its radius
  formula diverges exactly at 180); orthographic is intentionally left at 90 degrees, since past
  that point its own radius formula folds back on itself and stops being invertible. The
  dual-fisheye family already matched the GUI at 180 degrees and is unaffected.
- **Three further CLI rendering behaviors did not match the GUI preview** (#297): a `rectangular`
  render honoured only the camera's azimuth, silently ignoring any configured elevation or roll; the
  `visible` hemisphere clip was applied only to single-lens renders, so a dual-fisheye, rectangular
  or globe renderer left the large majority of its nominally hidden pixels still carrying energy;
  and the forward energy-binning pixel-center convention differed by half a pixel from the
  convention the render-domain mask and viewport already used. All three are now aligned with the
  GUI and with each other. An existing config using a `rectangular` lens with non-zero elevation or
  roll, or any lens combined with `visible: upper`/`lower` outside the single-lens family, renders
  differently (and correctly) as a result; the pixel-center change is a sub-pixel shift.
- **GUI preview showed a faint double image straddling the horizon in dual-fisheye scenes** (#302).
  A pixel-binning convention change had updated the render-domain mask and the viewport but missed
  a third place using the same convention — the preview shader's own source-texture sampling —
  which doubled an existing half-texel offset already contributed by bilinear filtering. The two
  fisheye source discs sample with opposite-signed offsets, so the error mixed pixels from two
  different points in the sky into the equator overlap band, visible as a roughly 0.26-degree-wide
  horizontal ghosting band a few degrees above and below the horizon. CLI renders were unaffected
  (no resampling step there).
- **Screenshot export silently dropped overlay text labels by default, and drew them at half size
  on a Retina display** (#304). A separate, unsaved "Include Overlay in Screenshot" toggle defaulted
  to off and controlled only the text, while overlay lines exported unconditionally — so the
  exported PNG usually had grid lines with no numbers next to them, despite the menu item's name
  implying an all-or-nothing choice. Separately, the export path hardcoded a 1:1 device-pixel ratio
  when placing label glyphs (while getting label position right), so on a 2x Retina display exported
  text rendered at half the size it appeared on screen. Both are fixed by rendering the screen
  preview and the export from one shared off-screen path (image, lines and labels together, at the
  actual output device-pixel scale); the separate toggle no longer exists — turning labels off is
  now the same Overlay-panel switch that already controls the on-screen preview.
- **A batch of traced rays could be silently dropped under a rare scheduling stall** (#300). The
  server published a batch of ray data to the consumer thread *before* recording it in its own
  pending-batch counter; if the producer thread was preempted for as little as 0.2-1 ms between
  those two steps, the consumer could see the batch (with the counter still reading zero), discard
  it as unexpected, and the counter's later increment left no trace that anything had been lost.
  Affects the CLI and the GUI equally. The counter now increments before the batch is published, so
  the ordering that made this possible no longer exists.
- **A `--benchmark` run could report a physically impossible throughput** (#312), typically when
  `ray_num` was smaller than the internal drain quantum. In that case no internal "steady" sample
  window ever formed, and the benchmark's fallback path could end up dividing the full ray count by
  however long it took the poller to next observe the run go idle — a quantity that measures poll
  latency, not trace time — producing rates of several hundred million rays/second on a run that
  took milliseconds either way. The fallback now divides by the run's actual wall-clock duration
  instead, a true (if conservative) lower bound.
- **A CLI overlay text label could be cut in half at the edge of a narrow field of view** (#310).
  The GUI already clamped a label's position to stay fully inside the viewport; the CLI renderer
  drew the raw anchor `LUMICE_ComputeAnnotationOverlay` returned, which for a curve entering the
  visible area at the frame's edge is exactly the edge itself. Both front ends now share one
  clamping implementation (`src/util/label_viewport_clamp.hpp`).
- **A hit-loop buffer overflow could crash the legacy CPU backend at small dispatch sizes** (#314).
  `LUMICE_DISPATCH_RAY_NUM` at or below 32 crashed deterministically in multi-scattering scenes; a
  hit-loop buffer's sizing left zero headroom for one code path's fan-out, corrupting the heap. The
  buffer's capacity contract is now a single owner with an explicit precondition check.
- **Reference-point markers were mirrored on `rectangular` renders, and their section header could
  steal a click meant for a marker beneath it** (#305). The screen-position conversion the markers
  share with overlay labels had an inverted vertical axis specifically for the `rectangular` lens —
  invisible until this change, since the previous zenith/nadir-only pair is symmetric under that
  flip and so could never show it; and the "Reference Points" section header was missing a flag that
  let a click reach a widget drawn on top of it. Also, a marker's name-to-ring spacing on a HiDPI
  display was off by the display's own scale factor, since a device-pixel radius was being added
  directly to an already-logical-space anchor point.

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
- **`LUMICE_ComplexComposition` becomes a pointer-and-count** (#202). Its sub-clause array —
  introduced as an independent, fixed-size pool by the Complex filter's flat reference encoding
  in 4.3.4 — is replaced by a heap-allocated buffer managed through `Create`/`Release` calls, the
  same pattern `raypath_color` established in 4.3.5, and what makes the 4096-clause ceiling above
  possible without inflating every config on the stack.
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
  files and core JSON configs.

### Changed
- **Multi-scattering layer `prob` footguns are now guarded in the GUI** (#169). Setting the
  *last* layer's continuation probability above 0 silently discarded every ray that "continued"
  past it — there is no next layer to receive them — and setting a *middle* layer's probability to
  exactly 0 silently starved every layer after it. The last layer's slider now locks at 0 (with a
  warning if a loaded config set it otherwise, so you can still change it back), a zero
  middle-layer probability shows a warning instead of passing silently, and adding a new layer
  promotes the old last layer's probability to a sane default (0.8) instead of leaving it at the
  near-zero value a final layer would have had.

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
- **A legacy hand-written discrete-spectrum config lost its spectrum when loaded** (#168, #173).
  Importing such a config silently dropped the discrete spectrum and fell back to the default, so
  the render you got was not the one the file described. It now loads as written.
- **Filters with multiple OR'd raypaths or several entry/exit conditions stalled the GUI while
  dragging** (#172). Programmatic filter commits had two paths — a fast typed-struct path and a
  slow, string-based JSON path — and any filter beyond a single plain raypath fell onto the slow
  path, which could not tell the GUI a lightweight update was possible; every slider drag forced a
  full filter rebuild that stalled the live preview. All GUI filter commits now go through the
  fast path.
- **Switching between the CPU and GPU backend and clicking Run left the previous backend's stale
  frame on screen** (#172). Reconstructing the server for the new backend reset its epoch counter
  to zero, but the GUI's anti-flicker display fence carried the old backend's higher epoch across
  the swap and kept refusing the new backend's first frames as "stale."
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

### ⚠️ Breaking Changes
- **The default worker count changes from `hardware_concurrency - 2` to the number of physical
  CPU cores** (#120). The simulator is memory-bound with a dedicated consumer thread; the old
  default packed workers onto SMT/hyperthread siblings, contending for cache with each other and
  starving the consumer — measured ~48% throughput loss on a common 8-core/16-thread desktop.
  **What to do**: nothing, unless you depend on the exact previous thread count (a benchmark
  script, a machine shared with other workloads) — set `num_workers` explicitly to pin it; a
  config that already sets `num_workers` is unaffected either way.

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
  were set at all (#131 — an unrelated fix carried on the same branch as the throughput-honesty
  work cited above, not a duplicate reference to it).

## [4.2.8] - 2026-06-01

### Added
- **Entry/Exit filters gain wildcards, multi-value OR, and path-length bounds** (#118). Leaving
  the entry or exit face blank now means "any face" (previously `0` was indistinguishable from
  "match face 0"); an entry or exit can match a set of faces (`entry={3,4}, exit={5,6}`); and a
  filter can additionally require a minimum and/or maximum hit-path length (unbounded / exactly N
  / at most N / a range). Existing single-value `{"entry": 3, "exit": 5}` configs are unaffected.

### Changed
- **The auto-EV brightness anchor moves from the 99.5th to the 99th percentile, and is now computed
  on a coarse downsampled grid rather than the full-resolution frame** (#116). On the intentionally
  sparse `77halo` scene (`ray_num: infinite`), the anchor statistic drifts monotonically with
  accumulation because the lit-pixel set never saturates — this is an intrinsic property of a
  percentile taken over an ever-growing lit set, not a bug, and no anchor choice makes it fully
  stable. The downsampled, lower-percentile anchor is a mitigation, not a fix; ordinary scenes are
  unaffected in practice (25-scene regression: 22/25 pixel-identical to the prior anchor).
- **Adaptive Brightness no longer holds onto a filter-independent EV anchor** (#115). The
  filter-independent `anchor_p995_y` / `anchor_snapshot_intensity` statistic introduced in 4.2.6
  (see that version's Breaking Changes entry) traded multi-scattering throughput for EV stability
  across filter toggles; beta feedback found the stability rarely mattered while the throughput
  cost was paid on every multi-scattering render. Brightness is now a straight per-frame P99 of
  the visible framebuffer, so **the exposure can shift when you toggle a filter** — accepted as
  the right trade since filter A/B brightness comparison is rare and multi-scattering is common.
  Filter-fail rays terminate immediately instead of completing a full multi-scattering trajectory,
  restoring the throughput a filter is supposed to buy: +74% at `ms_prob=0.5` and +114% at
  `ms_prob=0.8` (multi-worker, filter-on vs filter-off, macOS).

### Removed
- **The unused `norm_mode` config key is dropped** (#116). Its only live branch (`total_pix`) is
  now unconditional; the other option (`effective_pix`) had never actually been wired up, so no
  existing config's rendered output changes. A config that still sets `norm_mode` keeps loading —
  nlohmann JSON ignores unknown keys — the key is just silently ignored from now on.

### ⚠️ Breaking Changes
- **`LUMICE_RawXyzResult::anchor_p995_y` and `anchor_snapshot_intensity` are removed** (#115),
  shrinking the struct from 64 to 56 bytes on 64-bit platforms. These carried the
  filter-independent brightness statistic introduced in 4.2.6; there is no replacement field,
  since the new normalization (above) is computed entirely from the already-published
  `snapshot_intensity`. **What to do**: recompile against the new header; drop any code that reads
  `anchor_p995_y` / `anchor_snapshot_intensity`.

## [4.2.7] - 2026-05-28

### Changed
- **`max_hits` now accepts values up to 64, not just 8** (#110). A raypath filter or a scene that
  needs to record a longer hit sequence than 8 can now ask for it; the config loader validates the
  field stays within `[1, 64]`.
- **The `Front` visibility option becomes an independent checkbox instead of one radio-button
  choice among Upper/Lower/Full/Front** (#111), so it can be combined with the others (e.g.
  "Upper + Front"). `RenderConfig::front` and a dedicated shader uniform now drive the clip; a
  `.lmc`/JSON file with the old `"visible": "front"` value still loads, migrated on read to
  `visible: full, front: true`.
- **Clicking anywhere on an entry card's blank area opens its edit modal** (#111), instead of only
  a specific button; clicking the already-open card's own area is a no-op so it can't discard an
  in-progress edit.

### Fixed
- **The GUI preview at high resolution could lag well past one frame per VSync tick** (#113).
  Uploading the rendered texture to the GPU and computing the P99.5 brightness statistic both ran
  synchronously on the main thread every frame — at 4096×4096 the texture upload alone moved up to
  384 MB — together blocking 25-45 ms against a 16 ms budget. The texture upload now goes through
  an asynchronous double-buffered PBO, and the brightness statistic is computed on the background
  polling thread instead of the main thread.
- **A slider drag outside its own valid range could mark the document dirty without changing
  anything** (#110). The dirty check now compares the slider's actual value instead of trusting
  ImGui's raw `changed` flag.
- **Toggling a filter could leave a stale brightness anchor in the live preview, and an
  all-black frame could fail to display** (#110). The GUI now clears its anchor fields on a filter
  change and accepts a legitimately zero-intensity frame from the poller instead of dropping it.
- **A hover tooltip on a multi-part control only responded over its last sub-widget, not the whole
  control** (#114). Affected sliders that combine a drag control with a text-entry box now show
  their tooltip anywhere over the combined widget.

## [4.2.6] - 2026-05-25

### Added
- **Linked entries**: two or more entry cards can now explicitly share the same crystal and/or
  filter definition, so editing one edits every linked sibling (#103). A `Link to...` action enters
  a pick mode — click another card to bind to it — and a linked group shows a chain-link badge with
  a shared-highlight while its edit modal is open. `Unlink` forks a linked card back to an
  independent copy; `Duplicate` always creates an independent copy. `.lmc` files are unaffected —
  linking is a GUI-session concept that gets flattened to inline values on save.
- **Zenith/nadir marker overlay**: an optional ring marks straight up and straight down in the
  preview, at a constant on-screen size regardless of lens type or zoom (#106).
- **FontAwesome icons replace 12 single-character GUI buttons** (delete, duplicate, panel-toggle,
  Run/Stop, info, OK/Cancel, and others) that previously fell back to an ASCII glyph like `×` or
  `(i)` (#102).
- **New public C API**: `LUMICE_XyzToSrgbUint8`, a batched XYZ-to-sRGB conversion entry point
  (#101), factored out of an internal GUI helper so a caller no longer has to reimplement the
  conversion to interpret raw XYZ results.

### Changed
- **Filter routing reverts to gating ray emission in the simulator, undoing 4.2.5's
  consumer-side design** (#104). The consumer-side redesign introduced two regressions — a
  layer with continuation probability 0 could leak rays past it, and a scene with more than one
  active filter could render completely black — both traced to the same missing (layer, crystal)
  binding. Reverting is the documented original design; see the new `doc/filter-architecture.md`
  for the rationale this time, so the constraint doesn't have to be rediscovered again. As a
  consequence, the "unfiltered" buffer's brief life as a genuinely filter-independent readout (see
  4.2.5's breaking-change entry) ends here — see the ABI note below.
- **Adaptive Brightness drops its on/off toggle and always applies** (#105). The dual on/off mode
  added in 4.2.4 is replaced by one always-on pipeline (degenerating to a plain filtered readout
  when no filter is active), and its brightness anchor moves from the 99th to the 99.5th
  percentile with `target_white` changed from 200 to 135 — chosen after comparing 9
  `(percentile, target)` combinations across 9 scenes for perceptual balance.
- **A rendered scene exported via `Save → Screenshot` no longer comes out dimmer than what the
  live preview showed** (#105, see Fixed below).

### ⚠️ Breaking Changes
- **`LUMICE_RawXyzResult`'s `unfiltered_xyz_buffer` / `unfiltered_snapshot_intensity` fields are
  replaced by `anchor_p995_y` / `anchor_snapshot_intensity`** (#104, #105). 4.2.5 made the
  "unfiltered" buffer genuinely filter-independent by changing how the simulator routes
  filter-failed rays; this release reverts that routing (see Changed above), which makes the old
  "unfiltered" fields meaningless again — they are removed rather than left as dead weight. In
  their place, an explicit brightness-only anchor (P99.5 over filter-pass + filter-fail
  combined) drives the GUI's auto-EV so brightness still stays stable when toggling a filter.
  **What to do**: recompile against the new header; a caller reading the old `unfiltered_*` fields
  must move to the new `anchor_*` pair, which serves the brightness-stability use case only (it is
  not a substitute readout of "what would this scene look like with no filter applied").
- **`LUMICE_ServerConfig` gains a `sim_seed` field, and the `LUMICE_SIM_SEED` environment variable
  is no longer read** (#109). A non-zero seed now forces single-worker execution for bit-stable
  results, set through the C API instead of an environment variable. **What to do**: recompile
  against the new header; a caller (or CI script) that previously relied on `LUMICE_SIM_SEED` for
  a deterministic run must set `LUMICE_ServerConfig::sim_seed` instead — the environment variable
  is now silently ignored, not an error.

### Fixed
- **A server process could crash after roughly 31 create/destroy cycles when alternating between
  three different configs** (#100). Three `LUMICE_Get*Results` C API functions wrote one
  `LUMICE_*Result` element past the end of the caller's array whenever the result count exactly
  filled it; the array is caller-stack-allocated (`src/main.cpp`), and ASan confirmed the
  overwrite as a stack-buffer-overflow. In production it manifested as a SIGSEGV once the stack
  layout had shifted enough, after roughly 31 lifecycles, for the overwrite to land on live data.
- **A screenshot exported via `Save` could be noticeably dimmer than the on-screen preview of the
  same frame** (#105). The export path was missing the auto-EV and anchor/filtered path selection
  that the live preview already applied; both paths are now built from the same brightness
  pipeline.
- **A ray that failed a filter partway through a multi-scattering sequence could still leak into
  the final rendered image** (#108). The per-ray "failed a filter in an earlier layer" state
  wasn't tracked across layers, so on a probability-based path continuation such a ray could still
  land in the main output instead of being routed away from it.

## [4.2.5] - 2026-05-20

### Changed
- **A scattering entry's own filter is now also used to gate the simulator's live "unfiltered"
  readout** (#93), so `LUMICE_RawXyzResult::unfiltered_xyz_buffer` genuinely reflects the full,
  pre-filter ray set its documentation always claimed rather than the filtered set under a
  different name. See the breaking-change note below for the display-side consequence, since this
  entry is superseded by 4.2.6's revert.
- **The crystal-card slider labeled `prop.` is now labeled `Weight`** (#99), and the Adaptive
  Brightness panel's `Target` slider is removed — its value stayed at the default in practice, and
  user feedback found it added a control without a useful range to tune.
- **The raypath filter text field accepts up to 4096 characters, not 256** (#97), so a filter
  OR-combining dozens of long raypaths no longer gets silently truncated at the input box.
- **The Pyramid Upper/Lower Height preset combo shows three decimal places, not one** (#97), e.g.
  `28.000°` instead of `28.0°`.

### ⚠️ Breaking Changes
- **A scene combining multi-scattering with a `scattering.entries[].filter` renders its "OFF-mode"
  live-preview brightness from the unfiltered ray set instead of the filtered one** (#93), matching
  the same anchor the auto-EV path already used. Most scenes are visually unaffected, but a scene
  whose filter passes only a small fraction of rays (measured example: a `raypath=[4,6]` filter at
  a ~0.1% pass rate) can darken by roughly 10 stops compared to 4.2.4. **What to do**: if a scene
  looks unexpectedly dark after upgrading and uses a low-pass-rate filter, adjust EV manually; no
  config change is needed to restore the old look, since 4.2.6 reverts this routing again.
- **`LUMICE_CrystalMesh` gains 5 per-face fields** (`face_count`, `face_numbers_by_face`,
  `face_vtx_offsets`, `face_vtx_counts`, `face_vtx_pool`) and two new `LUMICE_MAX_CRYSTAL_FACES` /
  `LUMICE_MAX_CRYSTAL_VTXPOOL` constants (#95), letting a consumer read a crystal's faces (each as
  a CCW-ordered vertex loop) directly instead of re-deriving them from the raw triangle soup.
  **What to do**: recompile against the new header; a caller that doesn't read the new fields is
  unaffected (`face_count == 0` is the old shape).

### Fixed
- **A ray that grazed the shared edge between two faces after a total-internal-reflection off a
  raypath `[1,3]` face could escape out the far side of the crystal instead of continuing to
  reflect** (#92). The edge case was indistinguishable from a legitimate "ray floating off its own
  source face" using the tolerance's sign alone, so fixing one direction of the ambiguity kept
  reopening the other; resolved by tracking which face a ray actually came from and applying a
  different tolerance to that face than to every other candidate face.
- **A scene mixing a raypath filter with multi-scattering could render at roughly 1/6 the ray
  throughput of an equivalent scene without the filter's `D` symmetry variant** (#98). Matching a
  filter against a ray mutated shared per-crystal symmetry state on every single ray; filter
  matching is now a pure, allocation-free comparison against a precomputed canonical form.
- **A filter using the `D` (180°-roll-independent) symmetry variant against a pyramid crystal could
  fail to match raypaths it should have** (#98), because the reflection it relies on was skipped
  on pyramid (cone) faces.
- **Two raypaths that are genuinely the same up to symmetry could reduce to two different
  canonical forms** (#98) when reduction crossed a `D`-symmetry step, breaking the invariant that
  the same orbit always reduces to the same representative.
- **The Pyramid Upper/Lower Height slider rows were about 20px narrower than the Height row above
  them** (#97), from a leftover width-adjustment term in the slider layout formula.

## [4.2.4] - 2026-05-14

### Added
- **Adaptive Brightness (auto-EV)**: the GUI can automatically choose exposure from a P99
  brightness anchor over the rendered frame, on by default, with a manual on/off toggle
  (#89). When a filter is active, brightness is anchored against the unfiltered ray set so
  toggling a filter on and off doesn't itself change perceived brightness — the dedicated
  "unfiltered" C API fields this depends on ship in this release (see below).
- **New public C API**: `LUMICE_MAX_ID`, `LUMICE_CrystalKind` + `LUMICE_IsLegalFace`,
  `LUMICE_RaypathValidationState` + `LUMICE_ValidateRaypathText`, `LUMICE_LensType` +
  `LUMICE_MaxFov`, and a new `LUMICE_ERR_UNKNOWN` error code (#90) — pure additions, factored out
  of internal GUI logic so it no longer needs to reach past the C API into core headers; nothing
  changes for an existing caller.

### Changed
- **Raypath symmetry matching is redesigned for correctness**: the `D` (roll-mirror) filter
  operator now uses a closed-form roll-bucket calculation instead of an approximate scheme, and a
  GUI tooltip explains when `D` isn't applicable to the current crystal (#88).

### ⚠️ Breaking Changes
- **`LUMICE_RawXyzResult` gains `unfiltered_xyz_buffer` and `unfiltered_snapshot_intensity`
  fields** (#89), the readout Adaptive Brightness's filter-independent anchor (above) is built on.
  **What to do**: recompile against the new header; a caller that never uses Adaptive Brightness is
  unaffected.

### Fixed
- **The `B` (mirror) filter symmetry operator did not correctly swap pyramid faces** (#88), so a
  filter relying on `B` symmetry against a pyramid crystal could match the wrong raypaths.
- **Sampling a crystal orientation near the poles with a negative-mean Rayleigh (or a mean below
  the equator) distribution could fold into the wrong hemisphere or the wrong azimuth** (#88), a
  latent bug in the same routine that couples roll to latitude-folding.

## [4.2.3] - 2026-05-07

### Added
- **Globe lens**: a new full-sky projection with trackball-style drag to look around, azimuth wrap,
  and overlay labels that respect back-hemisphere depth culling (#83).
- **The filter editor splits into a dedicated subpanel per filter type** (Raypath / Entry-Exit /
  Direction / Crystal) instead of one shared text buffer, so switching a filter's type no longer
  discards values already entered for another type (#85). `.lmc` files gain a v2 track for the new
  shape; existing v1 files still load.
- **Overlay Line and Label are independent toggles per overlay** (Horizon / Grid / Angular
  Distance), instead of one combined on/off switch, so you can show lines without labels or vice
  versa (#82). "Sun Circles" is renamed "Angular Distance" in the UI (the underlying config key is
  unchanged). A `.lmc` with the old combined key still loads, turning both Line and Label on.
- **The entry card whose edit modal is currently open is visibly highlighted** (#84), so with
  several cards open in sequence it's clear which one the modal belongs to.

### Changed
- **The View panel is regrouped into Lens / Visibility / Pose sections** (#86), and the
  Visible selector becomes a row of checkboxes instead of a combo box; `Front` is disabled while
  the Globe lens is active (Globe has no front/back distinction), and every visibility control is
  disabled in Full-Sky mode.
- **Switching between the Globe lens and any other lens keeps the camera pointed at the same real
  direction** instead of jumping (#86): crossing the boundary applies a self-inverse
  azimuth/elevation transform, with elevation clamped to ±89° on entering Globe to avoid a
  degenerate view matrix.
- **A combo box opened from inside a detached Edit Entry modal now renders above the modal**
  instead of behind it (#81), a z-order gap specific to a modal dragged into its own OS window.
- **The Visibility row lays out its four options on one horizontal line** instead of a 2×2 grid
  (#87), and the Entry-Exit filter subpanel's field styling (validation coloring, labels, a
  Remove Filter button) is aligned with the Raypath subpanel's.

### Removed
- **The Direction filter type is removed from the GUI's filter-type selector** (#87). The core
  JSON path is unchanged — a hand-written `"type": "direction"` filter still loads and behaves the
  same — but the GUI editor no longer offers it, and an existing `.lmc` containing one degrades to
  an empty Raypath filter with a warning when reopened.

### Fixed
- **The Entry-Exit filter's crystal-card summary showed `?` instead of an arrow** (`EE:2?5`
  instead of `EE:2->5`) (#87). The bundled font has no glyph for `→` (U+2192); replaced with the
  ASCII `->`.

## [4.2.2] - 2026-04-29

### Added
- **Orthographic lens projection** (#75): a new lens type alongside the existing fisheye/linear
  family, wired through the core projection math, server dispatch, and GUI shader branch.
- **The edit modal's Crystal/Axis/Filter tabs can switch between a horizontal and a vertical
  layout** (#75), persisted per-document in `.lmc`.
- **The Edit Entry modal can be dragged out into its own OS window** (#75), via ImGui multi-viewport
  support; the previous docked-in-main-window behavior is unchanged when it isn't detached.
- **A too-small window now shows an inline warning when a fixed aspect-ratio preset can't be
  honored** (#79), instead of silently clamping to a different ratio than the one selected.
- **Overlay labels gain an interior placement for wide-FOV, non-linear lenses** (#79): a
  `fov=180` fisheye/rectangular/etc. scene now still shows latitude labels even where the
  projected sky disc is smaller than the viewport.

### Changed
- **Crystal orientation follows the "HaloRay v1" rotation convention** instead of this project's
  previous ad hoc chain (#76). The rotation chain becomes `Rz(azimuth − 180°) · Ry(−zenith) ·
  Rz(roll)`, chosen so preset poses (e.g. Plate's default face-up, Parry's default column axis)
  match the convention used by the HaloRay reference tool; the modal preview's Reset View and the
  entry-card thumbnail now derive from one shared default-view formula instead of two.
  `doc/coordinate-convention.md` documents the frame, the convention, and the `azimuth − 180°`
  offset. See the breaking-change note below — **this changes what an existing config's
  `crystal.axis.{zenith, azimuth, roll}` values produce**.
- **The modal preview's trackball drag rotates the crystal in world coordinates** (#76), so a
  horizontal drag always reads as a horizontal rotation regardless of the crystal's current pose,
  matching the behavior of dragging in real space rather than in the crystal's local frame.
- **Overlay labels stay anchored to the main viewport when the window is resized or moved** (#78),
  instead of drifting relative to it.

### ⚠️ Breaking Changes
- **A crystal orientation config (`crystal.axis.zenith` / `azimuth` / `roll`) can render at a
  different pose than before**, because the rotation convention itself changed (#76, see Changed
  above). There is no simple per-field conversion — the chain's structure changed, not just an
  offset on one axis — so **what to do**: after upgrading, re-check any saved orientation against
  the new convention documented in `doc/coordinate-convention.md`, or re-pose visually in the GUI
  and re-save. A distribution that samples azimuth and roll uniformly over the full circle (the
  common case) renders statistically unchanged.

## [4.2.1] - 2026-04-23

### Added
- **Crystal face numbers can be overlaid directly on the 3D preview mesh** while rotating a crystal
  in the edit modal (#73), using the same numbering as raypath filters (basal 1/2, prism 3-8,
  pyramidal 13-18/23-28), and hidden on faces pointed away from the camera.
- **The Edit Entry modal can be popped out of the main window in Immediate mode** (#72, #73):
  outside clicks pass through to the app underneath, and only an explicit close action dismisses
  it, unlike a normal staged modal.

### Changed
- **The Crystal/Axis/Filter edit modal keeps a live 3D crystal preview visible next to whichever
  tab is open** (#73), instead of the preview only appearing on the Crystal tab; the modal is
  clamped to stay fully on-screen across multiple monitors.
- **Every export path (Dual Fisheye, Equirectangular, `.lmc`/PNG/JSON `Save`) now renders through
  one shared off-screen pipeline** (#72), replacing several separate ad hoc pixel-readback
  implementations; the `Save` menu's `Panorama` option is renamed `Dual Fisheye Equal Area` and
  gains a new `Equirectangular` choice, with `Include Texture` / `Include Overlay` checkboxes.
- **Overlay labels no longer render behind an open modal** (#72): they now draw on the window's own
  draw list instead of a layer a modal could occlude.
- **The Face Distance and Pyramid Upper/Lower Height sliders cover a wider, purely linear range**
  — `[0, 2]` and `[0, 1]` respectively (#73) — replacing a narrower, non-linear scale.
- **Background panels no longer steal keyboard/mouse focus order from an open Edit modal** (#73).
- **Removing a filter is simplified to clearing its text field** (#74): the previous multi-step
  "pending removal" flow (with its own Undo state) is replaced by one rule — an empty raypath
  field means no filter.
- **A half-typed or invalid raypath can no longer be committed while editing in Immediate mode**
  (#74): the commit path now rejects anything that isn't a fully valid raypath expression before
  writing it into the model, instead of writing through on every keystroke and blanking the render.

### Fixed
- **Closing the detached (Immediate-mode) Edit Entry modal via ×, the Close button, or Esc could
  leave an empty title bar behind** on screen (#74).
- **"Reset All" on a Crystal only restored some of its shape parameters, not all of them** (#74).

## [4.2.0] - 2026-04-19

### Added
- **Front-hemisphere visibility mode**: the visible-hemisphere selector gains a `Front` option
  that clips the rear hemisphere in fisheye projections (#67), with `.lmc` serialization and
  overlay-label support.
- **The raypath filter text field validates as you type**, showing a three-state (valid /
  incomplete / invalid) colored background instead of silently accepting arbitrary text (#67).
- **The Pyramid crystal's height slider can reach exactly zero** (#67): a new mapping is linear
  near zero and logarithmic for larger values, replacing a purely logarithmic scale that couldn't
  represent zero.
- **The filter editor rejects a face number that doesn't exist on the selected crystal kind**
  (#69) — e.g. entering prism face `13` now shows an inline, crystal-specific error instead of
  silently accepting it.

### Changed
- **The left and right panels are reorganized around collapsible layers of entry cards** (#68,
  #71). Crystal/Scene/Filter used to be separate tabs referencing crystals and filters by ID;
  entries are now cards (thumbnail, type, pose, filter, proportion) grouped into collapsible
  layers, edited through Crystal (with a live 3D preview), Axis, and Filter modals that are later
  unified into one tabbed dialog. Existing `.lmc` files still load; the underlying core JSON
  format is unchanged.
- **The EV slider's range widens from [-3, +7] to [-6, +6]** (#71).
- **The multi-scattering layer probability slider is disabled, with an explanatory tooltip, when
  only one layer exists** (#71), instead of accepting a value that has no effect.
- **An edit-modal tab with unsaved changes shows a trailing `*`** (#71), so which tab has pending
  edits is visible at a glance.
- **A newly created GUI document starts at 5M rays instead of 1M** (#67). A config file that
  already sets `ray_num`, and any existing `.lmc`, is unaffected — only the starting point for a
  document created from scratch moves.

## [4.1.14] - 2026-04-09

### Added
- **Pyramid crystals take a continuous wedge angle (0.1°–89.9°) instead of only discrete
  Miller-index presets**, with a sqrt-scale GUI slider plus a Miller-index preset dropdown for the
  old values; the prism-segment height slider can now reach exactly zero for a pure-cone crystal
  (#53). A JSON config with the old `upper_indices`/`lower_indices` keys still loads unchanged.
- **Auxiliary overlay lines**: horizon, altitude/azimuth grid, and sun angular-distance circles,
  each with configurable color and opacity, laid out in a new collapsible right panel that
  separates display options (View/Display/Overlay) from the config panels (#54, #55).
- **Angle labels appear where an overlay line crosses the viewport edge or a hemisphere
  boundary**, with collision avoidance between labels (#55).
- **The toolbar gains a ray-count input, an Infinite toggle, and double-click renaming for
  crystals and filters** (#58).
- **`RenderConfig::overlap` becomes a real config field** (default `0.0`) instead of a
  hardcoded GUI-only constant, so the CLI still produces exact hemispheres while the GUI's
  equator-blend overlap is explicit and threaded through the C API and JSON (#60).
- **A legacy Gaussian axis distribution** (`kGaussianLegacy`, without the Jacobian correction
  from 4.1.13) is available for reproducing simulation results from before that correction (#61).
- **Config crystal/filter/scattering-entry limits raise from 16 to 256** (#62).

### Changed
- **Face-distance labels in the GUI now match the crystal's internal face numbering** (3–8
  instead of 1–6) (#58).
- **Run/Stop toolbar buttons are colored green/red**, and the toolbar layout no longer jumps
  between simulation states (#58).
- **Left and right panel layouts are reorganized**: the sun-azimuth control is hidden, axis
  distribution controls move into a collapsible Advanced group, the crystal preview stays fixed
  at the bottom of the left panel with independently scrolling parameters, and combo/slider
  controls align consistently across the right panel's tabs (#56).

### Fixed
- **Overlay labels could false-positively cluster near near-tangent sun circles**: the
  crossing-detection epsilon tightens from 0.1 to 0.01, while still catching real fisheye
  disc-edge crossings (#56).
- **Ray allocation could starve a low-proportion crystal indefinitely**: a per-scalar rounding
  scheme could leave a crystal with a small proportion (e.g. 0.5%) without a single ray for many
  batches; allocation now carries a per-crystal remainder so every crystal gets rays within a
  couple of batches (#62).
- **A config with more than 16 scattering entries (or crystals, or filters) was silently
  truncated in the GUI** (#62) — see the config-limit increase above.
- **Deterministic ray allocation across crystals could occasionally miss the exact requested
  total, or leave a crystal with zero rays**, from rounding in a single-scalar Bresenham scheme;
  replaced with a cumulative-rounding scheme that guarantees an exact total (#59).
- **A filter change could leave a stale rendered frame on screen even though the new filter
  produced no results** (#59).
- **The GUI could show a false "no data produced" warning on startup**, from a stats buffer that
  was one element too small (#59).

### ⚠️ Breaking Changes
- **`LUMICE_CrystalParam.upper_indices[3]`/`lower_indices[3]` (integer Miller indices) are
  replaced with `upper_wedge_angle`/`lower_wedge_angle` (float degrees)** (#53). A Miller index
  like `{1,0,-1,1}` corresponds to a wedge angle of 28.0°; the GUI's preset dropdown performs this
  conversion for the common indices. **What to do**: recompile against the new header and set the
  wedge-angle fields instead; a JSON config file is unaffected — the loader still accepts the old
  `upper_indices`/`lower_indices` keys.
- **`LUMICE_RenderParam` gains an `overlap` field** (#60), the dual-fisheye equator-blend
  threshold. **What to do**: recompile against the new header; a caller that doesn't set it gets
  `0.0` (no overlap), matching the previous CLI behavior.
- **`LUMICE_MAX_CONFIG_CRYSTALS`, `LUMICE_MAX_CONFIG_FILTERS`, and
  `LUMICE_MAX_CONFIG_SCATTER_ENTRIES` raise from 16 to 256** (#62), substantially growing the
  in-memory size of `LUMICE_Config` and `LUMICE_ScatterLayer`. **What to do**: recompile against
  the new header — any code that assumed the old sizes (e.g. a fixed-size buffer sized off the
  old constant) needs updating.

## [4.1.13] - 2026-04-06

### Added
- **`LUMICE_ParseConfigString`/`LUMICE_ParseConfigFile` C API**: parse JSON into a
  `LUMICE_Config` struct for load-modify-commit workflows, for the subset of JSON that
  round-trips through `ConfigToJson` (#49).
- **`LUMICE_CreateServerEx` C API**: create a server with an explicit worker count via a new
  `LUMICE_ServerConfig` struct (#45).
- **CLI `--benchmark` flag**: prints a machine-readable `[BENCHMARK]` JSON line for a
  single-worker and a multi-worker pass, including per-core parallel-scaling efficiency (#45).
- **Two new crystal axis distribution types, zigzag (rectified arcsine) and Laplacian
  (inverse-CDF)**, each with correct spherical-Jacobian rejection sampling; the C API gains named
  `LUMICE_AXIS_DIST_*` constants for the existing and new distribution values (#52).

### Changed
- **The crystal-type selector is a set of radio buttons instead of a combo box**, and the
  face-distance editor becomes an inline collapsible "Advanced" group instead of a floating
  popup; the crystal preview is now square (#43).

### Fixed
- **The mouse wheel over the crystal preview scrolled the parent panel instead of doing
  nothing/scrolling the preview** (#43).
- **A raypath filter could leak rays through that shouldn't match**: hit-triangle 0 (the top
  basal face) was excluded by an off-by-one face-index check, truncating raypaths like
  `{3,1,5}` to `{3,5}` and letting a 46° halo pass a filter meant for the 22° halo (#44).
- **A filter's crystal-symmetry flags always serialized as `"PBD"` regardless of the actual
  setting**, from a bitwise-OR where an AND was needed; a composite filter also failed to
  propagate its symmetry setting to its sub-filters (#44).
- **`DirectionFilter` silently failed to filter at the scene level**: a redundant state guard and
  a coordinate-rotation ordering bug meant it never actually evaluated world-space directions
  correctly (#50).
- **Crystal orientation sampling (Gaussian, Rayleigh, and uniform axis distributions) lacked the
  `cos(φ)` spherical-area correction**, over- or under-representing certain latitudes, and a
  latitude-folding routine gave wrong results for large standard deviations. Existing configs
  using axis distributions will render with statistically corrected crystal orientations (#51).

### ⚠️ Breaking Changes
- **`LUMICE_InitLogger(LUMICE_Server*)` is removed from the C API** (#48) — it only ever set the
  global log level to `kInfo`, which now happens automatically when the logger is first
  constructed. **What to do**: remove any call to `LUMICE_InitLogger`; recompile against the
  updated header.

## [4.1.12] - 2026-04-04

### Added
- **The GUI's internal preview texture switches from equirectangular to dual equal-area
  fisheye**, eliminating the brightness non-uniformity artifact equirectangular projection
  produced near the zenith; CLI output is unaffected (#37).
- **CLI `--format <jpg|png>` and `--quality <1-100>` options** select the output image format
  and JPEG quality; the default (no-flag) output is unchanged (#38).
- **A floating editor for a prism crystal's 6 individual face distances**, with a
  live-updating 3D preview (#40).

### Changed
- **Field of view is measured against the frame's short edge instead of its diagonal**, with
  per-lens-type maximum FOV limits and matching GUI slider clamping (#39).
- **A fisheye projection fills the whole rectangular viewport instead of being cropped to a
  circle** (#39).

### Fixed
- **The dual-fisheye projection showed a visible seam at the equator**, where the two
  hemispheres met (#39).
- **Changing only a crystal's face distance, with geometry otherwise unchanged, was silently
  ignored** — a premature-optimization check skipped mesh regeneration in that case (#41).
- **Pyramid face-plane equations mixed the face-distance offset into the face's tilt angle
  instead of only shifting its position**, distorting the crystal shape whenever a non-default
  face distance was set (#41).

### ⚠️ Breaking Changes
- **`LUMICE_CrystalParam` gains a `face_distance[6]` field** (#40), carrying the new per-face
  distance editor's values. **What to do**: recompile against the new header; a caller that
  doesn't set it gets the previous default (uniform faces).

## [4.1.11] - 2026-04-01

### Changed
- **Windows GUI compute throughput improves substantially during interactive use**: a persistent
  thread pool, consumer reuse, pre-packed ray-filter data, and switching the app to the console
  subsystem (the window itself stays hidden on a normal launch) together close most of the
  throughput gap with the CLI (#33, #35).
- **The EV (exposure) slider no longer restarts the simulation** — dragging it just re-tones the
  accumulating frame (#31).
- **The Render panel is reorganized into Projection / View / Display / File groups**, and the
  resolution combo warns when a resolution may distort a projection (#31).
- **The raypath filter's text field uses `-` instead of `,` to separate hit values** (e.g.
  `3,5` → `3-5`); an old `.lmc` file using commas still loads (#36).
- **Crystal height and pyramid prism-height sliders use a log scale** covering 0.01–100, instead
  of a plain or sqrt scale (#36).

### Fixed
- **Rendered brightness depended on the output resolution and field of view, not just the
  scene**: the normalization formula is rebuilt from first principles so it no longer scales
  with `1/(width×height)` or counts culled rays in its intensity total. Existing configs will
  render at a different brightness (#31, #32).
- **An equirectangular render showed a visible seam at the ±180° longitude wrap** (#31).
- **A pyramid crystal with a Miller index greater than 1 simulated the wrong geometry** — an
  erroneous scaling factor is removed and Miller indices are normalized by their GCD (#32).
- **Round-tripping a config with the `kNoRandom` axis distribution through JSON could crash**
  (#32).
- **A background image path containing non-ASCII characters could fail to load on Windows**
  (#32).
- **Save (and Save As) could visibly corrupt the render preview** — saving now only refreshes
  the CPU-side texture cache instead of touching the GPU texture format (#36).

### ⚠️ Breaking Changes
- **`LUMICE_LOG_VERBOSE` is inserted into the `LUMICE_LogLevel` enum between
  `LUMICE_LOG_DEBUG` and `LUMICE_LOG_INFO`** (#33), shifting the integer value of every level
  from `INFO` onward. **What to do**: recompile against the new header; a caller that references
  levels by name is unaffected, one that hardcodes the old integer values is not.
- **`LUMICE_RawXyzResult` gains an `effective_pixels` field, and `LUMICE_RenderParam` gains a
  `norm_mode` field** (#32), backing a new (GUI-hidden, config-only) adaptive normalization mode.
  **What to do**: recompile against the new header; a caller that doesn't set `norm_mode` keeps
  the previous (absolute) normalization behavior.

## [4.1.10] - 2026-03-25

### Changed
- **Slider dragging becomes visibly smoother**: persistent worker and poller thread pools
  (replacing per-commit thread spawn/join) and reuse of the existing consumer when the renderer
  layout is unchanged together raise texture uploads during a drag from about 1 in 10 frames to
  roughly 1 in 2 (#30).

### Fixed
- **Clicking Stop and then Run again could leave the live preview stuck without resuming
  updates** (#30).

### ⚠️ Breaking Changes
- **`LUMICE_CommitConfigStruct` gains a third parameter, `int* out_reused`** (#30), reporting
  whether the existing consumer/renderer was reused instead of rebuilt. **What to do**: recompile
  against the new header and add a third argument to any existing call site — pass `NULL` if the
  reuse signal isn't needed.

## [4.1.9] - 2026-03-24

### Added
- **Ray count, Max hits, Probability, and Proportion sliders accept typed text input** for
  precise values, instead of only drag interaction (#27).

### Fixed
- **The displayed log file path didn't match its actual location** on the Windows fallback, and
  log writes could sit in the OS buffer for a while before appearing on disk — warnings now flush
  immediately and other logs flush at least once a second (#28).
- **The GUI could crash during fast slider dragging on Windows+NVIDIA** when VSync silently
  stopped limiting the frame rate (a known GLFW/driver issue), causing a busy-wait loop and
  event-queue buildup; a sleep-based fallback frame limiter now activates whenever VSync isn't
  effectively limiting (#29).

## [4.1.8] - 2026-03-23

### Added
- **`LUMICE_SetLogCallback` C API**: the GUI's logging is now sourced from the core via a
  callback instead of a separate logger instance, unifying GUI and core logging under one
  architecture (#26).
- **A collapsible GUI log panel** with a per-component level filter and file output (#26).
- **Axis Std/Range sliders use a sqrt-scale mapping**, giving about 6.7× more resolution at
  small values (0–2°) (#25).

### Fixed
- **X-Ray wireframe mode showed the wrong solid/dashed edges on a zoomed-in pyramid crystal** —
  edge front/back classification is now perspective-correct (#25).
- **The crystal preview didn't refresh after changing a Miller index** (#25).
- **Slider dragging could still show an occasional all-black preview frame**: the texture-hold
  logic that skips sparse early snapshots ran on the poller thread and could consume its
  generation counter without staging data; it now runs on the main thread with a GPU fence for
  upload synchronization (#26).

## [4.1.7] - 2026-03-21

### Added
- **`LUMICE_GetCachedStats` C API**: query the last simulation's stats without the overhead of a
  full snapshot recompute (#24).

### Changed
- **The live preview updates roughly 4× more often outside of slider dragging** (about 6→23
  FPS), by polling every 20ms instead of only once per 50ms commit cycle (#24).

### Fixed
- **Restarting a simulation via a slider drag could show a visibly flickering/dimmer frame**:
  the preview now holds the previous texture for 30ms after a restart, skipping the sparse early
  snapshots that caused the flicker (#24).
- **The GUI could briefly show a "Done" state right after a restart, before the new simulation
  had produced any rays** (#24).

### ⚠️ Breaking Changes
- **`LUMICE_RawXyzResult` gains `has_valid_data` and `snapshot_generation` fields** (#24), which
  the GUI uses to detect a genuinely new snapshot instead of guessing from polling cadence.
  **What to do**: recompile against the new header; a caller that ignores the new fields is
  unaffected.

## [4.1.6] - 2026-03-20

### Added
- **GUI logging via spdlog**, with independently configurable core/GUI log levels and a
  `--log-level` CLI option (#22).

### Changed
- **The Export button is disabled while a simulation is running** (#22).

### Fixed
- **Dragging a slider could starve the preview of new data and stutter**: the background
  worker's sleep is now interruptible (so Stop takes ~1ms instead of ~50ms) and a fast poll
  follows each restart (#22).
- **An occasional black-frame flash could appear during slider dragging** (#22).

## [4.1.5] - 2026-03-19

### Added
- **Export offers a format menu** — Screenshot, Panorama (equirect PNG), or Config JSON —
  replacing the single Export button (#21).
- **The Open dialog accepts `.json` config files** alongside `.lmc`, so a config exported as
  JSON can be reloaded directly (#21).

### Fixed
- **Dragging a slider quickly could crash the GUI**, from a race between the background poller
  and config commit (#21).
- **Rendered colors had a green tint** — the tone-mapping shader used the XYZ→RGB matrix's
  columns where it needed rows (#21).

## [4.1.4] - 2026-03-19

### Added
- **The GUI stays interactive while a simulation is running**: parameter panels remain enabled
  during a run and edits auto-commit instead of waiting for the run to finish (#20).

### Changed
- **Parameter edits during a running simulation take effect up to 4× faster**: the auto-commit
  interval drops from 200ms to 50ms, and a new C API path (`LUMICE_CommitConfigStruct`) commits a
  config struct directly instead of round-tripping through JSON — together raising slider-drag
  restarts from 18 to 84 per 5 seconds (#20).

### Fixed
- **Infinite-ray-mode renders could flash an intermittent all-black frame**, from the snapshot
  buffer being mutated between being read and being copied out (#20).

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

[4.5.0]: https://github.com/LoveDaisy/ice_halo_sim/compare/v4.4.3...v4.5.0
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
