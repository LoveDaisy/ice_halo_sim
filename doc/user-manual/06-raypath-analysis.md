[中文版](06-raypath-analysis_zh.md)

# Raypath Analysis

The Raypath Analysis window answers "which raypaths make the light in this region of the sky, and how much does each contribute". Unlike the main preview, it does not render a picture — it runs a dedicated, non-rendering pass that groups every outgoing ray by its full raypath and reports each group's share of the total energy.

> **The document must have shown a picture once; it need not match.** Analyze submits the document as it is on the panels — the same scene a Run would render — so it works on a freshly opened `.lmc` that carries its rendered picture, and on a document you have edited since the last run (the picture on screen is then of the previous configuration, and a line under the button says so; the list always describes the configured scene). What it does not do is analyse a scene you have never seen rendered: on a new document, a JSON import, or an `.lmc` saved without its picture, Analyze and the **In frame** / **Point** region choices stay disabled — with a tooltip saying to press Run once — until the first Run. Apart from that, the only thing that keeps the button disabled is a run in progress.

## 1. Where it is

Click the **Analysis** button (route icon) in the Top Bar, next to **Colors**. It opens an independent, non-modal "Raypath Analysis" window that you can leave open alongside the main preview — it does not replace or dock into any existing panel.

## 2. Choosing a region (ROI)

The **Region** row offers three modes:

| Mode | What it analyzes |
|------|-------------------|
| **Whole sky** | Every ray that leaves the scene, in any direction. No spatial filtering. |
| **In frame** | Only rays that land inside the picture as currently framed (same lens / view / visible / front settings the preview uses). The choice needs the document to have been rendered once (a background photograph alone does not count); with the mode already selected and no live preview, the frame is the document's own view at its simulation resolution. |
| **Point** | A cone around a direction, shown on the preview as a marker. Like In frame, the choice waits for the document's first picture. |

**The Point marker**: switching to Point mode places the marker at the centre of the current view by default (if there is a preview to place it on, and no earlier pick to keep). It moves with the view every frame — projected from the direction it represents, not fixed to a screen position — so rotating or panning the view never leaves it behind. The marker, its ring and the pick crosshair are drawn on the preview itself, so any window over the preview — the analysis window, an entry editor — covers them rather than being drawn through. Hover over the marker for a hand cursor and drag it to a new spot to set the centre directly; the altitude/azimuth reading next to **Pick on preview** updates as you drag. Press **Pick on preview** instead for a banner across the top of the window ("Click on the preview to set the centre — Esc to cancel") and a crosshair cursor: click anywhere on the preview to move the marker there, or press Esc to cancel.

A **Radius** slider is available as soon as there is a centre — even before you press Analyze — and drives the ROI ring drawn on the preview. Before a result exists, dragging it only changes that ring's size; once a result exists, it instead changes how far from the centre the report sums energy, still without re-running the analysis: the underlying pass always records the full cone split into fine angular rings, and the slider only decides, at display time, how many of those rings to add up. This means you can sweep the radius back and forth instantly after a single Analyze click.

## 3. Running the analysis

Above the Analyze button, **Infinite rays** and **Rays(M)** set how many rays the analysis traces — its own budget, independent of the document's **Rays** setting (it starts from that value the first time you open the window). Turn off **Infinite rays** and drag **Rays(M)** to trace a fixed total, in millions, across every wavelength; turn it on to trace until you press Stop. There is no separate early-stop setting for any region mode — the ray budget and the Stop button are the only two ways a run ends.

Press **Analyze**. A dedicated pass traces the scene and groups rays by raypath; while it runs you can press **Stop** to end it early with whatever has accumulated so far — pressing Stop never discards results, it just stops accumulating more. When it finishes (or is stopped), the result list below fills in.

**The analysis always runs on the CPU**, even if you have Metal or CUDA selected for rendering. This is deliberate, not a bug or a fallback you can turn off: the GPU trace path does not keep the per-ray bookkeeping this feature needs. There is no separate indicator for this in the window — Analyze simply may take longer than a GPU-accelerated render of the same scene would.

The analysis also always deals rays across crystals by their population share, whatever the Simulation panel's **Adaptive ray allocation** checkbox says — that switch shapes the *render's* noise, and an analysis is a count of energy per raypath rather than a picture, so its totals are the same either way. Only the render preview changes with the checkbox.

If the configuration has changed since your last run, Analyze still works and reports on the edited document — the picture on screen is the older one, and the line under the button says so until you Run again.

## 4. Reading the result list

Each row is one distinct raypath — the full sequence of crystals and faces a group of rays went through, root to exit. A path through one crystal is written as its faces, `3-5` (entering face 3, exiting face 5). When a scattering layer holds more than one crystal, the crystal is named in front: `C1(3-5)`. With multiple scattering every layer is parenthesised and the layers are joined by ` -> `, root first: `(3-5) -> (1-3)`, or `C1(1-3) -> C4(3-5)` where the layers hold several crystals. (On screen the list draws that joiner as a right-arrow icon; the text itself — what the CLI prints, what an exported filter is named, what the C API hands out — is the ASCII ` -> `.) The columns:

| Column | Meaning |
|--------|---------|
| **Raypath** | The chain's printable name, as above. |
| **Energy** | This raypath's share of the total energy in the current region, as a percentage. Rows are sorted by this, descending. |
| **Cumulative %** | The running total of the Energy column down to and including this row, so you can see at a glance how many rows account for most of the light. |
| **Rays** | How many rays were counted for this raypath (grouped in thousands, `15,788`; the count in the "Analyzing…" line while a run is in progress is written the same way). |
| **+/-** | The statistical noise on that count (1/√N), so a thin tail entry reads as noisy rather than as a precise small number. A row that took over a slot vacated by a much smaller entry (see "Record limits" below) also shows, in parentheses, how much of its energy may actually belong to that other raypath. |

A raypath's raw energy is **not** the same as how visually prominent its arc looks on screen: a faint but wide-spread pattern (common with randomly-oriented crystals) can carry more total energy than a narrow bright arc, and can therefore outrank it in this list. The list answers "how much light", not "how eye-catching".

**Record limits.** The analysis keeps a large but fixed number of distinct raypaths — enough that a typical scene never notices — rather than growing without bound as more rays or scattering layers are added. If a scene does produce more distinct raypaths than fit, a grayed-out **other** row appears at the bottom of the list: it is the energy and ray count that did not fit in a named row, and it is what makes the Cumulative % column reach exactly 100 at the last row. It is never selectable and cannot be excluded, since it does not correspond to one raypath. The status line under the button notes when this happened ("record full (N hits)"); for the reference scenes shipped with this tool, and for most real configurations, it does not happen at all.

**Symmetry (P / B / D)**. The three checkboxes above the list decide which raypaths count as the same row — the same P, B and D symmetries the filter editor uses (prism-face rotation, basal-face reflection, mirror symmetry). With all three on (the default) the six rotations and the mirror image of `3-5` are one row; turn D off and the mirror path `3-7` becomes its own row, turn P off and every rotation does. This is a display-time choice: the analysis records every path unreduced, and toggling a checkbox regroups the result on hand at once — nothing re-runs, the totals do not change, and a selected row stays selected as long as its raypath is still a row (a row that merged into another is simply deselected). If you have pressed Run since the analysis, the result can no longer be regrouped; the window says which symmetry the list is showing, and the next Analyze applies the checkboxes.

## 5. "Exclude this raypath" and "Export CSV"

Select a row and press **Exclude this raypath** to generate a filter that removes rays taking that exact path, under the same P / B / D symmetry the list is showing (so the filter removes exactly what the row merged, no more and no less), and mark the document as modified — press Run again to see the picture without it. This reuses the same filter mechanism as manually editing a crystal's filter in the crystal editor; it does not introduce a new kind of rule.

The button is disabled, with a tooltip explaining why, when:

- no row is selected;
- **the raypath crosses more than one crystal** (multiple scattering across layers) — a filter attaches to a single crystal, so there is no way to express "exclude this exact multi-crystal sequence" today. Only single-crystal, single-layer raypaths can be excluded this way;
- the crystal the raypath went through is no longer in the current document (e.g. you edited the crystal list after analyzing — run and analyze again);
- **an entry using that crystal already has an In filter** — Exclude can only add to an existing Out filter, so it stays disabled here; edit the In filter by hand, or change its action to Out, to continue.

If the crystal already has an **Out** filter instead, the button does not disable: pressing it appends this raypath to that filter as one more excluded alternative — the tooltip says which filter and how many other entries share it — rather than being refused. Excluding the same raypath twice does not add a duplicate.

**Export CSV**, beside it, saves the list **as shown** — under the radius and the P / B / D symmetry currently on display, in the display order — to a `.csv` file through a save dialog. The file opens with `#`-prefixed lines that record the export time, the region, the cone centre / radius / rings summed for a Point result, the symmetry, the total ray count, the total energy the percentages are shares of, and the record-full hit count, so it describes itself without the window. Then one column-header row (`Raypath,Energy,Cumulative %,Rays,+/-`, matching the table above one for one) and the rows: the raypath name, its share of the total energy, the cumulative share, the ray count, and the 1/√N noise together with the takeover bound the **+/-** column shows in parentheses (empty when none), and finally the **other** row when there is one. Numbers are written plain — no `%` sign, no thousands grouping — so a spreadsheet or script reads them directly. The button is disabled while there is no result.

## Further reading

- Full panel reference → [`../gui-guide.md`](../gui-guide.md)
- Filter syntax used by "Exclude this raypath" → [`../gui-guide.md`](../gui-guide.md) §"Filter Tab"
- Design record and mechanism detail → [`../raypath-analysis-panel.md`](../raypath-analysis-panel.md)
