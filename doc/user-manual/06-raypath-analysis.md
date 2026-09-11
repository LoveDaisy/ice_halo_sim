[中文版](06-raypath-analysis_zh.md)

# Raypath Analysis

The Raypath Analysis window answers "which raypaths make the light in this region of the sky, and how much does each contribute". Unlike the main preview, it does not render a picture — it runs a dedicated, non-rendering pass that groups every outgoing ray by its full raypath and reports each group's share of the total energy.

> **Prerequisite**: run the simulation at least once, so there is a committed scene to analyze. The window's Analyze button stays disabled until you do (and again after you change the config, until you run it again).

## 1. Where it is

Click the **Analysis** button (route icon) in the Top Bar, next to **Colors**. It opens an independent, non-modal "Raypath Analysis" window that you can leave open alongside the main preview — it does not replace or dock into any existing panel.

## 2. Choosing a region (ROI)

The **Region** row offers three modes:

| Mode | What it analyzes |
|------|-------------------|
| **Whole sky** | Every ray that leaves the scene, in any direction. No spatial filtering. |
| **In frame** | Only rays that land inside the picture as currently framed (same lens / view / visible / front settings the preview uses). Disabled when there is no preview on screen to define "the frame". |
| **Point** | A cone around a direction you pick. Click **Pick on preview**, then click a point in the Render Preview; the crosshair direction becomes the cone's centre. |

In **Point** mode, once you have a result, a **Radius** slider appears. Dragging it changes how far from the centre the report sums energy — but it does **not** re-run the analysis. The underlying pass always records the full cone split into fine angular rings; the slider only decides, at display time, how many of those rings to add up. This means you can sweep the radius back and forth instantly after a single Analyze click.

## 3. Running the analysis

Press **Analyze**. A dedicated pass traces the scene and groups rays by raypath; while it runs you can press **Stop** to end it early with whatever has accumulated so far. When it finishes (or is stopped), the result list below fills in.

**The analysis always runs on the CPU**, even if you have Metal or CUDA selected for rendering. This is deliberate, not a bug or a fallback you can turn off: the GPU trace path does not keep the per-ray bookkeeping this feature needs. There is no separate indicator for this in the window — Analyze simply may take longer than a GPU-accelerated render of the same scene would.

If the configuration has changed since your last run, Analyze stays disabled until you press Run again — the analysis reports on the scene you last ran, not on unsaved edits.

## 4. Reading the result list

Each row is one distinct raypath — the full sequence of crystals and faces a group of rays went through, root to exit, written like `crystal1(3-5)` (one crystal, entering face 3 and exiting face 5) or `crystal1(3-5)-crystal2(1-3)` (two crystals in sequence, for multiple scattering). The columns:

| Column | Meaning |
|--------|---------|
| **Raypath** | The chain's printable name, as above. |
| **Energy** | This raypath's share of the total energy in the current region, as a percentage. Rows are sorted by this, descending. |
| **Rays** | How many rays were counted for this raypath. |
| **+/-** | The statistical noise on that count (1/√N), so a thin tail entry reads as noisy rather than as a precise small number. |

A raypath's raw energy is **not** the same as how visually prominent its arc looks on screen: a faint but wide-spread pattern (common with randomly-oriented crystals) can carry more total energy than a narrow bright arc, and can therefore outrank it in this list. The list answers "how much light", not "how eye-catching".

## 5. "Exclude this raypath"

Select a row and press **Exclude this raypath** to generate a filter that removes rays taking that exact path, and mark the document as modified — press Run again to see the picture without it. This reuses the same filter mechanism as manually editing a crystal's filter in the crystal editor; it does not introduce a new kind of rule.

The button is disabled, with a tooltip explaining why, when:

- no row is selected;
- **the raypath crosses more than one crystal** (multiple scattering across layers) — a filter attaches to a single crystal, so there is no way to express "exclude this exact multi-crystal sequence" today. Only single-crystal, single-layer raypaths can be excluded this way;
- the crystal the raypath went through is no longer in the current document (e.g. you edited the crystal list after analyzing — run and analyze again);
- an entry using that crystal already has a filter attached (excluding on top of it is not merged automatically — edit that filter by hand instead).

## 6. Known limitation: the region ring can drift after you rotate the view

In **Point** mode, the ring drawn on the preview to show the analyzed cone is placed using the picture as it looked at the moment you clicked. If you rotate or pan the view afterwards, the ring stays at the same screen position while the picture moves under it — the *direction* the analysis used is still correct, but the *ring's on-screen circle* no longer lines up with it. Re-pick the point (or avoid rotating the view) if you need the ring to track the current picture.

## Further reading

- Full panel reference → [`../gui-guide.md`](../gui-guide.md)
- Filter syntax used by "Exclude this raypath" → [`../gui-guide.md`](../gui-guide.md) §"Filter Tab"
- Design record and mechanism detail → [`../raypath-analysis-panel.md`](../raypath-analysis-panel.md)
