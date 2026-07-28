# Vines and Ivy — Geometry Nodes Leaf Setup

This is the companion geometry-nodes group used to grow leaves along the
curves/paths produced by `vine_path_addon.py`. The `.blend` file containing
this node group is included as an archive in this repository, along with a
short demo video.

- **Archive:** see the repo root for the `.zip`/`.blend` containing the
  "Vines and Ivy" node group.
- **Video:** see the repo root / releases for the demo walkthrough.

## How it fits together

1. Use **Vine Path Generator** (`vine_path_addon.py`) in Edit Mode to build
   a path across your mesh, stored in a vertex group.
2. Select that vertex group's vertices, separate them out, and convert to a
   **Curve** (`Object > Convert To > Curve`).
3. Add the **Vines and Ivy** geometry-nodes modifier to that curve object.
4. Assign the `LEAFS` collection (see below) and tune the parameters to
   taste.

## Parameters

| Parameter | What it controls |
|---|---|
| **thin one** | Thickness of the vine's thin/tip end (currently `1 mm`). |
| **thick one** | Thickness of the vine's thick/base end (currently `1 mm`). |
| **Density Factor** | Overall multiplier on how many leaves are placed along the curve. `1.0` = default density. |
| **scale seed** | Random seed used specifically for per-leaf scale variation. Change for a different random distribution of leaf sizes without affecting placement. |
| **rotatin seed** | Random seed for per-leaf rotation variation. Change for different leaf orientations without affecting scale/placement. |
| **Min leafscale** | Smallest a leaf can be scaled to (currently `13.8`). |
| **Max leafscale** | Largest a leaf can be scaled to (currently `29.9`). Leaves are randomly scaled between Min and Max. |
| **leaf Distance Min** | Minimum spacing enforced between neighboring leaves along the curve (currently `37.8 mm`), so leaves don't overlap/cluster. |
| **Density Max** | Upper cap on leaf density (currently `10.0`), works alongside Density Factor. |
| **vines ditortion** | Adds distortion/curl to the vine curve itself (currently `0.38`). Higher = more twisted, organic-looking growth. |
| **noise 1** | Primary noise strength affecting leaf placement/orientation (currently `53.17`). |
| **noise multi** (first) | Multiplier applied to noise 1. `0.0` currently disables its contribution. |
| **noise 2** | Secondary noise layer (currently `0.025`) — typically a finer-scale variation layered on top of noise 1. |
| **noise multi** (second) | Multiplier applied to noise 2. `0.0` currently disables its contribution. |
| **vines resulution** | Resolution/segment count along the curve (currently `200`). Higher = smoother curve, more geometry, slower to compute. |
| **LEAVES** | Toggle - enables/disables leaf generation entirely (checked = on). |
| **Collection** | Which collection of leaf mesh objects to instance from — currently `LEAFS`. Swap this to use different leaf shapes/species. |

## Suggested starting points

- **Sparse, wiry climbing vine:** lower `Density Factor` (~0.3-0.5), keep
  `leaf Distance Min` high, `vines ditortion` low (~0.1-0.2).
- **Dense, overgrown look:** raise `Density Factor` and `Density Max`,
  lower `leaf Distance Min`, raise `noise 1` for more scattered variation.
- **Natural curl matching an aged/weathered surface:** raise
  `vines ditortion` and `noise 1` together; keep `noise multi` values above
  `0` if you want the two noise layers to actually blend (they're currently
  set to `0`, which zeroes their contribution despite `noise 1`/`noise 2`
  having values).

## Notes

- `scale seed` and `rotatin seed` are independent — use different values on
  copies of the same curve to get visually distinct vines without changing
  density or distortion settings.
- `thin one` / `thick one` currently match at `1 mm` each (uniform
  thickness). Set them apart (e.g. thick base, thin tip) for a more
  tapered, natural vine profile.
