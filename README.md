# Vine Path Generator

A Blender add-on that generates organic, vine-like paths across a mesh's
vertices/edges using real pathfinding algorithms (Dijkstra, A*, Greedy
Best-First, BFS), with live-updating distance control, branching offshoots,
collision exclusion between paths, and results written straight into normal
Blender vertex groups.

Built for growing vines/cables/cracks/roots along a mesh surface (e.g. a
metal lattice or grid) without hand-placing curves.

## Features

- **Real pathfinding, not noise-only tricks** — Dijkstra / A* / Greedy
  Best-First / BFS, selectable per run.
- **Live updates** — tweak any setting and the result recomputes
  automatically in Edit Mode.
- **Distance control** — auto-calculated minimum distance per leg, plus a
  `Max Distance ×` multiplier that biases the search into longer, looping
  detours instead of a straight shortest path.
- **Multiple paths** — generate several independent full paths through the
  same anchor chain in one go.
- **Branches** — short dead-end offshoots growing off each path, for a more
  natural vine-like look.
- **Collision exclusion** — optionally prevent paths/branches from touching
  each other, so they don't merge into a solid block of faces.
- **Vertex groups, not custom data** — every generated path is written to a
  standard Blender vertex group (visible/editable in Object Data Properties
  > Vertex Groups like any other group), colored per-path for quick visual
  reference in Solid shading.

## Requirements

- Blender **4.0+**
- Vertex color / color attribute features used for the preview coloring
  require Blender **3.2+** (this add-on targets 4.0+ anyway).

## Installation

1. Download `vine_path_addon.py` from this repository.
2. In Blender: `Edit > Preferences > Add-ons > Install from Disk...`
3. Select the downloaded `.py` file.
4. Enable the checkbox next to **Vine Path Generator**.
5. In the 3D Viewport, open the sidebar (`N`) and find the **Vine** tab.

## Usage

1. Enter **Edit Mode** on your mesh.
2. Select vertices **in order** (click, then Shift+Click the next, etc.) —
   these become the "anchor chain" the path is built through.
3. Click **Set Anchors From Selection**.
4. Set a **Vertex Group** name — this is where the result is written.
5. Adjust **Algorithm**, **Noise Strength**, **Distance**, **Number of
   Paths**, **Collision**, and **Branches** — with Live Update on, the
   result recomputes as you tune each value.
6. The result appears as a vertex group (or several, if Number of Paths > 1),
   listed at the bottom of the panel with **Select** / **Duplicate**
   buttons.
7. From there: select the group's vertices, convert to a curve
   (`Object > Convert To > Curve` after separating, or use the vertices
   directly with your own geometry-nodes setup) and continue your workflow
   (e.g. feeding it into a leaf/vine geometry-nodes group).

See the add-on's entry in `Edit > Preferences > Add-ons` (expand it) for a
condensed in-Blender copy of this workflow.

## How the distance control works

Rather than randomly retrying shortest-path searches and hoping the result
lands near a target length, `Max Distance ×` adds a cost penalty to any edge
that moves the search closer to its destination. This keeps the search
provably connected (same reliability as plain Dijkstra/A*) while naturally
producing longer, curling detours as the multiplier increases — closer to
"take the scenic route" than "pick a random alternate."

## Extending

New pathfinding algorithms can be added without touching anything else:
write a function matching

```python
def my_algo(bm, src, dst, noise_strength, seed,
            blocked=None, allowed_always=None, detour_weight=0.0):
    ...
    return path, visited
```

and register it in the `ALGORITHMS` dict near the top of the file. It will
appear in the Algorithm dropdown automatically.

## License

MIT — see [LICENSE](LICENSE).

## Contributing

Issues and pull requests welcome. Please keep the single-file structure
(`vine_path_addon.py`) unless there's a good reason to split it up.
