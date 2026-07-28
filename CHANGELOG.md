# Changelog

## 3.0.0
- Reliable Dijkstra/A*-based path generation with a detour-bias distance
  system (replaces earlier noisy-retry / free-walk experiments).
- Multiple full paths per anchor chain (`Number of Paths`), each its own
  vertex group.
- Branch offshoots per path.
- Optional collision exclusion between paths/branches.
- Results written to standard, user-named vertex groups.
- Live-updating panel; workflow help moved to Add-on Preferences.

## 2.0.0
- Real-time recompute, min/max distance range, per-path vertex groups and
  colors, group select/duplicate operators.

## 1.0.0
- Initial release: pluggable pathfinding algorithm registry (Dijkstra, A*,
  Greedy Best-First, BFS), edit-mode vertex path selection.
