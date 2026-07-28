bl_info = {
    "name": "Vine Path Generator",
    "author": "Custom",
    "version": (3, 0, 0),
    "blender": (4, 0, 0),
    "location": "View3D > Sidebar > Vine",
    "description": "Real-time vine/route generator: length-budgeted wandering "
                    "paths (genuine loopy detours, not just noisy shortest-path "
                    "retries), branch offshoots, collision exclusion, and "
                    "per-path vertex groups.",
    "category": "Mesh",
}

import bpy
import bmesh
import heapq
import random
import json
import colorsys
from collections import deque


# ============================================================
# ALGORITHM REGISTRY
# ------------------------------------------------------------
# Used for: (a) computing the true shortest-path baseline per leg,
# (b) direct connections when no extra distance/curliness is needed,
# (c) completing the final stretch of a wandering path once it's
#     close enough to its target.
#
# To add a new algorithm, write:
#   def my_algo(bm, src, dst, noise_strength, seed,
#               blocked=None, allowed_always=None) -> (path, visited)
# and register it in ALGORITHMS below.
# ============================================================

def _edge_cost(edge, noise_strength, rng):
    base = edge.calc_length()
    if noise_strength > 0.0:
        base += rng.uniform(0.0, noise_strength)
    return base


def _reconstruct(prev, src, dst):
    path = []
    node = dst
    while node != src:
        path.append(node)
        if node not in prev:
            path.reverse()
            return path  # disconnected / partial
        node = prev[node]
    path.append(src)
    path.reverse()
    return path


def _is_blocked(v, blocked, allowed_always, dst=None):
    if not blocked:
        return False
    if (dst is not None and v is dst) or v in allowed_always:
        return False
    return v in blocked


def _detour_bias(v, other, dst, detour_weight):
    if detour_weight <= 0.0 or dst is None:
        return 0.0
    closing = (v.co - dst.co).length - (other.co - dst.co).length
    return max(0.0, closing) * detour_weight


def algo_dijkstra(bm, src, dst, noise_strength, seed, blocked=None, allowed_always=None, detour_weight=0.0):
    blocked = blocked or set()
    allowed_always = allowed_always or set()
    rng = random.Random(seed)
    dist = {src: 0.0}
    prev = {}
    visited = set()
    heap = [(0.0, src.index, src)]

    while heap:
        d, _, v = heapq.heappop(heap)
        if v in visited:
            continue
        visited.add(v)
        if v == dst:
            break
        for e in v.link_edges:
            other = e.other_vert(v)
            if _is_blocked(other, blocked, allowed_always, dst):
                continue
            nd = d + _edge_cost(e, noise_strength, rng) + _detour_bias(v, other, dst, detour_weight)
            if other not in dist or nd < dist[other]:
                dist[other] = nd
                prev[other] = v
                heapq.heappush(heap, (nd, other.index, other))

    return _reconstruct(prev, src, dst), visited


def algo_astar(bm, src, dst, noise_strength, seed, blocked=None, allowed_always=None, detour_weight=0.0):
    blocked = blocked or set()
    allowed_always = allowed_always or set()
    rng = random.Random(seed)

    def heuristic(a, b):
        return (a.co - b.co).length

    g_score = {src: 0.0}
    prev = {}
    visited = set()
    heap = [(heuristic(src, dst), 0.0, src.index, src)]

    while heap:
        f, g, _, v = heapq.heappop(heap)
        if v in visited:
            continue
        visited.add(v)
        if v == dst:
            break
        for e in v.link_edges:
            other = e.other_vert(v)
            if _is_blocked(other, blocked, allowed_always, dst):
                continue
            ng = g + _edge_cost(e, noise_strength, rng) + _detour_bias(v, other, dst, detour_weight)
            if other not in g_score or ng < g_score[other]:
                g_score[other] = ng
                prev[other] = v
                nf = ng + heuristic(other, dst)
                heapq.heappush(heap, (nf, ng, other.index, other))

    return _reconstruct(prev, src, dst), visited


def algo_greedy_best_first(bm, src, dst, noise_strength, seed, blocked=None, allowed_always=None, detour_weight=0.0):
    blocked = blocked or set()
    allowed_always = allowed_always or set()
    rng = random.Random(seed)

    def heuristic(a, b):
        return (a.co - b.co).length

    prev = {}
    visited = {src}
    heap = [(heuristic(src, dst), src.index, src)]

    while heap:
        _, _, v = heapq.heappop(heap)
        if v == dst:
            break
        for e in v.link_edges:
            other = e.other_vert(v)
            if other in visited:
                continue
            if _is_blocked(other, blocked, allowed_always, dst):
                continue
            visited.add(other)
            prev[other] = v
            noise = rng.uniform(0.0, noise_strength) if noise_strength > 0 else 0.0
            heapq.heappush(heap, (heuristic(other, dst) + noise, other.index, other))

    return _reconstruct(prev, src, dst), visited


def algo_bfs(bm, src, dst, noise_strength, seed, blocked=None, allowed_always=None, detour_weight=0.0):
    blocked = blocked or set()
    allowed_always = allowed_always or set()
    prev = {}
    visited = {src}
    q = deque([src])

    while q:
        v = q.popleft()
        if v == dst:
            break
        for e in v.link_edges:
            other = e.other_vert(v)
            if other not in visited and not _is_blocked(other, blocked, allowed_always, dst):
                visited.add(other)
                prev[other] = v
                q.append(other)

    return _reconstruct(prev, src, dst), visited


ALGORITHMS = {
    "DIJKSTRA": {"label": "Dijkstra (weighted shortest path)", "func": algo_dijkstra},
    "ASTAR": {"label": "A* (weighted, faster, heuristic-guided)", "func": algo_astar},
    "GREEDY": {"label": "Greedy Best-First (organic, non-optimal)", "func": algo_greedy_best_first},
    "BFS": {"label": "BFS (unweighted, fewest edges)", "func": algo_bfs},
}


def get_algorithm_items(self, context):
    return [(key, data["label"], data["label"]) for key, data in ALGORITHMS.items()]


def _path_cost(path):
    if not path or len(path) < 2:
        return 0.0
    return sum((path[i].co - path[i + 1].co).length for i in range(len(path) - 1))


def _path_is_complete(path, src, dst):
    return bool(path) and path[0] == src and path[-1] == dst


# ============================================================
# BRANCH WANDER (used only for the dead-end branch offshoots -
# these don't need to reach anything, so if it gets stuck it just
# ends the branch early rather than failing anything)
# ============================================================

def _wander_walk(bm, start, target_length, curliness, seed, blocked, allowed_always, max_steps=200):
    rng = random.Random(seed)
    path = [start]
    visit_count = {start: 1}
    remaining = max(target_length, 0.0)
    current = start
    prev_vert = None

    for _ in range(max_steps):
        if remaining <= 0:
            break

        candidates = []
        for e in current.link_edges:
            other = e.other_vert(current)
            if other is prev_vert:
                continue
            if _is_blocked(other, blocked, allowed_always, None):
                continue
            if visit_count.get(other, 0) >= 2:
                continue
            candidates.append((other, e.calc_length()))

        if not candidates:
            if prev_vert is not None and not _is_blocked(prev_vert, blocked, allowed_always, None):
                candidates = [(prev_vert, (current.co - prev_vert.co).length)]
            else:
                break  # genuinely stuck - branch just ends here, that's fine

        other, elen = max(candidates, key=lambda c: rng.uniform(0.0, curliness + 0.001))
        path.append(other)
        visit_count[other] = visit_count.get(other, 0) + 1
        remaining -= elen
        prev_vert = current
        current = other

    return path, _path_cost(path)



def _color_for_index(i):
    hue = (i * 0.6180339887) % 1.0
    r, g, b = colorsys.hsv_to_rgb(hue, 0.75, 0.95)
    return (r, g, b, 1.0)


DEFAULT_COLOR = (0.0, 0.0, 0.0, 1.0)


def _get_or_create_color_layer(bm, name="Vine_Viz"):
    layer = bm.verts.layers.float_color.get(name)
    if layer is None:
        layer = bm.verts.layers.float_color.new(name)
    return layer


def _show_vertex_colors_in_viewport(context, layer_name="Vine_Viz"):
    obj = context.edit_object
    if obj and obj.data and layer_name in obj.data.color_attributes:
        obj.data.color_attributes.active_color = obj.data.color_attributes[layer_name]
    for area in context.screen.areas:
        if area.type == 'VIEW_3D':
            for space in area.spaces:
                if space.type == 'VIEW_3D' and space.shading.type == 'SOLID':
                    space.shading.color_type = 'VERTEX'


VGROUP_PREFIX = "VinePath_"


def _clear_old_vine_groups(obj):
    for g in [g for g in obj.vertex_groups if g.name.startswith(VGROUP_PREFIX)]:
        obj.vertex_groups.remove(g)


# ============================================================
# CORE GENERATION
# ============================================================

_GENERATING = False  # reentrancy guard for live-update callbacks


def _generate_all(context):
    global _GENERATING
    if _GENERATING:
        return False, "Already generating - skipped re-entrant call"
    _GENERATING = True
    try:
        return _generate_all_inner(context)
    except Exception as exc:
        return False, f"Error: {exc}"
    finally:
        _GENERATING = False


def _generate_all_inner(context):
    props = context.scene.vine_path_props
    obj = context.edit_object

    if obj is None or context.mode != 'EDIT_MESH':
        return False, "Not in Edit Mode"
    if not props.anchors_data:
        return False, "No anchors set - select vertices and click 'Set Anchors From Selection'"

    data = json.loads(props.anchors_data)
    if data.get("object") != obj.name or data.get("mesh") != obj.data.name:
        return False, "Anchors belong to a different object - set anchors on this object first"

    bm = bmesh.from_edit_mesh(obj.data)
    bm.verts.ensure_lookup_table()

    anchors = [bm.verts[i] for i in data["indices"] if i < len(bm.verts) and bm.verts[i].is_valid]
    if len(anchors) < 2:
        return False, "Fewer than 2 valid anchors - reselect and set anchors again"

    algo_func = ALGORITHMS[props.algorithm]["func"]
    anchor_set = set(anchors)
    rng_master = random.Random(props.seed)

    base_name = props.target_vgroup_name.strip() or "Vine_Path"

    # clear only groups we created last time (safe - won't touch unrelated groups)
    try:
        prev_names = json.loads(props.last_generated_groups) if props.last_generated_groups else []
    except Exception:
        prev_names = []
    for gname in prev_names:
        g = obj.vertex_groups.get(gname)
        if g:
            obj.vertex_groups.remove(g)

    deform = bm.verts.layers.deform.verify()
    color_layer = _get_or_create_color_layer(bm)
    for v in bm.verts:
        v[color_layer] = DEFAULT_COLOR
        v.select = False

    blocked = set()
    any_leg_failed = False
    written_groups = []
    total_verts = 0

    for path_index in range(props.num_paths):
        combined = [anchors[0]]

        for leg_index in range(len(anchors) - 1):
            src, dst = anchors[leg_index], anchors[leg_index + 1]

            true_path, _ = algo_dijkstra(bm, src, dst, 0.0, 0, set(), anchor_set, 0.0)
            auto_min_cost = _path_cost(true_path) if _path_is_complete(true_path, src, dst) else 0.0
            effective_min = props.min_distance_override if props.min_distance_override_enabled else auto_min_cost

            bias_from_max = max(0.0, props.max_distance_multiplier - 1.0)
            bias_from_min = 0.0
            if auto_min_cost > 0.0 and effective_min > auto_min_cost:
                bias_from_min = (effective_min / auto_min_cost) - 1.0
            detour_weight = max(bias_from_max, bias_from_min)

            seed = props.seed + path_index * 20000 + leg_index * 10000
            leg_blocked = set() if props.allow_path_collision else blocked
            path, _explored = algo_func(bm, src, dst, props.noise_strength, seed, leg_blocked, anchor_set, detour_weight)

            if not _path_is_complete(path, src, dst):
                any_leg_failed = True
                continue

            combined.extend(path[1:])

            if not props.allow_path_collision:
                new_blocked = set(path)
                for v in path:
                    for e in v.link_edges:
                        new_blocked.add(e.other_vert(v))
                new_blocked -= anchor_set
                blocked |= new_blocked

        if len(combined) < 2:
            continue

        # --- branches for this path, folded into the same group ---
        parent_allowed = anchor_set | set(combined)
        for b in range(props.branch_count):
            origin = rng_master.choice(combined)
            branch_target = rng_master.uniform(props.branch_min_length, props.branch_max_length)
            seed = props.seed + path_index * 20000 + b * 311
            branch_blocked = set() if props.allow_path_collision else blocked
            bpath, _bcost = _wander_walk(bm, origin, branch_target, props.noise_strength, seed, branch_blocked, parent_allowed)
            if len(bpath) >= 2:
                combined.extend(bpath[1:])
                if not props.allow_path_collision:
                    new_blocked = set(bpath)
                    for v in bpath:
                        for e in v.link_edges:
                            new_blocked.add(e.other_vert(v))
                    new_blocked -= anchor_set
                    blocked |= new_blocked

        group_name = base_name if path_index == 0 else f"{base_name}_{path_index + 1}"
        vgroup = obj.vertex_groups.new(name=group_name)
        color = _color_for_index(path_index)
        for v in combined:
            if not v.is_valid:
                continue
            v[deform][vgroup.index] = 1.0
            v[color_layer] = color
            v.select = True

        written_groups.append(group_name)
        total_verts += len(set(combined))

    if not written_groups:
        props.last_generated_groups = "[]"
        return False, "No path could be found with current settings"

    props.last_generated_groups = json.dumps(written_groups)

    bm.select_flush(True)
    bmesh.update_edit_mesh(obj.data)
    _show_vertex_colors_in_viewport(context)

    msg = f"{len(written_groups)} path(s), {total_verts} verts -> {', '.join(written_groups)}"
    if any_leg_failed:
        msg += "  (some legs couldn't find a route - try higher Max Distance x)"
    return True, msg


# ============================================================
# LIVE-UPDATE CALLBACK
# ============================================================

def _on_live_property_changed(self, context):
    if not self.live_update:
        return
    if context.mode != 'EDIT_MESH' or context.edit_object is None:
        return
    if not self.anchors_data:
        return
    ok, msg = _generate_all(context)
    self.last_status = msg


# ============================================================
# OPERATORS
# ============================================================

class MESH_OT_vine_set_anchors(bpy.types.Operator):
    """Capture the currently selected vertices (in selection order) as the anchor chain"""
    bl_idname = "mesh.vine_set_anchors"
    bl_label = "Set Anchors From Selection"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and context.edit_object is not None

    def execute(self, context):
        props = context.scene.vine_path_props
        obj = context.edit_object
        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()

        history = [v for v in bm.select_history if isinstance(v, bmesh.types.BMVert) and v.is_valid]
        if len(history) < 2:
            selected = [v for v in bm.verts if v.select]
            if len(selected) < 2:
                self.report({'WARNING'}, "Select at least 2 vertices (Shift+Click in order)")
                return {'CANCELLED'}
            ordered = selected
        else:
            ordered = history

        props.anchors_data = json.dumps({
            "object": obj.name,
            "mesh": obj.data.name,
            "indices": [v.index for v in ordered],
        })
        self.report({'INFO'}, f"{len(ordered)} anchors set")

        if props.live_update:
            ok, msg = _generate_all(context)
            props.last_status = msg

        return {'FINISHED'}


class MESH_OT_vine_generate_now(bpy.types.Operator):
    """Run generation immediately using the current settings (works even with Live Update off)"""
    bl_idname = "mesh.vine_generate_now"
    bl_label = "Generate Now"
    bl_options = {'REGISTER', 'UNDO'}

    @classmethod
    def poll(cls, context):
        props = context.scene.vine_path_props
        return context.mode == 'EDIT_MESH' and context.edit_object is not None and bool(props.anchors_data)

    def execute(self, context):
        props = context.scene.vine_path_props
        ok, msg = _generate_all(context)
        props.last_status = msg
        self.report({'INFO'} if ok else {'WARNING'}, msg)
        return {'FINISHED'} if ok else {'CANCELLED'}


class MESH_OT_vine_group_select(bpy.types.Operator):
    """Select only the vertices belonging to this vine path's vertex group"""
    bl_idname = "mesh.vine_group_select"
    bl_label = "Select Group"
    bl_options = {'REGISTER', 'UNDO'}

    group_name: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and context.edit_object is not None

    def execute(self, context):
        obj = context.edit_object
        group = obj.vertex_groups.get(self.group_name)
        if group is None:
            self.report({'ERROR'}, f"Group '{self.group_name}' not found")
            return {'CANCELLED'}

        bm = bmesh.from_edit_mesh(obj.data)
        bm.verts.ensure_lookup_table()
        deform = bm.verts.layers.deform.active
        if deform is None:
            self.report({'ERROR'}, "No vertex groups on this mesh")
            return {'CANCELLED'}

        gi = group.index
        for v in bm.verts:
            v.select = gi in v[deform]

        bm.select_flush(True)
        bmesh.update_edit_mesh(obj.data)
        return {'FINISHED'}


class MESH_OT_vine_group_duplicate(bpy.types.Operator):
    """Duplicate this vine path's vertices out into a new standalone object, keeping the original intact"""
    bl_idname = "mesh.vine_group_duplicate"
    bl_label = "Duplicate Group"
    bl_options = {'REGISTER', 'UNDO'}

    group_name: bpy.props.StringProperty()

    @classmethod
    def poll(cls, context):
        return context.mode == 'EDIT_MESH' and context.edit_object is not None

    def execute(self, context):
        bpy.ops.mesh.vine_group_select(group_name=self.group_name)

        before = set(o.name for o in bpy.data.objects)
        bpy.ops.mesh.duplicate()
        bpy.ops.mesh.separate(type='SELECTED')
        after = set(o.name for o in bpy.data.objects)

        new_names = after - before
        if new_names:
            bpy.data.objects[next(iter(new_names))].name = self.group_name

        self.report({'INFO'}, f"Duplicated '{self.group_name}' as new object")
        return {'FINISHED'}


# ============================================================
# PROPERTIES
# ============================================================

class VinePathProperties(bpy.types.PropertyGroup):
    target_vgroup_name: bpy.props.StringProperty(
        name="Vertex Group",
        description="Name of the vertex group the single combined path is "
                    "written to - a normal Blender vertex group, editable in "
                    "the standard Vertex Groups panel too",
        default="Vine_Path",
        update=_on_live_property_changed,
    )
    algorithm: bpy.props.EnumProperty(
        name="Algorithm",
        description="Pathfinding algorithm used for every leg (Dijkstra/A* "
                    "respect the Distance Range below; Greedy/BFS ignore it)",
        items=get_algorithm_items,
        update=_on_live_property_changed,
    )
    noise_strength: bpy.props.FloatProperty(
        name="Noise Strength",
        description="Randomizes edge cost so paths wander/vary instead of "
                    "being perfectly optimal, and gives variety between "
                    "alternate paths and branches. On dense grids with tiny "
                    "edges, small values do almost nothing - try larger numbers",
        default=5.0,
        min=0.0,
        soft_max=50.0,
        update=_on_live_property_changed,
    )
    seed: bpy.props.IntProperty(
        name="Seed",
        description="Base random seed - change for a different family of results",
        default=0,
        update=_on_live_property_changed,
    )

    anchors_data: bpy.props.StringProperty(default="")
    live_update: bpy.props.BoolProperty(
        name="Live Update",
        description="Recompute automatically whenever a setting below changes "
                    "(anchors must be set first)",
        default=True,
    )
    last_status: bpy.props.StringProperty(default="")
    last_generated_groups: bpy.props.StringProperty(default="[]")

    min_distance_override_enabled: bpy.props.BoolProperty(
        name="Override Min Distance",
        description="By default the minimum distance is the true shortest "
                    "possible path (auto-calculated per leg). Enable to force "
                    "a custom minimum instead",
        default=False,
        update=_on_live_property_changed,
    )
    min_distance_override: bpy.props.FloatProperty(
        name="Min Distance",
        description="Forced minimum path length (only used when override is on)",
        default=0.0,
        min=0.0,
        soft_max=1000.0,
        update=_on_live_property_changed,
    )
    max_distance_multiplier: bpy.props.FloatProperty(
        name="Max Distance ×",
        description="Biases the search away from beelining straight to the "
                    "target, roughly scaling path length by this multiple of "
                    "the minimum distance. 1.0 = shortest path. Higher = "
                    "longer, more looping detours (works with Dijkstra/A* - "
                    "the search still always finds a way through if one "
                    "exists, it just isn't forced to hit this exactly)",
        default=2.0,
        min=1.0,
        soft_max=10.0,
        update=_on_live_property_changed,
    )
    num_paths: bpy.props.IntProperty(
        name="Number of Paths",
        description="How many separate full paths to generate through the "
                    "same anchor chain, each written to its own vertex group "
                    "(named 'GroupName', 'GroupName_2', 'GroupName_3'...)",
        default=1,
        min=1,
        soft_max=20,
        update=_on_live_property_changed,
    )
    allow_path_collision: bpy.props.BoolProperty(
        name="Allow Path Collision",
        description="ON: paths can freely cross/touch each other. "
                    "OFF: a new path cannot use, or be edge-adjacent to, any "
                    "vertex already claimed by a previous path or branch. "
                    "Combine with a higher Max Distance x so blocked paths "
                    "can still find a way around instead of just failing",
        default=True,
        update=_on_live_property_changed,
    )
    branch_count: bpy.props.IntProperty(
        name="Branches Per Path",
        description="How many short offshoot tendrils to grow off each "
                    "accepted path - these don't connect to anything, they "
                    "just wander and stop, like real vine side-shoots",
        default=2,
        min=0,
        soft_max=20,
        update=_on_live_property_changed,
    )
    branch_min_length: bpy.props.FloatProperty(
        name="Branch Min Length",
        default=1.0,
        min=0.0,
        soft_max=200.0,
        update=_on_live_property_changed,
    )
    branch_max_length: bpy.props.FloatProperty(
        name="Branch Max Length",
        default=5.0,
        min=0.0,
        soft_max=200.0,
        update=_on_live_property_changed,
    )


# ============================================================
# UI PANEL
# ============================================================

class VIEW3D_PT_vine_path(bpy.types.Panel):
    bl_label = "Vine Path Generator"
    bl_idname = "VIEW3D_PT_vine_path"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Vine"
    bl_context = "mesh_edit"

    def draw(self, context):
        layout = self.layout
        props = context.scene.vine_path_props

        col = layout.column()
        col.operator("mesh.vine_set_anchors", icon='PINNED')
        if props.anchors_data:
            data = json.loads(props.anchors_data)
            col.label(text=f"{len(data['indices'])} anchors on '{data['object']}'")
        else:
            col.label(text="No anchors set", icon='ERROR')
        col.prop(props, "target_vgroup_name")
        col.prop(props, "live_update", icon='FILE_REFRESH')

        box = layout.box()
        box.label(text="Generation", icon='CURVE_DATA')
        box.prop(props, "algorithm")
        box.prop(props, "noise_strength")
        box.prop(props, "seed")
        box.prop(props, "num_paths")
        box.prop(props, "allow_path_collision")

        box = layout.box()
        box.label(text="Distance", icon='DRIVER_DISTANCE')
        row = box.row()
        row.prop(props, "min_distance_override_enabled", text="")
        sub = row.row()
        sub.enabled = props.min_distance_override_enabled
        sub.prop(props, "min_distance_override")
        box.prop(props, "max_distance_multiplier")

        box = layout.box()
        box.label(text="Branches", icon='OUTLINER_OB_CURVES')
        box.prop(props, "branch_count")
        box.prop(props, "branch_min_length")
        box.prop(props, "branch_max_length")

        layout.separator()
        layout.operator("mesh.vine_generate_now", icon='FORCE_CURVE')
        if props.last_status:
            layout.label(text=props.last_status, icon='INFO')

        obj = context.edit_object
        if obj is not None:
            try:
                names = json.loads(props.last_generated_groups)
            except Exception:
                names = []
            names = [n for n in names if obj.vertex_groups.get(n)]
            if names:
                box = layout.box()
                box.label(text=f"Result ({len(names)})", icon='GROUP_VERTEX')
                for gname in names:
                    row = box.row(align=True)
                    row.label(text=gname)
                    op = row.operator("mesh.vine_group_select", text="", icon='RESTRICT_SELECT_OFF')
                    op.group_name = gname
                    op2 = row.operator("mesh.vine_group_duplicate", text="", icon='DUPLICATE')
                    op2.group_name = gname


class VineAddonPreferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    def draw(self, context):
        layout = self.layout
        layout.label(text="Workflow", icon='QUESTION')
        layout.label(text="1. Select vertices in order, click 'Set Anchors From Selection'")
        layout.label(text="2. Set a Vertex Group name - results are written there")
        layout.label(text="3. Tune Generation / Distance / Branches - updates live")
        layout.label(text="4. Raise 'Number of Paths' for multiple separate routes")
        layout.label(text="   (each gets its own group: Name, Name_2, Name_3...)")
        layout.label(text="5. Results are normal vertex groups - also visible/editable")
        layout.label(text="   in Object Data Properties > Vertex Groups")


# ============================================================
# REGISTRATION
# ============================================================

classes = (
    VinePathProperties,
    MESH_OT_vine_set_anchors,
    MESH_OT_vine_generate_now,
    MESH_OT_vine_group_select,
    MESH_OT_vine_group_duplicate,
    VIEW3D_PT_vine_path,
    VineAddonPreferences,
)


def register():
    for cls in classes:
        bpy.utils.register_class(cls)
    bpy.types.Scene.vine_path_props = bpy.props.PointerProperty(type=VinePathProperties)


def unregister():
    del bpy.types.Scene.vine_path_props
    for cls in reversed(classes):
        bpy.utils.unregister_class(cls)


if __name__ == "__main__":
    register()
