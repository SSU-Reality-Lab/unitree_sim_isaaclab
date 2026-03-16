"""
Scene configuration for CS-Projects generated scenes (v4 — physics-enabled).

Reads a scene JSON (from the generated_scenes4 pipeline), resolves asset paths
relative to the CS_PROJECTS environment variable, computes shelf-item placements,
and builds an InteractiveSceneCfg with:
  - AssetBaseCfg (kinematic) for shelf and walls
  - RigidObjectCfg (dynamic, with collision + mass + friction) for items
"""

import os
import json
import glob
import math

# NOTE: isaaclab imports are deferred to inside build_scene_assets() so that
# find_scene_jsons() can be called at import time without requiring isaaclab.


# ---------------------------------------------------------------------------
# Path resolution helpers
# ---------------------------------------------------------------------------

def _get_cs_projects_dir() -> str:
    """Return the base directory for CS-Projects assets (from env var)."""
    d = os.environ.get("CS_PROJECTS", "")
    if not d:
        raise EnvironmentError(
            "CS_PROJECTS environment variable is not set. "
            "Set it to the generated_scenes root directory."
        )
    return d


def _resolve(path: str, base_dir: str) -> str:
    """Resolve a relative path against base_dir; absolute paths pass through."""
    if not path:
        return path
    p = path.replace("\\", "/")
    if ":" not in p and not p.startswith("/"):
        return os.path.normpath(os.path.join(base_dir, p)).replace("\\", "/")
    return p


# ---------------------------------------------------------------------------
# Scene JSON defaults (mirrors generated_scenes4/shelf_cola/item_placement.py)
# ---------------------------------------------------------------------------

_DEFAULTS = {
    "shelf_usd": "Prop/shelf/shelf_real.usd",
    "shelf_name": "SHELF",
    "planes_dir": "Prop/shelf/planes_fixed_real",
    "walls_enabled": True,
    "wall_usd": "Prop/walls/walls.usda",
    "wall_height": 1.8,
    "wall_thickness": 0.05,
    "wall_margin": 0.05,
    "wall_length": 2.2,
    "fast_load": True,
    "placement_mode": "auto",
}

_PATH_KEYS = {"shelf_usd", "planes_dir", "wall_usd"}


def _patch_cfg(cfg: dict, base_dir: str) -> dict:
    """Fill defaults and resolve relative paths in a scene config dict."""
    for key, default_val in _DEFAULTS.items():
        if key not in cfg or (isinstance(default_val, str) and not cfg[key]):
            cfg[key] = default_val
    for key in _PATH_KEYS:
        if key in cfg and cfg[key]:
            cfg[key] = _resolve(cfg[key], base_dir)
    for label, val in cfg.get("item_usds", {}).items():
        if isinstance(val, dict) and "usd" in val:
            val["usd"] = _resolve(val["usd"], base_dir)
        elif isinstance(val, str):
            cfg["item_usds"][label] = _resolve(val, base_dir)
    return cfg


# ---------------------------------------------------------------------------
# Pure-python placement math (from view_scene.py)
# ---------------------------------------------------------------------------

def _vec3(lst):
    return [float(lst[0]), float(lst[1]), float(lst[2])]

def _mat3_vec3(M, v):
    return [
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2],
    ]

def _add3(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]

def _scale3(s, v):
    return [s*v[0], s*v[1], s*v[2]]

def _euler_deg_to_quat_xyzw(rx_deg, ry_deg, rz_deg):
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    w = cx*cy*cz + sx*sy*sz
    x = sx*cy*cz - cx*sy*sz
    y = cx*sy*cz + sx*cy*sz
    z = cx*cy*sz - sx*sy*cz
    return [x, y, z, w]

def _quat_xyzw_to_wxyz(q):
    return [q[3], q[0], q[1], q[2]]

# Shelf rotation: 90 degrees around X
_R_SHELF = [
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
    [0.0, 1.0,  0.0],
]

_SHELF_QUAT_OG = [0.7071067811865476, 0.0, 0.0, 0.7071067811865476]  # x,y,z,w
_SHELF_POS = [0.0, 0.0, 0.0]


def _load_planes(planes_dir):
    planes = []
    for fname in sorted(glob.glob(os.path.join(planes_dir, "plane_*.json"))):
        with open(fname, "r", encoding="utf-8") as f:
            pdata = json.load(f)

        ctr_l = _vec3(pdata["centroid_l"])
        u_l = _vec3(pdata["u_l"])
        v_l = _vec3(pdata["v_l"])
        n_l = _vec3(pdata["n_l"])

        ctr_w = _add3(_mat3_vec3(_R_SHELF, ctr_l), _SHELF_POS)
        u_w = _mat3_vec3(_R_SHELF, u_l)
        v_w = _mat3_vec3(_R_SHELF, v_l)
        n_w = _mat3_vec3(_R_SHELF, n_l)

        mask = pdata["usable_mask"]
        cell = float(pdata["cell"])
        u0, v0 = float(pdata["origin_uv"][0]), float(pdata["origin_uv"][1])

        u_min_idx = u_max_idx = v_min_idx = v_max_idx = None
        for vi, row in enumerate(mask):
            for ui, val in enumerate(row):
                if val:
                    if u_min_idx is None or ui < u_min_idx: u_min_idx = ui
                    if u_max_idx is None or ui > u_max_idx: u_max_idx = ui
                    if v_min_idx is None or vi < v_min_idx: v_min_idx = vi
                    if v_max_idx is None or vi > v_max_idx: v_max_idx = vi

        if u_min_idx is None:
            continue

        planes.append({
            "centroid": ctr_w, "u": u_w, "v": v_w, "normal": n_w,
            "cell": cell, "origin_uv": [u0, v0],
            "u_min_idx": u_min_idx, "u_max_idx": u_max_idx,
            "v_min_idx": v_min_idx, "v_max_idx": v_max_idx,
            "height": ctr_l[1],
        })

    planes.sort(key=lambda p: p["height"])
    return planes


def _compute_placements(planes, zone_config, item_usds):
    placements = []
    item_counters = {}
    item_rotations = {}
    for label, info in item_usds.items():
        if isinstance(info, dict) and "rotation" in info:
            item_rotations[label] = info["rotation"]

    for plane_key, pcfg in sorted(zone_config.items(), key=lambda x: int(x[0])):
        plane_index = int(plane_key) + 1
        if plane_index >= len(planes):
            continue
        plane = planes[plane_index]

        cell = plane["cell"]
        ctr = plane["centroid"]
        u_w = plane["u"]
        v_w = plane["v"]
        n_w = plane["normal"]
        u0, v0 = plane["origin_uv"]
        u_min_idx = plane["u_min_idx"]
        u_max_idx = plane["u_max_idx"]
        v_min_idx = plane["v_min_idx"]
        v_max_idx = plane["v_max_idx"]

        total_v_m = (v_max_idx - v_min_idx + 1) * cell
        total_u_m = (u_max_idx - u_min_idx + 1) * cell
        v_origin = v0 + v_min_idx * cell
        u_origin = u0 + u_min_idx * cell

        align = pcfg.get("align", "center")
        zone_gap = pcfg.get("zone_gap", 0.03)
        n_remove = pcfg.get("remove", 0)
        zones = pcfg.get("zones", [])

        total_gap_between = (len(zones) - 1) * zone_gap if len(zones) > 1 else 0.0
        available_v = total_v_m - total_gap_between

        zone_infos = []
        for z in zones:
            label = z["label"]
            info = item_usds.get(label)
            if not info or not isinstance(info, dict):
                continue
            foot = info.get("foot", 0.1)
            depth = info.get("depth", foot)
            H = info.get("H", 0.1)
            gap_lr = z.get("gap_lr", 0.02)
            gap_fb = z.get("gap_fb", 0.02)
            count = z.get("count", 1)

            n_rows = max(1, int((total_u_m + gap_fb) / (depth + gap_fb)))
            n_cols = math.ceil(count / n_rows)
            zone_width = n_cols * (foot + gap_lr) - gap_lr

            # bottom_offset: default H/2 (origin at center)
            bottom_offset = H / 2.0

            zone_infos.append({
                "label": label, "count": count, "foot": foot, "depth": depth,
                "gap_lr": gap_lr, "gap_fb": gap_fb,
                "n_cols": n_cols, "n_rows": n_rows,
                "zone_width": zone_width, "obj_H": H,
                "bottom_offset": bottom_offset,
                "_zone_cfg": z,
            })

        if not zone_infos:
            continue

        block_width = sum(zi["zone_width"] for zi in zone_infos)
        if len(zone_infos) > 1:
            block_width += (len(zone_infos) - 1) * zone_gap

        if block_width > total_v_m:
            for zi in zone_infos:
                foot = zi["foot"]
                gap_lr = zi["gap_lr"]
                share = zi["zone_width"] / (block_width - total_gap_between) if (block_width - total_gap_between) > 0 else 1.0
                max_zone_v = available_v * share
                max_cols = max(1, int((max_zone_v + gap_lr) / (foot + gap_lr)))
                if max_cols < zi["n_cols"]:
                    zi["n_cols"] = max_cols
                    zi["count"] = min(zi["count"], max_cols * zi["n_rows"])
                    zi["zone_width"] = max_cols * (foot + gap_lr) - gap_lr
            block_width = sum(zi["zone_width"] for zi in zone_infos)
            if len(zone_infos) > 1:
                block_width += (len(zone_infos) - 1) * zone_gap

        if isinstance(align, str):
            align_f = {"left": 0.0, "center": 0.5, "right": 1.0}.get(align, 0.5)
        else:
            align_f = max(0.0, min(1.0, float(align)))
        block_v_offset = (total_v_m - block_width) * align_f

        current_v = block_v_offset
        plane_placements = []

        for zi_idx, zi in enumerate(zone_infos):
            label = zi["label"]
            foot = zi["foot"]
            depth = zi["depth"]
            n_cols = zi["n_cols"]
            n_rows = zi["n_rows"]
            gap_lr = zi["gap_lr"]
            gap_fb = zi["gap_fb"]
            count = zi["count"]
            obj_H = zi["obj_H"]
            bottom_offset = zi["bottom_offset"]
            zone_width = zi["zone_width"]

            z_cfg = zi.get("_zone_cfg", {})

            if "u_pos" in z_cfg:
                u_start_m = u_origin + z_cfg["u_pos"] * total_u_m + depth / 2.0
            else:
                block_u = n_rows * depth + (n_rows - 1) * gap_fb
                u_start_m = u_origin + (total_u_m - block_u) / 2.0 + depth / 2.0

            if "v_pos" in z_cfg:
                zone_v_start = z_cfg["v_pos"] * total_v_m
            else:
                zone_v_start = current_v
            v_start_m = v_origin + zone_v_start + foot / 2.0

            ir = item_rotations.get(label, [0.0, 0.0, 0.0])
            zr = z_cfg.get("rotation", [0.0, 0.0, 0.0])
            base_rx = ir[0] + zr[0]
            base_ry = ir[1] + zr[1]
            base_rz = ir[2] + zr[2]
            per_item_rots = z_cfg.get("item_rotations", {})
            per_item_offsets = z_cfg.get("item_offsets", {})

            def _compute_ori_offset(crx, cry, crz):
                if crx != 0.0 or cry != 0.0 or crz != 0.0:
                    q = _euler_deg_to_quat_xyzw(crx, cry, crz)
                    rx_r = math.radians(crx)
                    ry_r = math.radians(cry)
                    cxr, sxr = math.cos(rx_r), math.sin(rx_r)
                    cyr, syr = math.cos(ry_r), math.sin(ry_r)
                    R20 = -syr
                    R21 = sxr * cyr
                    R22 = cxr * cyr
                    hx = foot / 2
                    hy = depth / 2
                    hz = obj_H / 2
                    boff = abs(R20) * hx + abs(R21) * hy + abs(R22) * hz
                    return q, boff
                return [0.0, 0.0, 0.0, 1.0], bottom_offset

            base_quat, base_boff = _compute_ori_offset(base_rx, base_ry, base_rz)

            counter = item_counters.get(label, 0)
            skip_set = set(z_cfg.get("skip_indices", []))
            placed = 0

            for row in range(n_rows):
                for col in range(n_cols):
                    if placed >= count:
                        break
                    if placed in skip_set:
                        placed += 1
                        continue

                    pir = per_item_rots.get(str(placed))
                    if pir:
                        crx = base_rx + pir[0]
                        cry = base_ry + pir[1]
                        crz = base_rz + pir[2]
                        quat, cur_boff = _compute_ori_offset(crx, cry, crz)
                    else:
                        quat = base_quat
                        cur_boff = base_boff

                    v_m = v_start_m + col * (foot + gap_lr)
                    u_m = u_start_m + row * (depth + gap_fb)

                    pio = per_item_offsets.get(str(placed))
                    if pio:
                        v_m += pio[0] / 1000.0
                        u_m += pio[1] / 1000.0

                    pos = _add3(ctr, _add3(
                        _scale3(u_m, u_w),
                        _add3(_scale3(v_m, v_w), _scale3(cur_boff, n_w))
                    ))

                    usd_path = item_usds[label]["usd"] if isinstance(item_usds[label], dict) else item_usds[label]
                    placements.append({
                        "label": label, "usd": usd_path, "index": counter,
                        "pos": pos, "quat_wxyz": _quat_xyzw_to_wxyz(quat),
                    })
                    plane_placements.append(len(placements) - 1)
                    counter += 1
                    placed += 1
                if placed >= count:
                    break

            item_counters[label] = counter
            current_v = zone_v_start + zone_width
            if zi_idx < len(zone_infos) - 1:
                current_v += zone_gap

        # Item removal
        remove_indices_cfg = pcfg.get("remove_indices", None)
        if n_remove > 0 and plane_placements:
            n_actual = min(n_remove, len(plane_placements))
            if remove_indices_cfg is not None:
                valid = [i for i in remove_indices_cfg if i < len(plane_placements)][:n_actual]
            else:
                valid = list(range(n_actual))
            for ri in valid:
                placements[plane_placements[ri]]["_removed"] = True

    return [p for p in placements if not p.get("_removed")]


# ---------------------------------------------------------------------------
# Scene discovery
# ---------------------------------------------------------------------------

def find_scene_jsons(root_dir: str = None, split: str = "train"):
    """Find scene_*.json files under root_dir (excluding _meta/_patched).

    Args:
        root_dir: Base directory to search. Defaults to CS_PROJECTS env var.
        split: Only include scenes whose path contains this subdirectory
            (e.g. ``"train"`` or ``"test"``).  Set to ``None`` to include all.
    """
    if root_dir is None:
        root_dir = _get_cs_projects_dir()
    pattern = os.path.join(root_dir, "**", "scene_*.json")
    all_files = glob.glob(pattern, recursive=True)
    filtered = [f for f in sorted(all_files)
                if "_meta" not in os.path.basename(f)
                and "_patched" not in os.path.basename(f)]
    if split:
        filtered = [f for f in filtered if os.sep + split + os.sep in f
                    or "/" + split + "/" in f]
    return filtered


def _sanitize_prim_name(name: str) -> str:
    """Make a string safe for use as a USD prim name."""
    result = ""
    for ch in name:
        if ch.isalnum() or ch == "_":
            result += ch
        else:
            result += "_"
    if result and result[0].isdigit():
        result = "_" + result
    return result or "_unnamed"


# ---------------------------------------------------------------------------
# Scene config factory
# ---------------------------------------------------------------------------

def load_scene_config(scene_json_path: str) -> dict:
    """Load and patch a scene JSON, returning the resolved config dict."""
    base_dir = os.path.dirname(os.path.abspath(scene_json_path))
    # Use CS_PROJECTS as the base for relative paths if set, otherwise use the JSON's parent dir
    cs_dir = os.environ.get("CS_PROJECTS", "")
    resolve_base = cs_dir if cs_dir else base_dir

    with open(scene_json_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    _patch_cfg(cfg, resolve_base)
    return cfg


def _estimate_mass(item_info: dict) -> float:
    """Estimate mass from item dimensions (H, foot, depth).

    Uses a simple volume-based heuristic assuming average density ~300 kg/m³
    (typical for packaged food items).  Clamped to [0.05, 2.0] kg.
    """
    H = item_info.get("H", 0.1)
    foot = item_info.get("foot", 0.1)
    depth = item_info.get("depth", foot)
    volume = H * foot * depth  # m³
    mass = max(0.05, min(2.0, volume * 300.0))
    return round(mass, 4)


def _estimate_shelf_aabb(planes):
    """Estimate the shelf's world-space AABB from loaded plane data.

    Computes the bounding box from plane centroids and their usable mask extents.
    Returns ((x_min, y_min, z_min), (x_max, y_max, z_max)).
    """
    x_vals, y_vals, z_vals = [], [], []
    for p in planes:
        ctr = p["centroid"]
        cell = p["cell"]
        u0, v0 = p["origin_uv"]
        u_w = p["u"]
        v_w = p["v"]

        u_min_m = u0 + p["u_min_idx"] * cell
        u_max_m = u0 + p["u_max_idx"] * cell
        v_min_m = v0 + p["v_min_idx"] * cell
        v_max_m = v0 + p["v_max_idx"] * cell

        for u_m in [u_min_m, u_max_m]:
            for v_m in [v_min_m, v_max_m]:
                x_vals.append(ctr[0] + u_m * u_w[0] + v_m * v_w[0])
                y_vals.append(ctr[1] + u_m * u_w[1] + v_m * v_w[1])
                z_vals.append(ctr[2] + u_m * u_w[2] + v_m * v_w[2])

    if not x_vals:
        return (0, 0, 0), (0, 0, 0)
    return (min(x_vals), min(y_vals), min(z_vals)), (max(x_vals), max(y_vals), max(z_vals))


def _compute_wall_geometry(planes, cfg):
    """Compute ㄷ-shaped wall cuboid positions and sizes.

    Mirrors ``_create_walls`` in generated_scenes4/shelf_cola/main.py.
    Front (-Y) is open.

    Returns dict of {name: {"pos": (x,y,z), "size": (sx,sy,sz)}} for back/left/right.
    """
    (sx_min, sy_min, _sz_min), (sx_max, sy_max, _sz_max) = _estimate_shelf_aabb(planes)

    wh = cfg.get("wall_height", 1.8)
    wt = cfg.get("wall_thickness", 0.05)
    mg = cfg.get("wall_margin", 0.05)
    wl = cfg.get("wall_length", 2.2)

    cx = (sx_min + sx_max) / 2.0

    # Back wall: behind shelf (+Y), with 0.25m additional offset (matches generated_scenes4)
    back_y = sy_max + mg + wt / 2.0 + 0.25

    # Side walls: extend from back wall inner face toward front (-Y)
    side_y_start = back_y - wt / 2.0
    side_y_end = side_y_start - wl
    side_cy = (side_y_start + side_y_end) / 2.0

    left_x = cx - wl / 2.0 - wt / 2.0
    right_x = cx + wl / 2.0 + wt / 2.0

    return {
        "back":  {"pos": (cx, back_y, wh / 2.0), "size": (wl + 2 * wt, wt, wh)},
        "left":  {"pos": (left_x, side_cy, wh / 2.0), "size": (wt, wl, wh)},
        "right": {"pos": (right_x, side_cy, wh / 2.0), "size": (wt, wl, wh)},
    }


def build_all_scenes_items(scene_json_paths: list, active_scene_index: int = 0,
                           pool_size: int = 10):
    """Build a *pooled* set of RigidObjectCfg shared across all scenes.

    Instead of spawning one object per scene-item (which explodes with 800+
    scenes), we spawn ``pool_size`` instances per *item type* from
    ``item_usds``.  Scene transitions simply reposition pool instances.

    Pool item names: ``pool_{label}_{i}``  (e.g. ``pool_Cuboid_40x40x80_3``).

    If a scene needs more instances of a type than ``pool_size``, the excess
    items are skipped and a warning is logged.

    Args:
        scene_json_paths: list of scene JSON paths (one per scene in the task).
        active_scene_index: index of the initially active scene.
        pool_size: number of instances to spawn per item type.

    Returns:
        items: list of ``(attr_name, RigidObjectCfg)`` — the pool.
        scene_placements: ``{scene_idx: [(pool_attr_name, pos, rot), ...]}``
        pool_names: list of all pool attr names (for SceneManager bulk ops).
    """
    import isaaclab.sim as sim_utils
    from isaaclab.assets import RigidObjectCfg
    from .rigid_usd_spawner import RigidUsdFileCfg
    import logging

    logger = logging.getLogger("cs_projects_scene_cfg")

    PARK_POS = (0.0, 0.0, -10.0)
    PARK_ROT = (1.0, 0.0, 0.0, 0.0)

    # --- 1. Collect item_usds from the first scene (shared across all) ---
    first_cfg = load_scene_config(scene_json_paths[0])
    item_usds = first_cfg.get("item_usds", {})

    # --- 2. Create pool: pool_size instances per item type ---
    items = []           # (attr_name, RigidObjectCfg)
    pool_names = []      # all pool attr names

    for label, info in item_usds.items():
        if not isinstance(info, dict):
            continue
        usd_path = info.get("usd", "")
        if not usd_path or not os.path.exists(usd_path):
            continue

        mass = _estimate_mass(info)

        for i in range(pool_size):
            safe_name = _sanitize_prim_name(f"pool_{label}_{i}")

            asset = RigidObjectCfg(
                prim_path=f"/World/envs/env_.*/{safe_name}",
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=PARK_POS,
                    rot=PARK_ROT,
                ),
                spawn=RigidUsdFileCfg(
                    usd_path=usd_path,
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(
                        disable_gravity=False,
                        max_depenetration_velocity=1.0,
                    ),
                    mass_props=sim_utils.MassPropertiesCfg(mass=mass),
                    collision_props=sim_utils.CollisionPropertiesCfg(
                        collision_enabled=True,
                    ),
                ),
            )
            items.append((safe_name, asset))
            pool_names.append(safe_name)

    logger.info(
        f"[Pool] Created {len(pool_names)} pool objects "
        f"({len(item_usds)} types x {pool_size} each)"
    )

    # --- 3. For each scene, map placements to pool slots ---
    scene_placements = {}

    for scene_idx, scene_json_path in enumerate(scene_json_paths):
        cfg = load_scene_config(scene_json_path)
        zone_config = cfg.get("zone_config", {})
        planes_dir = cfg["planes_dir"]
        scene_item_usds = cfg.get("item_usds", item_usds)

        planes = _load_planes(planes_dir)
        placements = _compute_placements(planes, zone_config, scene_item_usds)

        # Count how many of each type this scene needs
        type_counter = {}  # label -> next pool index
        scene_placement_list = []

        for p in placements:
            label = p["label"]
            pos = tuple(p["pos"])
            qw, qx, qy, qz = p["quat_wxyz"]
            rot = (qw, qx, qy, qz)

            idx = type_counter.get(label, 0)
            if idx >= pool_size:
                logger.warning(
                    f"[Pool] Scene {scene_idx}: type '{label}' needs "
                    f"{idx + 1} instances but pool_size={pool_size} — "
                    f"item skipped"
                )
                type_counter[label] = idx + 1
                continue

            pool_attr = _sanitize_prim_name(f"pool_{label}_{idx}")
            scene_placement_list.append((pool_attr, pos, rot))
            type_counter[label] = idx + 1

        scene_placements[scene_idx] = scene_placement_list

    # --- 4. Apply active scene positions to pool items ---
    active_placements = scene_placements.get(active_scene_index, [])
    active_lookup = {name: (pos, rot) for name, pos, rot in active_placements}

    for attr_name, asset_cfg in items:
        if attr_name in active_lookup:
            pos, rot = active_lookup[attr_name]
            asset_cfg.init_state.pos = pos
            asset_cfg.init_state.rot = rot

    return items, scene_placements, pool_names


def build_scene_assets(scene_json_path: str):
    """
    Parse a scene JSON and return lists of asset definitions suitable for
    setting as attributes on an InteractiveSceneCfg subclass.

    Items are returned as RigidObjectCfg with full physics (collision, mass,
    friction) so that they can be grasped and manipulated via teleoperation.

    Returns:
        dict with keys:
            "shelf": AssetBaseCfg for the shelf (kinematic)
            "walls": AssetBaseCfg or None (kinematic)
            "items": list of (attr_name, RigidObjectCfg) for each placed item
    """
    # Deferred imports — only needed when actually building scene assets
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
    cfg = load_scene_config(scene_json_path)

    item_usds = cfg.get("item_usds", {})
    zone_config = cfg.get("zone_config", {})
    shelf_usd = cfg["shelf_usd"]
    planes_dir = cfg["planes_dir"]
    walls_enabled = cfg.get("walls_enabled", True)
    wall_usd = cfg.get("wall_usd", "")

    # Load planes and compute placements
    planes = _load_planes(planes_dir)
    placements = _compute_placements(planes, zone_config, item_usds)

    # Shelf quaternion: OG [x,y,z,w] -> USD [w,x,y,z]
    shelf_quat_wxyz = _quat_xyzw_to_wxyz(_SHELF_QUAT_OG)

    result = {"shelf": None, "walls": [], "items": []}

    # Shelf — kinematic (static) with SDF mesh collision.
    # The shelf USD is purely visual (no physics APIs baked in).
    # KinematicUsdFileCfg applies RigidBodyAPI(kinematic) + PhysxSDFMeshCollisionAPI
    # to all visual meshes so items don't fall through the shelf surfaces.
    from .rigid_usd_spawner import KinematicUsdFileCfg

    result["shelf"] = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Shelf",
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=tuple(_SHELF_POS),
            rot=tuple(shelf_quat_wxyz),
        ),
        spawn=KinematicUsdFileCfg(
            usd_path=shelf_usd,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
    )

    # Walls — kinematic (static, collision)
    if walls_enabled and wall_usd and os.path.exists(wall_usd):
        # Pre-built wall USD file
        result["walls"].append(("walls", AssetBaseCfg(
            prim_path="/World/envs/env_.*/Walls",
            init_state=AssetBaseCfg.InitialStateCfg(
                pos=(0.0, 0.15, 0.0),
                rot=(1.0, 0.0, 0.0, 0.0),
            ),
            spawn=KinematicUsdFileCfg(
                usd_path=wall_usd,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True,
                ),
            ),
        )))
    elif walls_enabled:
        # Procedural ㄷ-shaped walls (matches generated_scenes4/shelf_cola/main.py)
        wall_geom = _compute_wall_geometry(planes, cfg)
        for wall_name, wg in wall_geom.items():
            result["walls"].append((f"wall_{wall_name}", AssetBaseCfg(
                prim_path=f"/World/envs/env_.*/Wall_{wall_name}",
                init_state=AssetBaseCfg.InitialStateCfg(
                    pos=wg["pos"],
                    rot=(1.0, 0.0, 0.0, 0.0),
                ),
                spawn=sim_utils.CuboidCfg(
                    size=wg["size"],
                    collision_props=sim_utils.CollisionPropertiesCfg(
                        collision_enabled=True,
                    ),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(
                        kinematic_enabled=True,
                    ),
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.85, 0.85, 0.85),
                        opacity=0.3,
                    ),
                ),
            )))

    # Items — dynamic rigid bodies (physics-enabled, graspable)
    #
    # The USD asset files are purely visual (no RigidBodyAPI baked in).
    # Isaac Lab's modify_rigid_body_properties() only MODIFIES existing
    # RigidBodyAPI — it never APPLIES one.  We therefore use a custom
    # spawner (RigidUsdFileCfg) that explicitly applies the API before
    # the standard schema pipeline runs.
    from .rigid_usd_spawner import RigidUsdFileCfg

    for p in placements:
        label = p["label"]
        idx = p["index"]
        usd_path = p["usd"]
        pos = p["pos"]
        qw, qx, qy, qz = p["quat_wxyz"]

        safe_name = _sanitize_prim_name(f"{label}_{idx}")

        if not os.path.exists(usd_path):
            continue

        # Look up item dimensions for mass estimation
        item_info = item_usds.get(label, {})
        if not isinstance(item_info, dict):
            item_info = {}
        mass = _estimate_mass(item_info)

        asset = RigidObjectCfg(
            prim_path=f"/World/envs/env_.*/{safe_name}",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=tuple(pos),
                rot=(qw, qx, qy, qz),
            ),
            spawn=RigidUsdFileCfg(
                usd_path=usd_path,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=False,
                    max_depenetration_velocity=1.0,
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=mass),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True,
                ),
            ),
        )
        result["items"].append((safe_name, asset))

    return result
