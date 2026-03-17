"""
Scene configuration loader for T-CS-PROJECT (converted / world-coordinate JSONs).

Reads a pre-converted scene JSON where all positions and orientations are
already in world coordinates.  No plane analysis or coordinate conversion needed.

Asset paths in the JSON are relative to T_CS_PROJECTS (the convert/ directory).
"""

import os
import json
import glob


def _get_t_cs_projects_dir() -> str:
    d = os.environ.get("T_CS_PROJECTS", "")
    if not d:
        raise EnvironmentError(
            "T_CS_PROJECTS environment variable is not set. "
            "Set it to the convert/ directory (e.g., ./utiles/convert)."
        )
    return d


def _resolve(path: str, base_dir: str) -> str:
    if not path:
        return path
    p = path.replace("\\", "/")
    if ":" not in p and not p.startswith("/"):
        return os.path.normpath(os.path.join(base_dir, p)).replace("\\", "/")
    return p


def find_converted_scene_jsons(root_dir: str = None):
    """Find scene_*.json files under root_dir/task*/."""
    if root_dir is None:
        root_dir = _get_t_cs_projects_dir()
    pattern = os.path.join(root_dir, "task*", "scene_*.json")
    return sorted(glob.glob(pattern))


def load_converted_scene(scene_json_path: str) -> dict:
    """Load a converted scene JSON (already in world coordinates)."""
    with open(scene_json_path, "r", encoding="utf-8") as f:
        return json.load(f)


def _sanitize_prim_name(name: str) -> str:
    result = ""
    for ch in name:
        if ch.isalnum() or ch == "_":
            result += ch
        else:
            result += "_"
    if result and result[0].isdigit():
        result = "_" + result
    return result or "_unnamed"


def build_scene_assets_from_converted(scene_json_path: str):
    """Parse a converted scene JSON and return Isaac Lab asset definitions.

    The converted JSON has all positions/orientations in world coordinates,
    so this function simply reads and wraps them in Isaac Lab config objects.

    Returns:
        dict with keys:
            "shelf": AssetBaseCfg for the shelf (kinematic)
            "walls": list of (attr_name, AssetBaseCfg) for walls
            "items": list of (attr_name, RigidObjectCfg) for each item
    """
    import isaaclab.sim as sim_utils
    from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
    from .rigid_usd_spawner import RigidUsdFileCfg, KinematicUsdFileCfg

    base_dir = _get_t_cs_projects_dir()
    scene = load_converted_scene(scene_json_path)

    result = {"shelf": None, "walls": [], "items": []}

    # --- Shelf ---
    shelf = scene.get("shelf")
    if shelf:
        shelf_usd = _resolve(shelf["usd"], base_dir)
        pos = tuple(shelf["position"])
        rot = tuple(shelf["orientation_wxyz"])

        if os.path.exists(shelf_usd):
            result["shelf"] = AssetBaseCfg(
                prim_path="/World/envs/env_.*/Shelf",
                init_state=AssetBaseCfg.InitialStateCfg(pos=pos, rot=rot),
                spawn=KinematicUsdFileCfg(
                    usd_path=shelf_usd,
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                ),
            )

    # --- Walls ---
    for wall in scene.get("walls", []):
        name = wall.get("name", "wall")
        safe_name = f"wall_{name}"
        pos = tuple(wall["position"])
        rot = tuple(wall.get("orientation_wxyz", [1.0, 0.0, 0.0, 0.0]))

        if wall.get("type") == "usd" and wall.get("usd"):
            wall_usd = _resolve(wall["usd"], base_dir)
            if os.path.exists(wall_usd):
                result["walls"].append((safe_name, AssetBaseCfg(
                    prim_path=f"/World/envs/env_.*/Wall_{name}",
                    init_state=AssetBaseCfg.InitialStateCfg(pos=pos, rot=rot),
                    spawn=KinematicUsdFileCfg(
                        usd_path=wall_usd,
                        rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                        collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    ),
                )))
        else:
            # Cuboid wall
            size = tuple(wall["size"])
            result["walls"].append((safe_name, AssetBaseCfg(
                prim_path=f"/World/envs/env_.*/Wall_{name}",
                init_state=AssetBaseCfg.InitialStateCfg(pos=pos, rot=rot),
                spawn=sim_utils.CuboidCfg(
                    size=size,
                    collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=True),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
                    visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=(0.85, 0.85, 0.85),
                        opacity=0.3,
                    ),
                ),
            )))

    # --- Items ---
    for item in scene.get("items", []):
        label = item["label"]
        idx = item["index"]
        usd_path = _resolve(item["usd"], base_dir)
        pos = tuple(item["position"])
        qw, qx, qy, qz = item["orientation_wxyz"]

        safe_name = _sanitize_prim_name(f"{label}_{idx}")

        if not os.path.exists(usd_path):
            print(f"[T-CS-Projects] WARNING: USD not found: {usd_path}")
            continue

        physics = item.get("physics", {})
        mass = physics.get("mass", 0.2)

        asset = RigidObjectCfg(
            prim_path=f"/World/envs/env_.*/{safe_name}",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=pos,
                rot=(qw, qx, qy, qz),
            ),
            spawn=RigidUsdFileCfg(
                usd_path=usd_path,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=not physics.get("gravity", True),
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=mass),
                collision_props=sim_utils.CollisionPropertiesCfg(
                    collision_enabled=True,
                    contact_offset=physics.get("contact_offset", 0.01),
                    rest_offset=physics.get("rest_offset", 0.005),
                ),
                physics_material=sim_utils.RigidBodyMaterialCfg(
                    friction_combine_mode=physics.get("friction_combine_mode", "max"),
                    restitution_combine_mode=physics.get("restitution_combine_mode", "min"),
                    static_friction=physics.get("static_friction", 1.5),
                    dynamic_friction=physics.get("dynamic_friction", 1.5),
                    restitution=physics.get("restitution", 0.0),
                ),
            ),
        )
        result["items"].append((safe_name, asset))

    return result
