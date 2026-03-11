"""Custom USD spawner that applies RigidBodyAPI + CollisionAPI before Isaac Lab's schema pipeline.

Isaac Lab's ``modify_rigid_body_properties`` only *modifies* prims that already
carry ``UsdPhysics.RigidBodyAPI``.  The generated-scene USD assets are purely
visual (mesh + material) with no physics APIs baked in, so the standard
``UsdFileCfg`` silently skips rigid-body setup and ``RigidObjectCfg`` then
fails with:

    "Failed to find a rigid body ... Please ensure that the prim has
     'USD RigidBodyAPI' applied."

``RigidUsdFileCfg`` / ``spawn_rigid_usd`` fix this by explicitly applying
``UsdPhysics.RigidBodyAPI`` and ``UsdPhysics.CollisionAPI`` to the root prim
*before* delegating to the normal ``spawn_from_usd`` path.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.sim.spawners.from_files import from_files as _ff
from isaaclab.sim.utils import clone
from isaaclab.utils import configclass

if TYPE_CHECKING:
    from pxr import Usd


@clone
def spawn_rigid_usd(
    prim_path: str,
    cfg: "RigidUsdFileCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> "Usd.Prim":
    """Spawn a USD file and ensure RigidBodyAPI + CollisionAPI are present."""
    # 1. Delegate to the standard USD spawner (creates prim, applies variants,
    #    modifies rigid/collision/mass props for prims that already have them).
    prim = _ff._spawn_from_usd_file(prim_path, cfg.usd_path, cfg, translation, orientation)

    # 2. Now apply the physics APIs that the asset is missing.
    from pxr import UsdPhysics, PhysxSchema, Usd as _Usd, UsdGeom

    stage = prim.GetStage()
    root = stage.GetPrimAtPath(prim_path)

    # Apply RigidBodyAPI to the root prim (makes it a dynamic rigid body).
    if not root.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(root)

    # Apply PhysxRigidBodyAPI for extended PhysX properties.
    if not root.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(root)

    # Apply CollisionAPI to every Gprim (Mesh, Cube, Sphere, Cylinder, etc.)
    # under the root so PhysX generates collision geometry.
    # Previously only checked UsdGeom.Mesh — missed primitive shapes like Cube.
    gprim_count = 0
    for child in _Usd.PrimRange(root):
        if child.IsA(UsdGeom.Gprim):
            gprim_count += 1
            if not child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(child)
            # For Mesh prims, use convex hull collision.
            if child.IsA(UsdGeom.Mesh):
                if not child.HasAPI(UsdPhysics.MeshCollisionAPI):
                    mesh_col_api = UsdPhysics.MeshCollisionAPI.Apply(child)
                    mesh_col_api.GetApproximationAttr().Set("convexDecomposition")

    if gprim_count == 0:
        print(f"[rigid_usd_spawner] WARNING: No geometry prims found in {prim_path}")

    # 3. Re-run the schema modifiers now that the APIs exist on the prim.
    from isaaclab.sim import schemas

    if cfg.rigid_props is not None:
        schemas.modify_rigid_body_properties(prim_path, cfg.rigid_props)
    if cfg.collision_props is not None:
        schemas.modify_collision_properties(prim_path, cfg.collision_props)
    if cfg.mass_props is not None:
        schemas.modify_mass_properties(prim_path, cfg.mass_props)

    return prim


@configclass
class RigidUsdFileCfg(UsdFileCfg):
    """UsdFileCfg variant that guarantees RigidBodyAPI + CollisionAPI on spawn.

    Drop-in replacement for ``UsdFileCfg`` when loading USD assets that lack
    baked-in physics APIs but need to be used as dynamic rigid bodies.
    """

    func = spawn_rigid_usd


# ---------------------------------------------------------------------------
# Kinematic (static) USD spawner — for shelf / walls
# ---------------------------------------------------------------------------

@clone
def spawn_kinematic_usd(
    prim_path: str,
    cfg: "KinematicUsdFileCfg",
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
) -> "Usd.Prim":
    """Spawn a USD file as a kinematic rigid body with SDF mesh collision.

    Mirrors the logic of ``shelf_setup.setup_collision_sdf`` from the
    generated_scenes4 loader:
      1. Apply ``RigidBodyAPI`` with ``kinematicEnabled = True``
      2. Disable collision on ``/collisions/`` sub-meshes (convex hull proxies)
      3. Apply ``PhysxSDFMeshCollisionAPI`` to all other ``Mesh`` prims
    """
    prim = _ff._spawn_from_usd_file(prim_path, cfg.usd_path, cfg, translation, orientation)

    from pxr import UsdPhysics, PhysxSchema, Usd as _Usd, UsdGeom

    stage = prim.GetStage()
    root = stage.GetPrimAtPath(prim_path)

    # --- RigidBodyAPI (kinematic) on root ---
    if not root.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(root)
    UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Set(True)

    if not root.HasAPI(PhysxSchema.PhysxRigidBodyAPI):
        PhysxSchema.PhysxRigidBodyAPI.Apply(root)

    # --- Collision on mesh children ---
    sdf_count = 0
    for child in _Usd.PrimRange(root):
        # Ensure any existing rigid sub-bodies are also kinematic
        if child.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(child).GetKinematicEnabledAttr().Set(True)

        if child.IsA(UsdGeom.Mesh):
            child_path = str(child.GetPath())

            # Disable convex-hull proxy meshes (commonly under /collisions/)
            if "/collisions/" in child_path:
                if child.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Set(False)
                continue

            # Remove triangle-mesh collision if present (replaced by SDF)
            if child.HasAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI):
                child.RemoveAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI)

            # Apply CollisionAPI
            if not child.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI.Apply(child)
            UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Set(True)

            # Apply SDF mesh collision (accurate for complex shelf geometry)
            if not child.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
                PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(child)

            # PhysX collision params
            if not child.HasAPI(PhysxSchema.PhysxCollisionAPI):
                PhysxSchema.PhysxCollisionAPI.Apply(child)
            physx_col = PhysxSchema.PhysxCollisionAPI(child)
            physx_col.CreateContactOffsetAttr().Set(0.002)
            physx_col.CreateRestOffsetAttr().Set(0.001)
            sdf_count += 1

    if sdf_count == 0:
        print(f"[kinematic_usd_spawner] WARNING: No visual meshes found in {prim_path}")
    else:
        print(f"[kinematic_usd_spawner] Applied SDF collision to {sdf_count} meshes in {prim_path}")

    # Re-run schema modifiers
    from isaaclab.sim import schemas

    if cfg.rigid_props is not None:
        schemas.modify_rigid_body_properties(prim_path, cfg.rigid_props)
    if cfg.collision_props is not None:
        schemas.modify_collision_properties(prim_path, cfg.collision_props)

    return prim


@configclass
class KinematicUsdFileCfg(UsdFileCfg):
    """UsdFileCfg variant for static/kinematic collision objects (shelf, walls).

    Applies ``RigidBodyAPI`` (kinematic) + ``PhysxSDFMeshCollisionAPI`` to all
    visual meshes so that dynamic items can collide with the surface.
    """

    func = spawn_kinematic_usd
