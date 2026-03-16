"""
Scene Manager — Handles scene lifecycle operations for the CS-Projects task.

Uses the **pooled pre-spawn approach**: a fixed number of instances per item
type are spawned at env init time.  Scene transitions reposition pool items
via ``write_root_pose_to_sim()`` — no USD prim creation or deletion at
runtime, which would invalidate physics tensor views.

Called by sim_main.py's main loop:
    scene_mgr.poll()   # once per tick
"""

import json
import logging
import torch
from typing import Optional

from .scene_server import SceneServer, CommandType

logger = logging.getLogger("scene_manager")

# Underground parking position (x, y, z, qw, qx, qy, qz)
_PARK_POSE = [0.0, 0.0, -10.0, 1.0, 0.0, 0.0, 0.0]


class SceneManager:
    """Manages scene transitions driven by the Scene API server.

    Usage in sim_main.py::

        scene_mgr = SceneManager(env, env_cfg, scene_server, scene_json_list)
        # In main loop:
        scene_mgr.poll()
    """

    def __init__(
        self,
        env,
        env_cfg,
        scene_server: SceneServer,
        scene_json_list: list[str],
        start_index: int = 0,
        task_num: int = 0,
    ):
        self.env = env
        self.env_cfg = env_cfg
        self.server = scene_server
        self.scene_json_list = scene_json_list
        self.current_index = start_index
        self.task_num = task_num

        # Pre-spawn placement data from env cfg (NOT scene cfg, to avoid
        # Isaac Lab treating it as an asset config)
        # {scene_idx: [(pool_attr_name, pos_tuple, rot_tuple), ...]}
        self.scene_placements: Optional[dict] = getattr(
            env_cfg, 'scene_placements', None
        )

        # Pool item names — the fixed set of spawned objects
        self.pool_names: list[str] = getattr(env_cfg, 'pool_names', None) or []

        if not self.pool_names and self.scene_placements:
            # Fallback: derive from placements (legacy non-pooled mode)
            names_set = set()
            for scene_idx in sorted(self.scene_placements.keys()):
                for attr_name, _pos, _rot in self.scene_placements[scene_idx]:
                    names_set.add(attr_name)
            self.pool_names = sorted(names_set)

        if self.pool_names:
            logger.info(
                f"[SceneManager] Pool mode: {len(self.pool_names)} pool objects, "
                f"{len(self.scene_placements)} scenes"
            )
        else:
            logger.warning("[SceneManager] No pool data — scene transitions disabled")

        # Update server state
        q = self.server.cmd_queue
        q.task_num = task_num
        q.total_scenes = len(scene_json_list)
        q.current_scene_index = start_index
        q.scene_json_path = scene_json_list[start_index] if scene_json_list else ""
        q.all_completed = len(scene_json_list) == 0
        q.task_id = f"CS-PROJECT-{task_num}-{start_index}"
        q.instructions = self._load_instructions(start_index)

        # Disable physics for all inactive pool items at startup
        if self.pool_names and self.scene_placements:
            active_names = {
                name for name, _, _ in self.scene_placements.get(start_index, [])
            }
            for name in self.pool_names:
                if name not in active_names:
                    self._set_physics_enabled(name, enabled=False)

    # ------------------------------------------------------------------
    # Main loop hook
    # ------------------------------------------------------------------

    def poll(self):
        """Check for pending commands and execute them. Call once per main loop tick."""
        cmd = self.server.cmd_queue.check_pending()
        if cmd is None:
            return

        try:
            if cmd.cmd == CommandType.RESET_CURRENT:
                self._reset_current_scene()
                self.server.cmd_queue.complete(
                    cmd, success=True, message="Scene reset to default positions"
                )
                logger.info(f"[SceneManager] Reset scene {self.current_index}")

            elif cmd.cmd == CommandType.LOAD_NEXT:
                next_idx = self.current_index + 1
                if next_idx >= len(self.scene_json_list):
                    # No more scenes
                    self.server.cmd_queue.all_completed = True
                    self.server.cmd_queue.complete(
                        cmd, success=True,
                        message="All tasks completed — no more scenes to load",
                    )
                    logger.info("[SceneManager] All scenes completed")
                else:
                    self._load_scene(next_idx)
                    self.server.cmd_queue.complete(
                        cmd, success=True,
                        message=f"Loaded scene {next_idx}/{len(self.scene_json_list) - 1}",
                    )
                    logger.info(f"[SceneManager] Loaded scene {next_idx}")

        except Exception as e:
            logger.error(f"[SceneManager] Command failed: {e}", exc_info=True)
            self.server.cmd_queue.complete(cmd, success=False, message=str(e))

    # ------------------------------------------------------------------
    # Scene operations (pooled pre-spawn approach)
    # ------------------------------------------------------------------

    def _reset_current_scene(self):
        """Reset all objects in the current scene to their correct positions.

        Also resets the robot via ``reset_scene_to_default``, then re-applies
        item positions (since default_root_state reflects init-time positions,
        not the current scene after a switch).
        """
        import isaaclab.envs.mdp as base_mdp

        # Re-enable physics on ALL pool items BEFORE reset_scene_to_default,
        # otherwise PhysX errors out on setVelocity for disabled bodies.
        self._enable_all_physics()

        # Reset robot + all rigid bodies to init-time defaults
        base_mdp.reset_scene_to_default(
            self.env,
            torch.arange(self.env.num_envs, device=self.env.device),
        )

        if self.scene_placements:
            # After reset_to_default, items are at init-time positions.
            # Park everything, then place current scene items correctly.
            self._park_all_items()
            self._apply_scene_positions(self.current_index)

    def _load_scene(self, scene_index: int):
        """Switch to a new scene by repositioning pool items.

        1. Re-enable physics on all pool items (avoid PhysX errors)
        2. Reset robot to default
        3. Park ALL pool items underground + disable physics
        4. Place new scene's items at their correct positions + enable physics
        5. Update server state
        """
        if not self.scene_placements:
            logger.error("[SceneManager] Cannot switch scene — no pre-spawn data")
            return

        import isaaclab.envs.mdp as base_mdp

        # Re-enable physics on ALL pool items BEFORE reset_scene_to_default
        self._enable_all_physics()

        # Reset robot to default pose
        base_mdp.reset_scene_to_default(
            self.env,
            torch.arange(self.env.num_envs, device=self.env.device),
        )

        # Park all pool items underground
        self._park_all_items()

        # Place new scene items at correct positions
        self._apply_scene_positions(scene_index)

        # Update state
        self.current_index = scene_index
        q = self.server.cmd_queue
        q.current_scene_index = scene_index
        q.scene_json_path = self.scene_json_list[scene_index]
        q.task_id = f"CS-PROJECT-{self.task_num}-{scene_index}"
        q.instructions = self._load_instructions(scene_index)

        logger.info(
            f"[SceneManager] Scene {scene_index}: "
            f"{len(self.scene_placements.get(scene_index, []))} items placed"
        )

    # ------------------------------------------------------------------
    # Item repositioning helpers
    # ------------------------------------------------------------------

    def _park_all_items(self):
        """Move ALL pool items underground, zero velocity, and disable physics."""
        device = self.env.device
        park_pose = torch.tensor([_PARK_POSE], dtype=torch.float32, device=device)
        zero_vel = torch.zeros((1, 6), dtype=torch.float32, device=device)
        env_ids = torch.tensor([0], dtype=torch.long, device=device)

        for name in self.pool_names:
            try:
                obj = self.env.scene[name]
                obj.write_root_pose_to_sim(park_pose, env_ids)
                obj.write_root_velocity_to_sim(zero_vel, env_ids)
                self._set_physics_enabled(name, enabled=False)
            except KeyError:
                logger.warning(f"[SceneManager] Pool item '{name}' not found in scene")
            except Exception as e:
                logger.warning(f"[SceneManager] Failed to park '{name}': {e}")

    def _apply_scene_positions(self, scene_index: int):
        """Reposition pool items for the given scene and re-enable their physics."""
        placements = self.scene_placements.get(scene_index, [])
        if not placements:
            logger.warning(f"[SceneManager] No placements for scene {scene_index}")
            return

        device = self.env.device
        zero_vel = torch.zeros((1, 6), dtype=torch.float32, device=device)
        env_ids = torch.tensor([0], dtype=torch.long, device=device)

        for attr_name, pos, rot in placements:
            try:
                obj = self.env.scene[attr_name]
                # Re-enable physics FIRST — velocity writes fail on disabled bodies
                self._set_physics_enabled(attr_name, enabled=True)
                # pose: (N, 7) = [x, y, z, qw, qx, qy, qz]
                pose = torch.tensor(
                    [[pos[0], pos[1], pos[2], rot[0], rot[1], rot[2], rot[3]]],
                    dtype=torch.float32,
                    device=device,
                )
                obj.write_root_pose_to_sim(pose, env_ids)
                obj.write_root_velocity_to_sim(zero_vel, env_ids)
            except KeyError:
                logger.warning(f"[SceneManager] Pool item '{attr_name}' not found in scene")
            except Exception as e:
                logger.warning(f"[SceneManager] Failed to reposition '{attr_name}': {e}")

    # ------------------------------------------------------------------
    # Physics toggle helpers
    # ------------------------------------------------------------------

    def _enable_all_physics(self):
        """Re-enable PhysX simulation on ALL pool items.

        Must be called before ``reset_scene_to_default()`` which internally
        writes velocities to every rigid body.  Without this, disabled bodies
        trigger ``PxRigidDynamic::setLinearVelocity: Not allowed if
        eDISABLE_SIMULATION is set`` errors that crash the simulation.
        """
        for name in self.pool_names:
            self._set_physics_enabled(name, enabled=True)

    def _set_physics_enabled(self, item_name: str, *, enabled: bool):
        """Enable or disable PhysX simulation for a pool item.

        Uses ``RigidBodyView.set_disable_simulations()`` from the PhysX tensor
        API.  When re-enabling, also calls ``wake_up()`` so the body responds
        to forces immediately instead of staying asleep.
        """
        try:
            obj = self.env.scene[item_name]
            view = obj.root_physx_view
            # flag: 0 = simulated, 1 = disabled
            flag = 0 if enabled else 1
            data = torch.tensor(
                [[flag]] * view.count, dtype=torch.uint8, device="cpu"
            )
            indices = torch.arange(view.count, dtype=torch.int32, device="cpu")
            view.set_disable_simulations(data, indices)
            if enabled:
                view.wake_up(indices)
        except KeyError:
            pass
        except Exception as e:
            logger.warning(
                f"[SceneManager] Failed to {'enable' if enabled else 'disable'} "
                f"physics for '{item_name}': {e}"
            )

    # ------------------------------------------------------------------
    # Scene JSON helpers
    # ------------------------------------------------------------------

    def _load_instructions(self, scene_index: int) -> dict:
        """Load vla_metadata.instructions from the scene JSON file."""
        if scene_index >= len(self.scene_json_list):
            return {}
        try:
            with open(self.scene_json_list[scene_index], "r", encoding="utf-8") as f:
                cfg = json.load(f)
            return cfg.get("vla_metadata", {}).get("instructions", {})
        except Exception as e:
            logger.warning(f"[SceneManager] Failed to load instructions for scene {scene_index}: {e}")
            return {}
