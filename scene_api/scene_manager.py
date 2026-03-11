"""
Scene Manager — Handles scene lifecycle operations for the CS-Projects task.

This module is called by the sim_main.py main loop to:
  1. Poll for pending API commands (reset / next)
  2. Execute scene reset (restore object positions from JSON)
  3. Execute scene load (remove old objects, spawn new ones from next JSON)

It bridges the SceneCommandQueue (from scene_server.py) with the Isaac Sim
environment and the CS-Projects scene config system.
"""

import os
import logging
import torch
from typing import Optional

from .scene_server import SceneServer, CommandType

logger = logging.getLogger("scene_manager")


class SceneManager:
    """Manages scene transitions driven by the Scene API server.

    Usage in sim_main.py:
        scene_mgr = SceneManager(env, env_cfg, scene_server, scene_json_list)
        # In main loop:
        scene_mgr.poll(env)
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

        # Update server state
        q = self.server.cmd_queue
        q.task_num = task_num
        q.total_scenes = len(scene_json_list)
        q.current_scene_index = start_index
        q.scene_json_path = scene_json_list[start_index] if scene_json_list else ""
        q.all_completed = len(scene_json_list) == 0

    def poll(self):
        """Check for pending commands and execute them. Call once per main loop tick."""
        cmd = self.server.cmd_queue.check_pending()
        if cmd is None:
            return

        try:
            if cmd.cmd == CommandType.RESET_CURRENT:
                self._reset_current_scene()
                self.server.cmd_queue.complete(cmd, success=True, message="Scene reset to default positions")
                logger.info(f"[SceneManager] Reset scene {self.current_index}")

            elif cmd.cmd == CommandType.LOAD_NEXT:
                next_idx = self.current_index + 1
                if next_idx >= len(self.scene_json_list):
                    # No more scenes
                    self.server.cmd_queue.all_completed = True
                    self.server.cmd_queue.complete(
                        cmd, success=True,
                        message="All tasks completed — no more scenes to load"
                    )
                    logger.info("[SceneManager] All scenes completed")
                else:
                    self._load_scene(next_idx)
                    self.server.cmd_queue.complete(
                        cmd, success=True,
                        message=f"Loaded scene {next_idx}/{len(self.scene_json_list) - 1}"
                    )
                    logger.info(f"[SceneManager] Loaded scene {next_idx}")

        except Exception as e:
            logger.error(f"[SceneManager] Command failed: {e}")
            self.server.cmd_queue.complete(cmd, success=False, message=str(e))

    def _reset_current_scene(self):
        """Reset all dynamic objects in the current scene to their default positions."""
        import isaaclab.envs.mdp as base_mdp

        # Use the event manager's reset_all_self event if registered
        if hasattr(self.env_cfg, 'event_manager'):
            self.env_cfg.event_manager.trigger("reset_all_self", self.env)
        else:
            # Fallback: direct reset
            base_mdp.reset_scene_to_default(
                self.env,
                torch.arange(self.env.num_envs, device=self.env.device),
            )

    def _load_scene(self, scene_index: int):
        """Load a new scene by index: remove old dynamic prims and spawn new ones.

        This performs a "hot swap" of scene objects:
          1. Delete existing item prims from the USD stage
          2. Rebuild scene assets from the new JSON
          3. Inject new assets into the scene
          4. Reset the simulation
        """
        import omni.usd
        from pxr import Usd, UsdGeom
        import isaaclab.envs.mdp as base_mdp

        scene_json_path = self.scene_json_list[scene_index]
        stage = omni.usd.get_context().get_stage()

        # --- Step 1: Remove existing dynamic item prims ---
        # Items are spawned under /World/envs/env_0/<safe_name>
        # We identify them by checking which prims are RigidBody enabled
        # and NOT the robot or shelf.
        env_prim_path = "/World/envs/env_0"
        env_prim = stage.GetPrimAtPath(env_prim_path)

        keep_prefixes = {"Robot", "Shelf", "Wall_", "Walls", "light",
                         "front_camera", "left_wrist_camera", "right_wrist_camera",
                         "PerspectiveCamera", "GroundPlane"}

        if env_prim.IsValid():
            children_to_remove = []
            for child in env_prim.GetChildren():
                name = child.GetName()
                if not any(name.startswith(pfx) for pfx in keep_prefixes):
                    children_to_remove.append(child.GetPath().pathString)

            for prim_path in children_to_remove:
                stage.RemovePrim(prim_path)
                logger.info(f"  Removed prim: {prim_path}")

        # --- Step 2: Rebuild scene assets from new JSON ---
        from tasks.g1_tasks.cs_projects.cs_projects_scene_cfg import build_scene_assets
        new_assets = build_scene_assets(scene_json_path)

        # --- Step 3: Spawn new item prims ---
        # We need to use the spawner to create new prims on the stage
        for attr_name, asset_cfg in new_assets["items"]:
            try:
                # Resolve the prim path for env_0
                prim_path = asset_cfg.prim_path.replace("env_.*", "env_0")
                spawn_cfg = asset_cfg.spawn
                # Spawn the prim
                spawn_cfg.func(
                    prim_path,
                    spawn_cfg,
                    translation=asset_cfg.init_state.pos,
                    orientation=asset_cfg.init_state.rot,
                )
                logger.info(f"  Spawned: {prim_path}")
            except Exception as e:
                logger.error(f"  Failed to spawn {attr_name}: {e}")

        # --- Step 4: Reset simulation to settle new objects ---
        self.env.sim.reset()
        self.env.reset()

        # --- Step 5: Update state ---
        self.current_index = scene_index
        q = self.server.cmd_queue
        q.current_scene_index = scene_index
        q.scene_json_path = scene_json_path
