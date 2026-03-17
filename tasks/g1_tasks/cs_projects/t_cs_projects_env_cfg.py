"""
Environment configuration for T-CS-PROJECT (converted scene loader).

Simplified version of CSProjectsEnvCfg that loads pre-converted scene JSONs
with world coordinates. No scene-api dependency, no multi-scene pooling.
Single scene per environment — shelf + walls + items + robot (G1 dex3).
"""

import os
import torch

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from tasks.common_config import G1RobotPresets, CameraPresets, CameraBaseCfg
from tasks.common_event.event_manager import SimpleEvent, SimpleEventManager

from . import mdp
from .t_cs_projects_scene_cfg import build_scene_assets_from_converted


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------

@configclass
class TCSProjectsSceneCfg(InteractiveSceneCfg):
    """Scene config for T-CS-PROJECT.

    Static assets (light, robot, cameras) are declared as class attributes.
    Dynamic assets (shelf, walls, items) are injected from the converted JSON.
    """

    # Light
    light: AssetBaseCfg = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # World camera
    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=(-0.1, 3.6, 1.6),
        rot_offset=(-0.00617, 0.00617, 0.70708, -0.70708),
        focal_length=16.5,
    )

    # Robot — same position as CS-PROJECT
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex3_base_fix_waist(
        init_pos=(0.10, -0.40, 0.76),
    )

    # Cameras
    front_camera = CameraPresets.g1_front_camera()
    left_wrist_camera = CameraPresets.left_dex3_wrist_camera()
    right_wrist_camera = CameraPresets.right_dex3_wrist_camera()

    def inject_scene_assets(self, scene_json_path: str):
        """Load converted scene assets and inject them as attributes."""
        assets = build_scene_assets_from_converted(scene_json_path)

        if assets["shelf"] is not None:
            setattr(self, "shelf", assets["shelf"])

        for attr_name, asset_cfg in assets["walls"]:
            setattr(self, attr_name, asset_cfg)

        for attr_name, asset_cfg in assets["items"]:
            setattr(self, attr_name, asset_cfg)


# ---------------------------------------------------------------------------
# MDP configs (same as CS-PROJECT)
# ---------------------------------------------------------------------------

@configclass
class ActionsCfg:
    joint_pos = mdp.JointPositionActionCfg(
        asset_name="robot", joint_names=[".*"], scale=1.0, use_default_offset=True
    )


@configclass
class ObservationsCfg:
    @configclass
    class PolicyCfg(ObsGroup):
        robot_joint_state = ObsTerm(func=mdp.get_robot_boy_joint_states)
        robot_dex3_state = ObsTerm(func=mdp.get_robot_dex3_joint_states)
        camera_image = ObsTerm(func=mdp.get_camera_image)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class TerminationsCfg:
    time_out = DoneTerm(func=mdp.time_out, time_out=True)


@configclass
class RewardsCfg:
    pass


@configclass
class EventCfg:
    pass


# ---------------------------------------------------------------------------
# Env config
# ---------------------------------------------------------------------------

@configclass
class TCSProjectsEnvCfg(ManagerBasedRLEnvCfg):
    """Environment configuration for a single T-CS-PROJECT scene.

    Loads a pre-converted scene JSON (world coordinates) and spawns
    shelf + walls + items + G1 robot. No scene-api required.
    """

    scene: TCSProjectsSceneCfg = TCSProjectsSceneCfg(
        num_envs=1, env_spacing=2.5, replicate_physics=True
    )

    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    rewards: RewardsCfg = RewardsCfg()
    events: EventCfg = EventCfg()
    commands = None
    curriculum = None

    # --- set by factory before instantiation ---
    scene_json_path: str = ""

    def __post_init__(self):
        self.decimation = 2
        self.episode_length_s = 20.0

        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.enable_ccd = True
        self.sim.physx.contact_offset = 0.01
        self.sim.physx.rest_offset = 0.001
        self.sim.physx.num_position_iterations = 16
        self.sim.physx.num_velocity_iterations = 4
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

        # Simple reset event
        def _reset_fn(env):
            base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device),
            )

        self.event_manager = SimpleEventManager()
        self.event_manager.register("reset_all_self", SimpleEvent(func=_reset_fn))
        self.event_manager.register("reset_object_self", SimpleEvent(func=_reset_fn))

        # Inject scene assets from the converted JSON
        if self.scene_json_path:
            self.scene.inject_scene_assets(self.scene_json_path)


# ---------------------------------------------------------------------------
# Factory for gym registration
# ---------------------------------------------------------------------------

def get_t_env_cfg_class(task_num: int, scene_index: int):
    """Return TCSProjectsEnvCfg class pre-configured for given task/scene."""
    from . import T_SCENE_REGISTRY

    scene_path = T_SCENE_REGISTRY.get((task_num, scene_index))
    if scene_path is None:
        raise ValueError(
            f"T-CS-PROJECT-{task_num}-{scene_index} not found in T_SCENE_REGISTRY. "
            f"Available: {sorted(T_SCENE_REGISTRY.keys())}"
        )

    @configclass
    class _BoundEnvCfg(TCSProjectsEnvCfg):
        scene_json_path: str = scene_path

    return _BoundEnvCfg
