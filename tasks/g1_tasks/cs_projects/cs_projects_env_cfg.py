"""
Environment configuration for CS-Projects generated scenes.

Builds a ManagerBasedRLEnvCfg whose InteractiveSceneCfg is populated at
__post_init__ time from a scene JSON.  The robot (G1 dex3) and cameras use
the exact same coordinates as pick_place_cylinder_g1_29dof_dex3.
"""

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
from .cs_projects_scene_cfg import build_scene_assets


# ---------------------------------------------------------------------------
# Scene
# ---------------------------------------------------------------------------

@configclass
class CSProjectsSceneCfg(InteractiveSceneCfg):
    """Base scene config for CS-Projects.

    Static assets (light, robot, cameras) are declared as normal class
    attributes.  Dynamic assets (shelf, walls, items) are injected at
    runtime via ``inject_scene_assets()``.
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

    # Robot — positioned to align with the convenience store shelf scene
    robot: ArticulationCfg = G1RobotPresets.g1_29dof_dex3_base_fix_waist(
        init_pos=(-0.15, -0.403, 0.76),
    )

    # Cameras
    front_camera = CameraPresets.g1_front_camera()
    left_wrist_camera = CameraPresets.left_dex3_wrist_camera()
    right_wrist_camera = CameraPresets.right_dex3_wrist_camera()

    def inject_scene_assets(self, scene_json_path: str):
        """Load a scene JSON and inject shelf/wall/item assets as attributes."""
        assets = build_scene_assets(scene_json_path)

        if assets["shelf"] is not None:
            setattr(self, "shelf", assets["shelf"])

        if assets["walls"] is not None:
            setattr(self, "walls", assets["walls"])

        for attr_name, asset_cfg in assets["items"]:
            setattr(self, attr_name, asset_cfg)


# ---------------------------------------------------------------------------
# MDP configs (reused from existing modules)
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
class CSProjectsEnvCfg(ManagerBasedRLEnvCfg):
    """Environment configuration for a single CS-Projects scene.

    The ``scene_json_path`` must be set *before* ``__post_init__`` runs.
    The factory function ``get_env_cfg_class()`` below handles this.
    """

    # Scene (static portion — dynamic assets injected in __post_init__)
    scene: CSProjectsSceneCfg = CSProjectsSceneCfg(
        num_envs=1, env_spacing=2.5, replicate_physics=True
    )

    # MDP
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
        # General settings
        self.decimation = 2
        self.episode_length_s = 20.0

        # Simulation settings
        self.sim.dt = 0.005
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

        # Event manager
        self.event_manager = SimpleEventManager()
        self.event_manager.register("reset_all_self", SimpleEvent(
            func=lambda env: base_mdp.reset_scene_to_default(
                env,
                torch.arange(env.num_envs, device=env.device),
            )
        ))

        # Inject dynamic scene assets from the JSON
        if self.scene_json_path:
            self.scene.inject_scene_assets(self.scene_json_path)


# ---------------------------------------------------------------------------
# Lazy factory used by __init__.py's gym registration
# ---------------------------------------------------------------------------

def get_env_cfg_class(scene_index: int):
    """Return the CSProjectsEnvCfg *class* pre-configured for the given scene index.

    This is called lazily by the callable registered in gym.register kwargs,
    so scene JSON parsing only happens when the specific task is requested.
    """
    from . import SCENE_REGISTRY

    scene_path = SCENE_REGISTRY.get(scene_index)
    if scene_path is None:
        raise ValueError(
            f"CS-Projects scene index {scene_index} not found in SCENE_REGISTRY. "
            f"Available: {sorted(SCENE_REGISTRY.keys())}"
        )

    # Return a subclass that bakes in the scene_json_path as a default.
    # This way, when load_cfg_from_registry calls cls() it gets a properly
    # configured instance.
    @configclass
    class _BoundEnvCfg(CSProjectsEnvCfg):
        scene_json_path: str = scene_path

    return _BoundEnvCfg
