"""CS-Projects: dynamically registered gym environments from generated scene JSONs.

On import, this module discovers all scene_*.json files under the directory
specified by the CS_PROJECTS environment variable and registers each one as
a gymnasium environment with id "CS-Projects-{i}" (0-indexed).

The env config class is built lazily (only when the specific task is requested),
so importing this module is fast even with hundreds of scenes.

If CS_PROJECTS is not set, no environments are registered (silent skip so
the rest of the task system still loads).

Usage:
    export CS_PROJECTS=/path/to/generated_scenes2
    python sim_main.py --task CS-Projects-0 --device cpu --enable_cameras --enable_dex3_dds --robot_type g129
"""

import os
import gymnasium as gym

from .cs_projects_scene_cfg import find_scene_jsons

# Only register if CS_PROJECTS is configured
_cs_dir = os.environ.get("CS_PROJECTS", "")

if _cs_dir and os.path.isdir(_cs_dir):
    _scene_files = find_scene_jsons(_cs_dir)

    # Build a mapping from index -> scene path so the factory can look it up.
    # This is a module-level dict read by cs_projects_env_cfg.get_env_cfg_class().
    SCENE_REGISTRY: dict[int, str] = {}

    for _i, _scene_path in enumerate(_scene_files):
        SCENE_REGISTRY[_i] = _scene_path

        # Register with a callable entry point that lazily builds the config class.
        # When parse_env_cfg calls load_cfg_from_registry -> cfg_entry_point(),
        # it invokes our factory which builds the class at that moment.
        #
        # We use a default-arg closure to capture _i by value.
        def _make_factory(scene_idx=_i):
            def _factory():
                from .cs_projects_env_cfg import get_env_cfg_class
                cls = get_env_cfg_class(scene_idx)
                return cls()
            return _factory

        gym.register(
            id=f"CS-Projects-{_i}",
            entry_point="isaaclab.envs:ManagerBasedRLEnv",
            kwargs={
                "env_cfg_entry_point": _make_factory(_i),
            },
            disable_env_checker=True,
        )

    if _scene_files:
        print(f"[CS-Projects] Registered {len(_scene_files)} environments "
              f"(CS-Projects-0 .. CS-Projects-{len(_scene_files) - 1})")
    else:
        print(f"[CS-Projects] No scene_*.json found under CS_PROJECTS={_cs_dir}")
else:
    SCENE_REGISTRY: dict[int, str] = {}
    if _cs_dir:
        print(f"[CS-Projects] CS_PROJECTS={_cs_dir} is not a valid directory, skipping registration")
