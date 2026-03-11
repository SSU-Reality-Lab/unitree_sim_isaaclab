"""CS-Projects: dynamically registered gym environments from generated scene JSONs.

On import, this module discovers task{N}_LLM_scene directories under the
CS_PROJECTS environment variable and registers each scene as a gymnasium
environment with id "CS-PROJECT-{task}-{scene}" (e.g., CS-PROJECT-1-3).

The env config class is built lazily (only when the specific task is requested),
so importing this module is fast even with hundreds of scenes.

If CS_PROJECTS is not set, no environments are registered (silent skip so
the rest of the task system still loads).

Usage:
    export CS_PROJECTS=/path/to/generated_scenes4
    python sim_main.py --task CS-PROJECT-1-0 --device cpu --enable_cameras --enable_dex3_dds --robot_type g129
"""

import os
import glob
import gymnasium as gym

from .cs_projects_scene_cfg import find_scene_jsons

# Only register if CS_PROJECTS is configured
_cs_dir = os.environ.get("CS_PROJECTS", "")

# Build a mapping from (task_type, scene_index) -> scene path so the factory can look it up.
# This is a module-level dict read by cs_projects_env_cfg.get_env_cfg_class().
SCENE_REGISTRY: dict[tuple[int, int], str] = {}

if _cs_dir and os.path.isdir(_cs_dir):
    # Discover task{N}_LLM_scene directories
    _task_dirs = sorted(glob.glob(os.path.join(_cs_dir, "task*_LLM_scene")))
    _total_registered = 0

    for _task_dir in _task_dirs:
        # Extract task number from directory name (e.g., task1_LLM_scene -> 1)
        _task_name = os.path.basename(_task_dir)
        try:
            _task_num = int(_task_name.split("_")[0].replace("task", ""))
        except (ValueError, IndexError):
            continue

        _scene_files = find_scene_jsons(_task_dir)
        if not _scene_files:
            continue

        for _i, _scene_path in enumerate(_scene_files):
            SCENE_REGISTRY[(_task_num, _i)] = _scene_path

            def _make_factory(task_num=_task_num, scene_idx=_i):
                def _factory():
                    from .cs_projects_env_cfg import get_env_cfg_class
                    cls = get_env_cfg_class(task_num, scene_idx)
                    return cls()
                return _factory

            gym.register(
                id=f"CS-PROJECT-{_task_num}-{_i}",
                entry_point="isaaclab.envs:ManagerBasedRLEnv",
                kwargs={
                    "env_cfg_entry_point": _make_factory(_task_num, _i),
                },
                disable_env_checker=True,
            )

        print(f"[CS-Projects] Task {_task_num}: registered {len(_scene_files)} scenes "
              f"(CS-PROJECT-{_task_num}-0 .. CS-PROJECT-{_task_num}-{len(_scene_files) - 1})")
        _total_registered += len(_scene_files)

    if _total_registered:
        print(f"[CS-Projects] Total: {_total_registered} environments registered")
    else:
        print(f"[CS-Projects] No scene_*.json found under task*_LLM_scene dirs in CS_PROJECTS={_cs_dir}")
elif _cs_dir:
    print(f"[CS-Projects] CS_PROJECTS={_cs_dir} is not a valid directory, skipping registration")
