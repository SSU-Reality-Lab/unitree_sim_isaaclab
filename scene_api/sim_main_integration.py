"""
=== sim_main.py Integration Example ===

This file shows the EXACT code changes needed in sim_main.py to integrate the
Scene API server. Apply these changes as a diff to the existing sim_main.py.

Three integration points:
  1. Import + CLI arg (top of file)
  2. Server + SceneManager initialization (after env creation)
  3. Main loop polling (inside the while loop)
  4. Cleanup (in finally block)
"""


# ═══════════════════════════════════════════════════════════════════════════
# CHANGE 1: Add CLI argument (after line 83, with other parser.add_argument)
# ═══════════════════════════════════════════════════════════════════════════

# parser.add_argument("--scene-api-port", type=int, default=8200,
#                     help="Port for Scene Control API server (0 to disable)")
# parser.add_argument("--scene-api-host", type=str, default="0.0.0.0",
#                     help="Host for Scene Control API server")


# ═══════════════════════════════════════════════════════════════════════════
# CHANGE 2: Initialize SceneServer + SceneManager
#           (after line 405 "create dds success", before action provider)
#
# Only activates for CS-Projects tasks.
# ═══════════════════════════════════════════════════════════════════════════

def _init_scene_api_example(args_cli, env, env_cfg):
    """Example initialization — paste this logic into sim_main.py main()."""

    scene_server = None
    scene_manager = None

    # Only start Scene API for CS-Projects tasks
    if (args_cli.task.startswith("CS-Projects")
            and not args_cli.replay_data
            and getattr(args_cli, 'scene_api_port', 8200) > 0):

        from scene_api import SceneServer, SceneManager
        from tasks.g1_tasks.cs_projects.cs_projects_scene_cfg import find_scene_jsons
        import os

        # Discover all scene JSONs (same list used by CS-Projects registration)
        cs_dir = os.environ.get("CS_PROJECTS", "")
        scene_json_list = find_scene_jsons(cs_dir) if cs_dir else []

        # Determine starting index from task name (e.g., "CS-Projects-3" → 3)
        try:
            start_index = int(args_cli.task.split("-")[-1])
        except (ValueError, IndexError):
            start_index = 0

        # Create and start server
        scene_server = SceneServer(
            host=args_cli.scene_api_host,
            port=args_cli.scene_api_port,
        )
        scene_server.start()

        # Create manager (bridges API commands → Isaac Sim environment)
        scene_manager = SceneManager(
            env=env,
            env_cfg=env_cfg,
            scene_server=scene_server,
            scene_json_list=scene_json_list,
            start_index=start_index,
        )

        print(f"[Scene API] Server running on "
              f"http://{args_cli.scene_api_host}:{args_cli.scene_api_port}")
        print(f"[Scene API] Managing {len(scene_json_list)} scenes, "
              f"starting at index {start_index}")

    return scene_server, scene_manager


# ═══════════════════════════════════════════════════════════════════════════
# CHANGE 3: Poll for scene commands in the main loop
#           (add inside the `while simulation_app.is_running()` loop,
#            at the top of the non-replay branch, around line 481)
# ═══════════════════════════════════════════════════════════════════════════

def _main_loop_poll_example(scene_manager):
    """Add this call at the top of each main loop iteration."""
    if scene_manager is not None:
        scene_manager.poll()


# ═══════════════════════════════════════════════════════════════════════════
# CHANGE 4: Cleanup in the finally block (around line 600)
# ═══════════════════════════════════════════════════════════════════════════

def _cleanup_example(scene_server):
    """Add this to the finally block."""
    if scene_server is not None:
        try:
            scene_server.stop()
        except Exception as e:
            print(f"Failed to stop scene server: {e}")


# ═══════════════════════════════════════════════════════════════════════════
# FULL DIFF SUMMARY (line references to original sim_main.py)
# ═══════════════════════════════════════════════════════════════════════════
#
# --- a/sim_main.py
# +++ b/sim_main.py
#
# @@ line 83 (after --network-interface arg) @@
# +parser.add_argument("--scene-api-port", type=int, default=8200,
# +                    help="Port for Scene Control API server (0 to disable)")
# +parser.add_argument("--scene-api-host", type=str, default="0.0.0.0",
# +                    help="Host for Scene Control API server")
#
# @@ line 405 (after "create dds success") @@
# +    # --- Scene API server (for CS-Projects remote scene control) ---
# +    scene_server = None
# +    scene_manager = None
# +    if (args_cli.task.startswith("CS-Projects")
# +            and not args_cli.replay_data
# +            and args_cli.scene_api_port > 0):
# +        from scene_api import SceneServer, SceneManager
# +        from tasks.g1_tasks.cs_projects.cs_projects_scene_cfg import find_scene_jsons
# +        cs_dir = os.environ.get("CS_PROJECTS", "")
# +        scene_json_list = find_scene_jsons(cs_dir) if cs_dir else []
# +        try:
# +            start_index = int(args_cli.task.split("-")[-1])
# +        except (ValueError, IndexError):
# +            start_index = 0
# +        scene_server = SceneServer(host=args_cli.scene_api_host, port=args_cli.scene_api_port)
# +        scene_server.start()
# +        scene_manager = SceneManager(env, env_cfg, scene_server, scene_json_list, start_index)
# +        print(f"[Scene API] Server on http://{args_cli.scene_api_host}:{args_cli.scene_api_port}")
# +        print(f"[Scene API] {len(scene_json_list)} scenes, start={start_index}")
#
# @@ line 481 (inside main loop, non-replay branch, before env_state) @@
# +                    # Poll Scene API commands
# +                    if scene_manager is not None:
# +                        scene_manager.poll()
#
# @@ line 600 (in finally block, before env.close()) @@
# +        if scene_server is not None:
# +            try:
# +                scene_server.stop()
# +            except Exception as e:
# +                print(f"Failed to stop scene server: {e}")
