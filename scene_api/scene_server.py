"""
Scene Control API Server (FastAPI)

Runs as a background thread inside unitree_sim_isaaclab's sim_main.py.
Provides REST endpoints for xr_teleoperate to control scene lifecycle:

  POST /scene/reset     — Reset current scene objects to original JSON positions
  POST /scene/next      — Load next scene (advance to next task)
  GET  /scene/status    — Current scene index, total scenes, and state

Architecture:
  The FastAPI server does NOT directly manipulate Isaac Sim objects (thread safety).
  Instead, it sets pending commands in a thread-safe SceneCommandQueue, which
  the main simulation loop polls each tick via check_pending_command().
"""

import threading
import time
import logging
from enum import Enum
from dataclasses import dataclass, field
from typing import Optional

import uvicorn
from fastapi import FastAPI
from pydantic import BaseModel

logger = logging.getLogger("scene_server")


# ---------------------------------------------------------------------------
# Shared command queue (main loop <-> FastAPI thread)
# ---------------------------------------------------------------------------

class CommandType(Enum):
    RESET_CURRENT = "reset_current"
    LOAD_NEXT = "load_next"


class CommandStatus(Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class SceneCommand:
    cmd: CommandType
    status: CommandStatus = CommandStatus.PENDING
    result_message: str = ""


class SceneCommandQueue:
    """Thread-safe single-slot command queue between FastAPI and sim main loop.

    Only one command can be pending at a time.  The API blocks until the main
    loop picks up and completes the command (or a timeout is reached).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._command: Optional[SceneCommand] = None

        # Scene state (written by main loop)
        self.task_num: int = 0
        self.current_scene_index: int = 0
        self.total_scenes: int = 0
        self.scene_json_path: str = ""
        self.all_completed: bool = False
        self.task_id: str = ""          # e.g. "CS-PROJECT-1-3"
        self.instructions: dict = {}    # e.g. {"en": ["Stand the fallen ...", ...]}

    def submit(self, cmd_type: CommandType, timeout: float = 30.0) -> SceneCommand:
        """Submit a command and wait for the main loop to complete it."""
        with self._lock:
            if self._command is not None and self._command.status == CommandStatus.PENDING:
                raise RuntimeError("A command is already pending")
            self._command = SceneCommand(cmd=cmd_type)
            self._event.clear()

        # Block until main loop signals completion
        completed = self._event.wait(timeout=timeout)
        with self._lock:
            cmd = self._command
            self._command = None
        if not completed:
            cmd.status = CommandStatus.FAILED
            cmd.result_message = "Timeout waiting for simulation to process command"
        return cmd

    def check_pending(self) -> Optional[SceneCommand]:
        """Called by main loop each tick. Returns pending command or None."""
        with self._lock:
            if self._command is not None and self._command.status == CommandStatus.PENDING:
                self._command.status = CommandStatus.IN_PROGRESS
                return self._command
        return None

    def complete(self, cmd: SceneCommand, success: bool, message: str = ""):
        """Called by main loop after executing the command."""
        with self._lock:
            cmd.status = CommandStatus.COMPLETED if success else CommandStatus.FAILED
            cmd.result_message = message
            self._event.set()


# ---------------------------------------------------------------------------
# FastAPI application
# ---------------------------------------------------------------------------

class SceneStatusResponse(BaseModel):
    task_num: int
    current_scene_index: int
    total_scenes: int
    scene_json_path: str
    all_completed: bool
    state: str  # "ready", "busy", "finished"
    task_id: str = ""
    instructions: dict = {}


class SceneActionResponse(BaseModel):
    success: bool
    message: str
    task_num: int
    current_scene_index: int
    all_completed: bool
    task_id: str = ""
    instructions: dict = {}


def create_app(cmd_queue: SceneCommandQueue) -> FastAPI:
    app = FastAPI(title="Scene Control API", version="1.0.0")

    def _action_resp(success, message):
        return SceneActionResponse(
            success=success,
            message=message,
            task_num=cmd_queue.task_num,
            current_scene_index=cmd_queue.current_scene_index,
            all_completed=cmd_queue.all_completed,
            task_id=cmd_queue.task_id,
            instructions=cmd_queue.instructions,
        )

    @app.get("/scene/status", response_model=SceneStatusResponse)
    def get_status():
        with cmd_queue._lock:
            busy = cmd_queue._command is not None
        return SceneStatusResponse(
            task_num=cmd_queue.task_num,
            current_scene_index=cmd_queue.current_scene_index,
            total_scenes=cmd_queue.total_scenes,
            scene_json_path=cmd_queue.scene_json_path,
            all_completed=cmd_queue.all_completed,
            state="finished" if cmd_queue.all_completed else ("busy" if busy else "ready"),
            task_id=cmd_queue.task_id,
            instructions=cmd_queue.instructions,
        )

    @app.post("/scene/reset", response_model=SceneActionResponse)
    def reset_scene():
        """Reset current scene objects to their original JSON positions."""
        try:
            cmd = cmd_queue.submit(CommandType.RESET_CURRENT, timeout=30.0)
            return _action_resp(
                cmd.status == CommandStatus.COMPLETED,
                cmd.result_message or "Scene reset completed",
            )
        except RuntimeError as e:
            return _action_resp(False, str(e))

    @app.post("/scene/next", response_model=SceneActionResponse)
    def next_scene():
        """Advance to the next scene. Returns all_completed=True if no more scenes."""
        if cmd_queue.all_completed:
            return _action_resp(True, "All tasks completed")
        try:
            cmd = cmd_queue.submit(CommandType.LOAD_NEXT, timeout=60.0)
            return _action_resp(
                cmd.status == CommandStatus.COMPLETED,
                cmd.result_message or "Next scene loaded",
            )
        except RuntimeError as e:
            return _action_resp(False, str(e))

    return app


# ---------------------------------------------------------------------------
# Server lifecycle
# ---------------------------------------------------------------------------

class SceneServer:
    """Manages the FastAPI server running in a background daemon thread."""

    def __init__(self, host: str = "0.0.0.0", port: int = 8200):
        self.host = host
        self.port = port
        self.cmd_queue = SceneCommandQueue()
        self._thread: Optional[threading.Thread] = None
        self._uvicorn_server: Optional[uvicorn.Server] = None

    def start(self):
        """Start the API server in a background thread."""
        app = create_app(self.cmd_queue)
        config = uvicorn.Config(
            app, host=self.host, port=self.port,
            log_level="warning",
            access_log=False,
        )
        self._uvicorn_server = uvicorn.Server(config)
        self._thread = threading.Thread(
            target=self._uvicorn_server.run,
            name="scene-api-server",
            daemon=True,
        )
        self._thread.start()
        logger.info(f"Scene API server started on http://{self.host}:{self.port}")

    def stop(self):
        """Gracefully shut down the server."""
        if self._uvicorn_server is not None:
            self._uvicorn_server.should_exit = True
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        logger.info("Scene API server stopped")
