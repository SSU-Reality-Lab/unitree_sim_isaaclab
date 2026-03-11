# Scene API Integration Guide

## Overview

Communication pipeline between xr_teleoperate (Client) and unitree_sim_isaaclab (Server)
using a FastAPI REST API for scene lifecycle control.

```
┌─────────────────────────┐         HTTP/REST          ┌────────────────────────────┐
│   xr_teleoperate        │  ───────────────────────►  │   unitree_sim_isaaclab     │
│   (Client)              │  ◄───────────────────────  │   (Server)                 │
│                         │                             │                            │
│  SceneClient            │   POST /scene/reset         │  SceneServer (FastAPI)     │
│   .reset_scene()  ──────┼──────────────────────────► │   └─► SceneCommandQueue    │
│   .next_scene()   ──────┼──────────────────────────► │       └─► SceneManager     │
│   .get_status()   ──────┼──────────────────────────► │           └─► env.reset()  │
│                         │                             │                            │
│  teleop_hand_and_arm.py │                             │  sim_main.py main loop     │
│   save → next_scene()   │                             │   poll() each tick         │
│   discard → reset()     │                             │                            │
└─────────────────────────┘                             └────────────────────────────┘
```

## Files Created

### Server (unitree_sim_isaaclab)
- `scene_api/__init__.py` — Package exports
- `scene_api/scene_server.py` — FastAPI server + thread-safe command queue
- `scene_api/scene_manager.py` — Scene lifecycle (reset, load next)

### Client (xr_teleoperate)
- `teleop/utils/scene_client.py` — HTTP client for scene API

## Integration Steps

### 1. Server: sim_main.py changes

See `sim_main_integration.py` for the exact diff.

### 2. Client: teleop_hand_and_arm.py changes

See `teleop_integration.py` for the exact diff.

### 3. Dependencies

Server: `pip install fastapi uvicorn`
Client: `pip install requests`

## API Endpoints

| Method | Endpoint        | Description                              |
|--------|-----------------|------------------------------------------|
| GET    | /scene/status   | Current scene index, total, state        |
| POST   | /scene/reset    | Reset objects to JSON default positions   |
| POST   | /scene/next     | Load next scene (or return all_completed) |

## Sequence Diagrams

### Save Episode → Load Next Scene
```
xr_teleoperate                    unitree_sim_isaaclab
     │                                    │
     │  recorder.save_episode()           │
     │  ──────────────────────►           │
     │                                    │
     │  POST /scene/next                  │
     │  ─────────────────────────────────►│
     │                                    │ remove old prims
     │                                    │ spawn new prims
     │                                    │ env.sim.reset()
     │  {success, scene_index}            │
     │  ◄─────────────────────────────────│
     │                                    │
     │  → READY state                     │
```

### Discard Episode → Reset Scene
```
xr_teleoperate                    unitree_sim_isaaclab
     │                                    │
     │  recorder.discard_episode()        │
     │  ──────────────────────►           │
     │                                    │
     │  POST /scene/reset                 │
     │  ─────────────────────────────────►│
     │                                    │ reset_scene_to_default()
     │  {success, scene_index}            │
     │  ◄─────────────────────────────────│
     │                                    │
     │  → READY state (same scene)        │
```

### All Tasks Completed
```
xr_teleoperate                    unitree_sim_isaaclab
     │                                    │
     │  POST /scene/next                  │
     │  ─────────────────────────────────►│
     │                                    │ no more scenes
     │  {success, all_completed=True}     │
     │  ◄─────────────────────────────────│
     │                                    │
     │  → notify user, shutdown/idle      │
```
