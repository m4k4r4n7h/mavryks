ARCHITECTURE

DATA FLOW

Physical webcam frame
  -> vision/tracker.py calibration, YOLO detection, ArUco localization, occupancy filtering
  -> POST /api/vision/update with obstacles, rho, workers, robots, quality, proximity, fps, estop
  -> backend/main.py state manager applies updates under an asyncio lock
  -> backend/router.py recomputes single or fleet routes synchronously
  -> backend/safety.py approves or blocks motion and limits speed
  -> backend/robotics.py converts paths to wheel commands
  -> WS /ws/live broadcast to every operator console
  -> frontend React panels render the live grid, telemetry, tasks, and events

BACKEND MODULES

main.py FastAPI application, state manager, all HTTP and WebSocket endpoints, task dispatch, command generation.
router.py pure A* from scratch with heapq, exact and industrial edge weights, waypoint ordering, Space-Time A* fleet routing, step and instruction generation.
store.py SQLite persistence for key-value config, static zones, tasks, and events with a threading lock.
tasks.py warehouse task queue with pick, place, dock, inspect, charge kinds and a full lifecycle.
robotics.py differential-drive kinematics, pure pursuit, wheel command output, braking distance, battery drain, fleet utilization.
safety.py clearance, braking, congestion, vision quality, proximity, and emergency stop enforcement.
diagnostics.py uptime, solve latency, vision FPS, command latency, and counters.
dashboard.html classic fallback operator console served at the backend root.

VISION MODULES

tracker.py live capture, calibration UI, detection, temporal occupancy, keyboard controls, sync worker, and clean shutdown including the close-window fix.
pose.py ArUco marker detection, foot-point projection to grid coordinates, and marker heading.
quality.py blur, exposure, glare, shadow, and lighting drift scoring.
proximity.py person-robot distance classification.

FRONTEND MODULES

App.tsx WebSocket lifecycle, initial state load, and all command handlers.
api.ts small POST and DELETE helpers.
types.ts shared snapshot types.
components/GridPanel.tsx dynamic SVG grid, multi-route overlay, targets, robots, and click handling.
components/StatusPanel.tsx operator status table.
components/ControlPanel.tsx emergency stop, speed, movement, cost mode, optimization, and reset controls.
components/TaskPanel.tsx task queue and lifecycle.
components/RobotPanel.tsx robot poses, wheel commands, and proximity.
components/InstructionPanel.tsx compressed directions and exact A* steps.
components/EventPanel.tsx persisted event log.

CONCURRENCY

The backend state manager uses a single asyncio lock so a vision update, operator command, and WebSocket broadcast never interleave half-applied state. The vision sync worker runs on a daemon thread with its own lock and a bounded command queue. HTTP calls are short and time out, so a stopped backend never blocks frame processing indefinitely.

NO SIMULATION

There is no code path that generates synthetic obstacles, seeded targets, replayed frames, or a mock camera. The tracker opens a physical webcam index only. If the camera is unavailable it retries the same webcam index. If the backend is unavailable vision processing continues locally and the status banner reports the outage.
