API REFERENCE

All HTTP endpoints are served by the backend on port 8000. The Vite frontend proxies /api and /ws to the backend. Every command response that changes state returns the full snapshot unless noted otherwise.

GET /api/health
Returns status, uptime_sec, grid_size, ws_clients, vision_online, vision_age_sec, estop, pending_tasks, and safety_estop.

GET /api/state
Returns the full snapshot: grid, rho, obstacles, static and dynamic obstacle lists, workers, robots, targets, path, steps, instructions, cost, distance, eta, fleet routes, commands, tasks, safety, quality, proximity, vision, metrics, diagnostics, events, and updated_at.

GET /api/metrics
Returns solve timing, vision FPS, command latency, counters, obstacle counts, and WebSocket client count.

GET /api/history?limit=50
Returns the most recent routing history entries.

POST /api/vision/update
Body: obstacles, rho, workers, targets, robots, quality, proximity, diagonal, fps, estop. All fields are optional. Sets dynamic obstacles and congestion, applies robot poses and safety inputs, then recomputes routes and broadcasts. Called by the tracker every frame.

POST /api/target
Body: x and y, or targets as a list of pairs. Optional append, workers, diagonal, optimize_order. Sets or appends waypoints and recomputes.

POST /api/zones/static
Body: obstacles list and mode one of replace, add, remove, toggle. Zones are persisted.

DELETE /api/zones/static
Clears all static zones.

POST /api/config
Body with any of diagonal, cost_mode, congestion_weight, turn_penalty, clearance_weight, clearance_radius, cell_size_m, worker_speed_mps, optimize_order, snap_start_if_blocked, decel_mps2. Persisted.

POST /api/estop
Body: active boolean. Stops all commanded motion when true. Persisted.

POST /api/speed
Body: speed_mps between 0 and 3. Persisted.

POST /api/reset
Clears dynamic obstacles, targets, robots, rho, and tasks. Static zones and config remain.

GET /api/tasks
Returns tasks and stats.

POST /api/tasks
Body: kind, x and y or target, priority, robot_id, payload_kg, label. Returns task and state. A pending task is dispatched automatically when no other task is active.

POST /api/tasks/{id}/start
Marks a task active and sets it as the current target.

POST /api/tasks/{id}/complete
Marks done, increments the done counter, and dispatches the next pending task.

POST /api/tasks/{id}/fail
Body: optional reason query parameter. Marks failed and dispatches the next pending task.

POST /api/tasks/{id}/cancel
Marks cancelled.

DELETE /api/tasks/{id}
Removes a task permanently.

GET /api/events?limit=100
Returns persisted events newest first.

DELETE /api/events
Clears the event log.

GET /api/system
Returns version, database path, diagnostics, task stats, route count, and command count.

WS /ws/live
Sends a full snapshot on connect and on every state change. Clients may send JSON commands: ping, get_state, set_target, toggle_static, add_task, complete_task, cancel_task, estop, speed, config, clear_targets, clear_zones. A ping receives a pong message with a timestamp.

COST REFERENCE

Exact mode: w(u,v) = d(u,v) * (1 + 4.5 * rho_v^2) + 1000000 * I_blocked(v). The blocked term is applied by excluding blocked cells in A*, while the helper exposes it explicitly for verification. Industrial mode adds turn penalty and obstacle-clearance penalty on top of the same base.
