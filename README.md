# MAVRYKS

**Live warehouse vision and dynamic routing, driven by a real webcam.**

MAVRYKS watches a physical floor through a webcam, turns what it sees into a 12 x 12 occupancy grid, and routes robots around people, objects, and congested cells. Every cell, cost, pose, and path comes from the live camera and the live backend state. There is no simulation mode, no replay, no demo data, and no fallback grid.

## Built by

- Makaranth MA
- Rikshikesh
- Sagar Rajesh
- Sai krishna
- Vibhin Rathod
- Yadhu krishna

## What you get

- Four-click camera calibration that maps pixels to floor cells
- YOLO object detection plus empty-floor background subtraction
- A temporal filter that removes camera flicker before routing
- Congestion-aware A* routing written from scratch with `heapq`
- Multi-waypoint ordering and fleet routing with collision reservations
- ArUco marker localization for real robot poses and headings
- Pure-pursuit path tracking converted into differential-drive wheel speeds
- A safety monitor that can slow or stop every robot
- A classic web console, a live WebSocket feed, and SQLite persistence

## Requirements

- Linux with a physical webcam and an X11 display
- Python 3
- Node.js with npm

## Start it

From the repository root:

```bash
bash setup.sh
bash run.sh
```

`run.sh` installs anything missing, starts the backend, frontend, and tracker, waits for the API, and prints the addresses. Stop everything with:

```bash
bash stop.sh
```

If you prefer one terminal that stops cleanly with `CTRL-C`:

```bash
python3 run.py
```

Other root helpers:

```bash
bash validate.sh
source activate.sh
```

| Command | What it does |
| --- | --- |
| `bash setup.sh` | Creates `venv` and installs Python and Node packages |
| `bash run.sh` | Starts all three services in the background |
| `python3 run.py` | Starts all three services in the foreground, `CTRL-C` to stop |
| `bash stop.sh` | Stops all services |
| `bash validate.sh` | Runs the clean scan, Python compile, and frontend build |
| `source activate.sh` | Activates `venv` and loads `.env` if present |

Open:

- Console: `http://localhost:5173`
- Health: `http://localhost:8000/api/health`
- Live state: `http://localhost:8000/api/state`
- Classic dashboard: `http://localhost:8000/`

Another device on the LAN can use `http://<this-machine-lan-ip>:5173`. The services bind to `0.0.0.0`, but no public tunnel or reverse proxy is configured.

## Calibrate the camera

1. Point the webcam at an empty floor or table with the grid area clear.
2. In the `MAVRYKS Live` window, click four corners in this order: Top-Left, Top-Right, Bottom-Right, Bottom-Left.
3. Press `SPACE` to lock the perspective transform.
4. Keep the floor empty and press `SPACE` again when asked for the clean reference frame.
5. Put real objects in the grid. Red cells appear and routes recompute immediately.
6. Press `c` any time to clear calibration and start again.

Calibration files are saved in `vision/` and loaded automatically after a restart.

## Controls

In the OpenCV window:

| Key | Action |
| --- | --- |
| `SPACE` | Lock calibration or capture the empty-floor reference |
| `c` | Clear calibration |
| `r` | Recapture the empty-floor reference |
| `d` | Toggle YOLO debug boxes |
| `p` | Pause vision processing |
| `e` | Toggle the emergency stop |
| `n` | Complete the next open task |
| `q` or `ESC` | Quit and release the camera |

Closing the window also quits the tracker cleanly and releases the camera.

In the web console:

- Left click a cell to set a destination.
- Shift click to append another waypoint.
- Right click to toggle a persistent static obstacle.
- The operator panel controls emergency stop, speed limit, diagonal movement, cost mode, waypoint optimization, reset, and task lifecycle.
- The task panel queues pick, place, dock, inspect, and charge jobs.
- The telemetry panel shows live robot poses, wheel commands, braking distance, battery drain, fleet utilization, and person proximity.

## The maths, simply explained

### 1. Camera pixels to floor cells

The webcam sees a tilted floor, so a square on the floor looks like a trapezoid in the image. Four clicked corners fix that distortion. MAVRYKS solves for a 3 x 3 homography `H` such that:

```text
[x_floor, y_floor, 1]^T  ~  H * [x_pixel, y_pixel, 1]^T
```

Each point is divided by its third coordinate after the multiplication, which is what the `~` means. `inv_H` does the reverse. It maps the grid back to the image so the console can draw masks and cell outlines in the right place. A degenerate quad is rejected by checking the contour area and the determinant of `H`.

The 12 x 12 grid has cells indexed `0..11`. The physical centre of cell `(x, y)` is `((x + 0.5) * cell_size_m, (y + 0.5) * cell_size_m)`.

### 2. Finding objects

YOLO returns bounding boxes. For a person, MAVRYKS projects the foot midpoint `(box_center_x, box_bottom_y)` through `H` to get a worker cell. For other objects, it projects the lower 45% of the box and marks every covered cell as occupied.

A separate check catches objects YOLO may miss. The tracker keeps a blurred grayscale image of the empty floor. Each frame it computes:

```text
diff = abs(blurred_current_gray - empty_floor_gray)
```

Pixels above threshold 20 are cleaned with morphological open and close, masked to the floor polygon built with `inv_H`, and rasterized into occupied cells. Quiet pixels slowly update the reference with `ref = (1 - 0.004) * ref + 0.004 * current` so lighting changes do not accumulate forever.

### 3. Stable obstacles and congestion

A single noisy frame should not block a route. Each cell keeps an exponential average:

```text
E_t = 0.45 * observed_t + 0.55 * E_(t-1)
```

A free cell becomes blocked when `E >= 0.55`. A blocked cell clears only when `E < 0.25`. That gap is hysteresis, and it stops cells from flickering between states.

Congestion `rho` is a soft version of the blocked map. It is the blocked state blurred with a Gaussian kernel, scaled by 1.6, and clipped to `0..1`. Blocked cells are forced to at least `0.95`. Robot and person cells are cleared so robots never block themselves.

### 4. The route cost

This is the heart of the router. For a step from cell `u` into cell `v`:

```text
w(u,v) = d(u,v) * (1 + 4.5 * rho_v^2) + 1000000 * I_blocked(v)
```

- `d(u,v)` is `1` for a straight step and `sqrt(2)` for a diagonal step.
- `rho_v` is congestion in the destination cell, between `0` and `1`.
- `I_blocked(v)` is `1` if the cell is blocked, otherwise `0`.

The congestion term is quadratic. At `rho = 0.5` the step costs 2.125 times its distance; at `rho = 1` it costs 5.5 times. That makes robots prefer a slightly longer clear aisle over a short crowded one. The blocked penalty of one million makes blocked cells effectively impossible without special handling. Diagonal moves never cut through the corner of two blocked cells.

### 5. A* search

A* keeps a priority queue of cells ordered by:

```text
f(v) = g(v) + h(v)
```

`g(v)` is the cheapest cost found so far from the start. `h(v)` is an optimistic guess of the remaining cost, so the search stays focused. With diagonal movement the guess is the octile distance:

```text
h = max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)
```

With four-direction movement it is the Manhattan distance `dx + dy`. A `came_from` map reconstructs the path once the goal is popped. The default `exact` cost mode uses the formula above. The optional `industrial` mode adds a turn penalty `penalty * (1 - cos(theta)) / 2` and a clearance penalty based on distance to obstacles.

### 6. Several destinations

When an operator queues multiple waypoints, MAVRYKS first computes the A* cost of every pair. For up to 11 waypoints it finds the best order exactly with Held-Karp dynamic programming. Above 11 it uses nearest neighbour followed by 2-Opt swaps.

The estimated time is:

```text
effective_speed = speed / (1 + 0.6 * average_rho)
eta = distance_meters / effective_speed + 0.25 * turns
```

### 7. Several robots

Fleet routing uses Space-Time A*. A search state is `(cell, time_step)`, and a robot may also wait in place. Earlier robots reserve `(time, cell)` pairs and `(time, from_cell, to_cell)` swaps. Later robots cannot enter a reserved cell or swap through a reserved edge, which prevents head-on collisions. A wait step costs `1.05`.

### 8. From a path to wheel speeds

Pure pursuit picks a target point about `0.55 m` ahead on the path. It computes heading error and cross-track error, then limits speed by curvature so the robot does not tip:

```text
curvature = min(4, 2 * cross_track / lookahead^2)
speed <= sqrt(1.1 / curvature)
angular = 2.2 * heading_error
```

Speed and angular velocity are rate limited with `0.9 m/s^2` acceleration and `1.3 m/s^2` deceleration. For a differential-drive robot with wheel base `b = 0.38 m` and wheel radius `r = 0.065 m`:

```text
left_wheel  = (linear - angular * b / 2) / r
right_wheel = (linear + angular * b / 2) / r
```

### 9. Safety first

The safety monitor runs before any command is sent. Braking distance is:

```text
brake_distance = speed^2 / (2 * deceleration)
```

The clearance speed limit is `sqrt((clearance_m - 0.12) * 2 * deceleration)` and the congestion speed limit is `1 / (1 + 3 * rho^2)`. An emergency stop is raised when:

- an obstacle is within 1.1 cells,
- braking distance reaches the available clearance,
- a person is within 1.2 cells of a robot,
- or the operator presses the emergency stop.

Low frame quality caps speed at `0.25 m/s`. A person between 1.2 and 2.5 cells raises a warning.

### 10. Localization and frame quality

Real ArUco 4x4 markers give robot poses. The midpoint of the marker's bottom edge is projected through `H` into a grid cell, and the heading comes from `atan2` of that bottom edge.

Frame quality multiplies four scores: blur from Laplacian variance, exposure, glare, and drift from the empty-floor reference. A shadow alert is added when more than 25% of pixels are very dark. Bad quality never invents data; it only slows or stops the robots.

## Project layout

| Path | Role |
| --- | --- |
| `backend/main.py` | FastAPI app, state, endpoints, WebSocket feed |
| `backend/router.py` | A*, waypoint ordering, Space-Time fleet A* |
| `backend/robotics.py` | Pure pursuit, differential drive, battery model |
| `backend/safety.py` | Clearance, braking, congestion, and proximity rules |
| `backend/store.py` | SQLite persistence |
| `backend/tasks.py` | Task lifecycle |
| `backend/diagnostics.py` | Metrics and history |
| `vision/tracker.py` | Webcam capture, calibration, detection, main loop |
| `vision/pose.py` | ArUco localization |
| `vision/quality.py` | Frame quality assessment |
| `vision/proximity.py` | Person-robot proximity |
| `frontend/src/` | React web console |
| `scripts/` | Setup, runtime, and validation helpers |
| `docs/` | Architecture, operations, API, safety, hardware notes |

## API

Read:

```text
GET /api/health
GET /api/state
GET /api/metrics
GET /api/history
GET /api/tasks
GET /api/events
GET /api/system
GET /
```

Write:

```text
POST /api/vision/update
POST /api/target
POST /api/zones/static
DELETE /api/zones/static
POST /api/config
POST /api/reset
POST /api/tasks
POST /api/tasks/{id}/start
POST /api/tasks/{id}/complete
POST /api/tasks/{id}/fail
POST /api/tasks/{id}/cancel
DELETE /api/tasks/{id}
POST /api/estop
POST /api/speed
DELETE /api/events
```

Live socket: `WS /ws/live`. It sends a full snapshot on connect and accepts `ping`, `get_state`, `set_target`, `toggle_static`, `add_task`, `complete_task`, `cancel_task`, `estop`, `speed`, `config`, `clear_targets`, and `clear_zones`.

## Configuration

Copy `.env.example` to `.env`, edit it, then run `source activate.sh`.

| Variable | Default | Meaning |
| --- | --- | --- |
| `LOG_LEVEL` | `INFO` | Python log level |
| `GRID_SIZE` | `12` | Grid dimension |
| `VISION_STALE_SEC` | `5.0` | Age before vision is marked offline |
| `MAVRYKS_DB` | `backend/mavryks.db` | SQLite file |
| `DECEL_MPS2` | `1.3` | Braking deceleration |
| `MIN_CLEARANCE_CELLS` | `1.1` | Obstacle stop distance |
| `CAMERA_SOURCE` | `0` | Physical webcam index |
| `BACKEND_URL` | `http://127.0.0.1:8000` | Backend base URL |
| `MODEL_PATH` | `yolov8n.pt` | YOLO weights |
| `YOLO_DEVICE` | unset | Optional inference device |

## Persistence

`backend/mavryks.db` stores configuration, static zones, tasks, and events. Emergency stop state and speed limit survive restarts. Calibration lives in `vision/H.npy`, `vision/inv_H.npy`, `vision/floor_ref.npy`, and `vision/calibration_meta.json`.

## Check the project

```bash
bash validate.sh
```

It runs a zero-comment source scan, Python byte compilation, and the frontend TypeScript and Vite build.

## More detail

- `docs/ARCHITECTURE.md` - module map and data flow
- `docs/OPERATIONS.md` - startup, checks, recovery, shutdown
- `docs/API.md` - request and response reference
- `docs/SAFETY.md` - safety rules and emergency stop behavior
- `docs/HARDWARE.md` - camera, markers, and robot integration notes

## License

MIT. See `LICENSE`.
