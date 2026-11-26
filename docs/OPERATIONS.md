OPERATIONS

STARTUP

Run bash setup.sh once before first startup. From the repository root use bash run.sh for a detached stack or python3 run.py for an attached stack that stops on CTRL-C. Three-terminal startup is described in README.md. PID and log files are written under run/. Stop with bash stop.sh.

HEALTH CHECKS

curl http://localhost:8000/api/health
curl http://localhost:8000/api/system
curl http://localhost:8000/api/state
curl http://localhost:5173

The health response reports backend uptime, grid size, WebSocket client count, vision freshness, vision age, emergency stop state, pending tasks, and safety stop state. Vision is considered online when the last update arrived within VISION_STALE_SEC, which defaults to 5 seconds.

FIRST RUN

1. Start the backend and frontend before the tracker so the first vision update reaches a live API.
2. Calibrate the webcam using the four corner clicks and SPACE prompts.
3. Confirm the status banner shows ONLINE with a latency value.
4. Click a grid cell in the web console or the OpenCV top-down panel and confirm a route appears.
5. Queue a task from the web console and confirm the task panel shows assigned or active status.

SHUTDOWN

Press q or ESC in the OpenCV window, or close the window with the window manager. The tracker releases the camera, stops the sync thread, and exits. Press CTRL-C in each terminal for the frontend and backend. Alternatively run bash scripts/stop.sh.

CLOSE-WINDOW BEHAVIOR

The loop checks cv2.getWindowProperty before processing each frame and immediately after cv2.imshow. Once the operator closes the window the tracker sets its running flag to false, breaks the loop, and never calls cv2.imshow again. This prevents the window from reopening on the next frame.

RECOVERY

If the webcam stream drops, the tracker releases and reopens the same camera index with exponential backoff capped at 5 seconds. If the backend restarts, vision updates resume automatically on the next successful POST. Calibration files in vision/ are loaded at startup, so a tracker restart does not require recalibration unless the camera moved. Press r to recapture the floor reference after lighting changes. Press c to fully recalibrate after camera movement.

PERSISTENCE RESET

Remove backend/mavryks.db to clear persisted config, static zones, tasks, and events. Remove vision/H.npy, vision/inv_H.npy, vision/floor_ref.npy, and vision/calibration_meta.json to clear calibration.

VALIDATION

bash scripts/validate.sh runs Python byte compilation, the frontend TypeScript build, and the clean-source scan. Run it after any code change.

CONFIGURATION

Environment variables: GRID_SIZE, VISION_STALE_SEC, MAVRYKS_DB, LOG_LEVEL, CAMERA_SOURCE, BACKEND_URL, MODEL_PATH, YOLO_DEVICE, DECEL_MPS2, MIN_CLEARANCE_CELLS. Runtime configuration can also be changed through POST /api/config and is persisted.
