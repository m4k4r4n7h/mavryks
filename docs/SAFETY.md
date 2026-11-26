SAFETY

SAFETY MONITOR

The safety monitor runs on every state recomputation, which happens on every vision update and every operator command. It has authority to zero all motion commands before they are generated.

INPUTS

Obstacle set from live webcam detection and persistent static zones.
Congestion grid rho from the temporal occupancy filter.
Worker or robot positions used as the reference for clearance.
Commanded speed limit from operator configuration.
Operator emergency stop from the web console or the e key in the OpenCV window.
Vision quality score from blur, exposure, glare, shadow, and drift checks.
Person-robot proximity from YOLO person detections and localized robots.

RULES

Obstacle clearance: if the nearest obstacle is within MIN_CLEARANCE_CELLS of a worker, raise OBSTACLE_CLEARANCE and engage the safety stop.
Braking distance: braking distance is v squared over two times deceleration. If it reaches the available clearance, raise BRAKE_DISTANCE and engage the safety stop.
Congestion: recommended speed is scaled by 1 / (1 + 3 * rho_peak squared).
Vision quality: a score below 0.35 limits recommended speed to 0.25 m/s and raises VISION_QUALITY_LOW.
Proximity: a critical person-robot alert engages the safety stop. A warning alert is logged and surfaced in telemetry.
Operator emergency stop: always wins. It persists in SQLite and is restored on restart.

COMMAND ENFORCEMENT

If either the operator emergency stop or the computed safety stop is active, the speed passed to pure pursuit is zero. Pure pursuit rate limits acceleration and deceleration, and every command includes the resulting brake distance so an external controller can independently verify the stop envelope.

EVENTS

New alerts are recorded once when they first appear and cleared from the active alert set when they resolve. Events are stored in SQLite and shown in the web console event log.

OPERATOR PROCEDURE

Press e in the OpenCV window or click ESTOP in the web console to stop all motion immediately. Verify the status panel shows ACTIVE and the telemetry linear values are zero. Resolve the physical cause, then press e again or click CLEAR ESTOP. If vision quality is degraded, improve lighting or recapture the floor reference with r before resuming normal speed.
