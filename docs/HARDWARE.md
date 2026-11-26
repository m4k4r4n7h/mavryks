HARDWARE

CAMERA

The tracker opens a physical webcam index through cv2.VideoCapture with index 0 by default. Set CAMERA_SOURCE or pass --source to select a different index. Non-numeric sources are rejected and fall back to index 0, so video files and streams cannot be used as simulation inputs. Requested capture resolution is 640 by 480 with a one-frame buffer. X11 is required for the OpenCV window.

CALIBRATION SURFACE

Mount the camera looking straight down at a flat, well-lit floor or table. The four calibration corners map the physical surface onto a square grid whose default dimension is 12 by 12. A stable mount is important because calibration is persisted and reused after restart. If the camera moves, press c and recalibrate. If only lighting changed, press r to recapture the empty floor reference.

ARUCO MARKERS

vision/pose.py uses the DICT_4X4_50 dictionary. Print markers with IDs 0 through 49 and attach them flat to the top or base of each robot so the bottom edge is level with the floor. Marker foot point is the midpoint of the two lower corners and is projected through the calibration homography to a grid cell. Heading is derived from the vector between the lower-left and lower-right corners. A minimum marker area in pixels filters out distant false positives. Markers provide real robot localization and are not simulated.

OBJECT AND PERSON DETECTION

The default model is vision/yolov8n.pt. Person detections project their foot point to a grid cell and feed the worker tracker and proximity monitor. Other detected classes rasterize their lower portion into occupied cells. A morphological foreground pass over the calibrated floor mask adds objects that YOLO does not know, such as pallets or boxes. A temporal EMA filter suppresses single-frame noise before publishing obstacles and rho.

ROBOT INTEGRATION

Each snapshot contains a commands array. Every robot command has linear_mps, angular_radps, left_wheel_radps, right_wheel_radps, heading_target_rad, cross_track_m, lookahead_m, brake_distance_m, arrival, and battery_drain_pct. Drive defaults are a 0.38 m wheel base, 0.065 m wheel radius, 1.4 m/s maximum linear speed, 2.4 rad/s maximum angular speed, 0.9 m/s squared acceleration, and 1.3 m/s squared deceleration. These values live in backend/robotics.py and should be matched to the physical drive before deployment. Feed wheel commands to the motor controller and feed measured pose back through POST /api/vision/update as robots entries with cell, heading_deg, speed_mps, and battery_pct.

SAFETY HARDWARE

Wire a physical emergency stop to the operator workstation workflow. The software stop is authoritative for commands, but a hardware contactor should independently cut motor power. Verify brake_distance_m against measured stopping distance on the real floor before enabling full speed.
