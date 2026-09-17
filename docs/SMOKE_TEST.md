# TurtleBot 4 Runtime Smoke Test

이 체크리스트는 Ubuntu 22.04, ROS 2 Humble, TurtleBot 4와 OAK-D가 연결된 실제 환경에서 수행합니다. Windows 정리 환경에서는 실행하지 않았습니다.

| # | Check | Expected result | First inspection point on failure |
|---:|---|---|---|
| 1 | Package build | `colcon build --packages-select rokey_pjt` succeeds | `package.xml`, `setup.py`, sourced TurtleBot workspace |
| 2 | Mission launch | Both production nodes start without traceback | `launch/mission.launch.py` |
| 3 | Webcam index 2 | `/dev/video2` opens and frames are read | device connection/permission, `webcam_trigger.py` |
| 4 | Webcam YOLO | class ID 1 is detected | model classes, lighting, model path |
| 5 | Mission trigger | `/robot1/start_mission` publishes `True` | transient-local QoS and topic namespace |
| 6 | Scan point navigation | Robot reaches the authored scan coordinate | Nav2/localization/map and fixed scan point |
| 7 | Rotation scan | Robot rotates at approximately 0.15 rad/s | `/robot1/cmd_vel`, `SCANNING` state |
| 8 | OAK-D RGB | compressed RGB messages arrive | RGB topic and robot namespace |
| 9 | OAK-D Depth | compressedDepth decodes to valid values | transport format and depth topic |
| 10 | CameraInfo | fx/fy/ppx/ppy are populated | CameraInfo topic |
| 11 | YOLO target | class ID 1 passes confidence/5-frame confirmation | model labels and threshold |
| 12 | Camera → Map TF | object camera point transforms to map | optical frame ID and TF tree |
| 13 | Nav2 target approach | Goal is in front of object and avoids obstacles | object/robot map positions and Nav2 |
| 14 | Moving-target replan | Replan occurs only after 2s and 0.25m movement | goal update logs |
| 15 | TRACKING transition | Nav2 completion/near range enters TRACKING | state logs |
| 16 | `linear.x` | Positive only when target is farther than desired | depth/error and 0.12m/s clamp |
| 17 | `angular.z` | Corrects horizontal offset within ±0.15 rad/s | bbox center and sign convention |
| 18 | Target lost | After 30 missed frames robot stops and scans again | lost counter and cancellation logs |
| 19 | Safe stop | Ctrl-C and invalid depth leave zero Twist | shutdown and depth-jump paths |

Capture the node logs, active topics, TF tree and first traceback for any failure. Do not tune gains or thresholds until the behavior above has been reproduced with the original values.
