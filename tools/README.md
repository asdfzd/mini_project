# Development and Diagnostic Tools

These scripts are intentionally outside the production ROS console entry points.

| Script | Classification | Purpose |
|---|---|---|
| `approach_object_nav.py` | Historical prototype | Earlier RGB-D → TF2 → Nav2 object approach, superseded by `mission_controller` |
| `nav_to_pose.py` | Navigation utility | Sends one fixed Nav2 pose and docks afterward |
| `depth_checker.py` | Diagnostic | Visualizes center depth and CameraInfo |
| `depth_checker_mouse_click.py` | Diagnostic | Reports depth at an OpenCV mouse click |
| `robot_yolo_viewer.py` | Diagnostic | Displays OAK-D YOLO detections |
| `yolo_test.py` | Diagnostic | Tests synchronized RGB/compressedDepth detection output |
| `webcam_test.py` | Diagnostic | Checks an arbitrary local webcam index |
| `webcam_yolo_test.py` | Diagnostic | Checks YOLO with a local webcam |

Build and source `rokey_pjt` before running ROS-aware tools so `rokey_pjt.model_paths` is importable. For example:

```bash
source install/setup.bash
python3 src/rokey_pjt/tools/robot_yolo_viewer.py
```

They are retained for hardware diagnosis and implementation history, but are not launched by `mission.launch.py`.
