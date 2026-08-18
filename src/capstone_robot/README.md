# capstone_robot

A simulated Ackermann-steered rover in Gazebo (see the companion
`Hunter_ROS2` description/gazebo packages for the vehicle model itself) with
camera-based obstacle avoidance, built up in two layers:

1. A **hand-coded, rule-based avoider** that fuses three physical depth
   cameras (front-left, front-center, front-right) with YOLOv4-tiny person
   detection on the RGB camera.
2. An **imitation-learned alternative** -- a small NumPy neural net trained on
   logged `(sensor state, driver intent) -> command` pairs from driving
   against the rule-based system, as a from-scratch example of behavior
   cloning without any ML framework dependency.

## Package layout

```
capstone_robot/
├── src/                        ROS2 executable nodes + standalone training/eval scripts
├── models/                     YOLOv4-tiny weights, trained MLP weights, test person model
├── urdf/                       (placeholder, unused -- see note below)
├── DATA_SUMMARY.md             History and stats of every training-data file collected
├── MODEL_UPGRADE_PLAN.md       When/how to move past the current small MLP if it hits its limits
├── package.xml / CMakeLists.txt
```

`urdf/robot.urdf` is an empty placeholder left over from the initial
`ros2 pkg create` scaffold -- the actual robot description lives in the
separate `Hunter_ROS2/hunter_description` package, not here.

## Dependencies

Declared in `package.xml`: `rclpy`, `sensor_msgs`, `geometry_msgs`,
`cv_bridge`, `message_filters`, `ament_index_python`, `gazebo_msgs`,
`python3-opencv`, `python3-numpy`. All standard ROS2 Humble + apt packages --
**no `pip` packages required anywhere in this package.** The neural net
(`train_avoidance_model.py`) and person detector (via `cv2.dnn`) are both
built on plain NumPy/OpenCV specifically so nothing here needs PyTorch,
scikit-learn, or `ultralytics`.

Also requires (outside this package, provided by the wider workspace):
`Hunter_ROS2` (robot description + Gazebo world/spawn launch files) and
Gazebo Classic 11.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select capstone_robot
source install/setup.bash
```

## Nodes

| Node | What it does |
|---|---|
| `wasd_teleop.py` | Keyboard teleop (`w/a/s/d`, space=stop, q=quit) → `/teleop_cmd_vel`. Pressing `w`/`s` also re-centers steering to straight. |
| `obstacle_detector.py` | Earliest, simplest avoider -- single depth camera split into left/center/right thirds. |
| `fused_obstacle_avoider.py` | Production rule-based avoider -- 3 *real* depth cameras + YOLO person-stop safety override. Speed scales by center-zone distance (`CLEAR`≥2.0m→100%, `NEARBY`≥1.0m→50%, `CLOSE`≥0.5m→20%, `CRITICAL`<0.5m→0%); auto-steers toward whichever side is clearer when blocked; a person within 1.5m blocks forward motion (reverse still allowed, to avoid a Ackermann-steering deadlock). |
| `person_stop_controller.py` | Lighter variant -- YOLO person-stop only, no zone-based steering. |
| `learned_avoider.py` | Drop-in replacement for `fused_obstacle_avoider.py` -- same inputs/safety overrides, but the driving decision comes from `models/avoidance_mlp.npz` instead of if/else rules. |
| `object_distance_viewer.py` | Debug/visualization: YOLO detections + distance overlaid on the RGB feed in an OpenCV window. |
| `data_logger.py` | Logs `(sensor state, teleop intent, actual command, controller_source)` rows to CSV for training data. |
| `spawn_person.py` | Spawns test person models in Gazebo -- front+left pair, or `--scatter N` for a ring of N people around the robot. |
| `train_avoidance_model.py` | Standalone (no ROS import needed) -- trains the NumPy MLP on logged CSVs. |
| `evaluate_model.py` | Standalone -- confusion matrix + MAE/RMSE of the trained model against logged data. |

Only run **one** of `obstacle_detector.py` / `fused_obstacle_avoider.py` /
`person_stop_controller.py` / `learned_avoider.py` at a time -- each publishes
to the same `/ackermann_steering_controller/reference_unstamped` topic.

## Running it

```bash
# terminal 1: world
ros2 launch hunter_gazebo start_world.launch.py
# terminal 2: robot
ros2 launch hunter_gazebo spawn_robot_ros2.launch.xml
# terminal 3: drive
ros2 run capstone_robot wasd_teleop.py
# terminal 4: pick ONE avoider
ros2 run capstone_robot fused_obstacle_avoider.py
# optional: spawn test people
ros2 run capstone_robot spawn_person.py --scatter 12
```

## Retraining the learned model

```bash
ros2 run capstone_robot data_logger.py --output ~/capstone_avoidance_data/session1.csv
# ...drive around with fused_obstacle_avoider.py active...

python3 src/train_avoidance_model.py \
  --data ~/capstone_avoidance_data/*.csv \
  --source fused_obstacle_avoider   # exclude any learned_avoider-generated rows
python3 src/evaluate_model.py --data ~/capstone_avoidance_data/*.csv
```
