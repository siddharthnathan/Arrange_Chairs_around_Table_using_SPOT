# SPOT Autonomous Chair Arrangement using Fiducial Markers

Autonomous mobile manipulation system using the Boston Dynamics SPOT robot to detect, localize, grasp, and arrange chairs around a table using fiducial-marker-based perception.

This project integrates:
- Boston Dynamics SPOT SDK
- Intel RealSense RGB-D cameras
- ArUco/AprilTag pose estimation
- Multi-camera localization
- Coordinate-frame transformations
- Autonomous waypoint navigation
- Vision-guided robotic manipulation

The system demonstrates a complete perception-to-action pipeline for structured indoor mobile manipulation tasks.

---

# Project Overview

The robot autonomously:
1. Detects fiducial markers attached to chairs and walls
2. Localizes itself relative to a global origin marker
3. Computes chair poses in the global frame
4. Identifies unarranged chairs
5. Navigates to the target chair
6. Grasps the chair using SPOT’s manipulator arm
7. Moves the chair to a predefined goal configuration
8. Repeats until all chairs are arranged

The project explores the question:

> Are fiducial markers sufficient for performing autonomous mobile manipulation tasks in structured indoor environments?

---

# System Architecture

## Perception Layer
- Dual Intel RealSense D435i cameras
- ArUco/AprilTag marker detection
- Pose estimation using OpenCV
- Multi-camera spatial registration

## Localization Layer
- Global origin marker
- Wall reference markers
- SPOT body-frame localization
- Homogeneous transformation matrices

## Control Layer
- SPOT SDK locomotion commands
- Waypoint-based navigation
- Manipulator arm control
- Gripper actuation

## Integration Layer
- Real-time pose updates
- Chair state validation
- Autonomous task sequencing
- Closed-loop manipulation pipeline

---

# Hardware Setup

## Robot Platform
- Boston Dynamics SPOT
- 6-DOF manipulator arm
- Parallel jaw gripper

## Sensors
- 2 × Intel RealSense D435i cameras
- SPOT onboard perception cameras

## Fiducial Markers
- ArUco / AprilTag markers
- 20 cm × 20 cm markers
- Mounted on:
  - chairs
  - walls
  - reference surfaces

---

# Repository Structure

```bash
.
├── main.py
├── arrange_chairs_around_table.py
├── pose_estimation.py
├── spot_robot_commands.py
├── read_video_stream.py
├── utils.py
├── saved_objects_poses.json
├── aruco_markers.pdf
├── Camera_Calibration/
│   ├── Camera_1/
│   └── Camera_2/
├── chair_aruco.3mf
├── wall_aruco.3mf
└── README.md
```

---

# Core Modules

## `main.py`
Master execution pipeline.

Responsibilities:
- Initialize SPOT
- Configure cameras
- Update object poses
- Detect unarranged chairs
- Execute navigation and manipulation

---

## `pose_estimation.py`
Perception and localization module.

Features:
- ArUco detection
- 6-DoF pose estimation
- Camera-to-world transformations
- SPOT localization

Key Functions:
- `estimate_poses_of_aruco_tags()`
- `localize_spot_wrt_origin()`
- `update_poses_of_chairs()`

---

## `spot_robot_commands.py`
SPOT SDK interface layer.

Features:
- Robot initialization
- Locomotion
- Arm manipulation
- Gripper control
- Fiducial detection

Key Functions:
- `move_robot_to_location()`
- `move_SPOT_behind_chair()`
- `grasp_chair_using_SPOT()`
- `move_arm_to_grasp_pose()`

---

## `read_video_stream.py`
RealSense camera interface.

Features:
- Camera streaming
- Calibration loading
- Frame acquisition
- Image visualization

---

## `utils.py`
Transformation and helper utilities.

Features:
- Homogeneous transformations
- Rotation conversions
- Pose utilities
- Object management
- Waypoint logic

---

# Coordinate Frames

The system uses multiple coordinate frames:

| Frame | Description |
|---|---|
| Origin Frame | Global world reference |
| Camera Frame | RealSense camera coordinates |
| SPOT Body Frame | Robot body reference |
| Gripper Frame | Manipulator end-effector |
| Chair Marker Frame | Fiducial marker on chair |

---

# Workflow Pipeline

## 1. Camera Initialization
- Configure RealSense pipelines
- Load calibration parameters

## 2. Environment Mapping
- Detect wall fiducials
- Compute global coordinate system

## 3. Chair Pose Estimation
- Detect chair markers
- Compute chair poses in world frame

## 4. SPOT Localization
- Detect visible wall markers
- Compute SPOT pose relative to origin

## 5. Chair Selection
- Identify nearest unarranged chair

## 6. Navigation
- Move SPOT through predefined waypoints

## 7. Manipulation
- Move behind chair
- Detect marker using gripper camera
- Execute grasp
- Transport chair
- Release chair

---

# Installation

## Requirements

### Hardware
- Boston Dynamics SPOT
- Intel RealSense D435i cameras

### Software
- Python 3.10+
- OpenCV
- NumPy
- SciPy
- pyrealsense2
- Boston Dynamics SDK

---

## Install Dependencies

```bash
pip install numpy scipy opencv-python pyrealsense2
```

Install Boston Dynamics SDK:

```bash
pip install bosdyn-client bosdyn-mission bosdyn-api
```

---

# Camera Calibration

Run calibration before execution:

```bash
python calibration.py
```

Calibration files will be stored in:

```bash
Camera_Calibration/
```

---

# Running the System

## Step 1 — Arrange Chairs Manually
Place chairs in desired goal configuration.

## Step 2 — Save Goal Poses
Capture goal-state images.

## Step 3 — Randomize Chairs
Move chairs to arbitrary positions.

## Step 4 — Execute System

```bash
python main.py
```

---

# Experimental Results

| Metric | Value |
|---|---|
| Total Trials | 10 |
| Chairs Arranged | 21/28 |
| Success Rate | 75% |
| Average Position Error | 1.6 cm |
| Average Orientation Error | 5° |

---

# Known Limitations

## Fiducial Dependency
Requires markers attached to objects.

## Occlusion Sensitivity
Markers may become invisible during navigation.

## Calibration Sensitivity
Accurate camera calibration is critical.

## Static Environment Assumption
Limited robustness in dynamic environments.

## No Collision-Aware Planning
Potential collisions during placement.

---

# Future Work

- Markerless object detection
- Learning-based grasp planning
- SLAM integration
- Dynamic obstacle avoidance
- ROS2 migration
- Multi-robot coordination
- Force-feedback manipulation
- Real-time collision checking

---

# Research Contribution

This project demonstrates:
- Vision-guided mobile manipulation
- Hybrid external + onboard perception
- Fiducial-based autonomous manipulation
- Real-world deployment on SPOT
- Closed-loop chair arrangement

---

# Citation

If you use this work, please cite:

```bibtex
@mastersthesis{siddharth2026spot,
  title={Are Fiducial Markers All We Need to Perform Mobile Manipulation Tasks?},
  author={Senthilnathan, Siddharth},
  school={University of Minnesota},
  year={2026}
}
```

---

# Acknowledgements

- Boston Dynamics
- University of Minnesota
- RPM Lab
- Intel RealSense SDK
- OpenCV ArUco Library

---

# License

This project is intended for academic and research purposes.

