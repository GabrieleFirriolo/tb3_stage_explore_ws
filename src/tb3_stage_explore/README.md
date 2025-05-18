# 🧭 tb3_stage_explore – Coordination Package for TurtleBot3 Exploration

This package provides a coordination layer that launches and connects all necessary ROS2 nodes for autonomous exploration in the Stage simulator using TurtleBot3. It integrates navigation, SLAM, exploration logic, and simulation components.

## 🧩 Role in the System

`tb3_stage_explore` is the **core launcher and integrator** of the system. It ensures that:

- `GMapping (SLAM)`, `explore_lite`, and `Stage` simulation nodes run together
- Navigation stack (`nav2`) is configured properly
- All components use consistent parameters and map references

This package is a ROS 2 port and adaptation of the ROS 1 setup from [prina404/ros-exploration-config](https://github.com/prina404/ros-exploration-config).

---

## 🚀 Main Launch File

The central launch file is:

> launch/turtlebot_stage_explore.launch.py


This file starts:
- Stage simulator with the selected `.world` file
- SLAM (`ros2_gmapping`)
- Exploration behavior (`m-explore-ros2`)
- Navigation2 stack

---

## ⚙️ Parameters & Configuration

### 🔧 `params/`
This folder contains:
- `nav2_params.yaml`: Configuration for the Navigation2 stack

### 🌍 `worlds/`
Contains `.world` files compatible with Stage. You can define multiple environments for simulation testing.

### 🖼 `worlds/img/`
Each `.world` should have a corresponding `.png` image map. These are used for filtering or batch simulation (e.g., in `spawnContainers.py`).

### 📜 `worlds/worldcfg.ini`
Optional configuration file for Stage-related settings or world metadata (format to be specified).

---

## 🔗 Node Communication Overview

Here's how nodes interact:

- `Stage` publishes `/base_scan`, `/odom`,`/ground_truth`, and `/tf`
- `ros2_gmapping` consumes scan + odometry and publishes the `/map`
- `nav2` uses the map and robot localization to plan and move
- `explore_lite` requests goals for unexplored areas
- The robot moves based on `cmd_vel` commands from `nav2`

## 🚀 singlerun.py

In  `python/singlerun.py` this script:

automates simulation execution with the following features:

* **Launches** the full exploration stack using a specified `.world` file.
* **Saves intermediate maps** every 60 seconds during exploration (`nav2_map_server`), stored in:

  ```
  runs/outputs/<map>/runX/Maps/
  ```
* **Saves a final map** automatically when exploration ends.
* **Records a rosbag** (`ros2 bag record`) with relevant topics like `/base_scan` and `/ground_truth`.

* **Terminates the simulation automatically** upon receiving a signal on `/explore/done`, a custom topic published by the modified `m-explore-ros2` node or after waiting 200s after no new valid frontiers are received.

---