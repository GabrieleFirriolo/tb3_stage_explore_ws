# 🧭 TurtleBot3 Stage Explore – ROS 2 Workspace

This project integrates several ROS 2 packages to simulate autonomous exploration of a (simulated) TurtleBot3 in a Stage-based 2D-world.

## 📦 Included Packages (as submodules)

- [`m-explore-ros2`](https://github.com/GabrieleFirriolo/m-explore-ros2): autonomous exploration logic (forked and customized)
- [`ros2_gmapping`](https://github.com/GabrieleFirriolo/ros2_gmapping): SLAM via GMapping (forked and customized)
- [`stage_ros2`](https://github.com/tuw-robotics/stage_ros2): Stage ROS2 bridge
- [`Stage`](https://github.com/tuw-robotics/Stage): Stage simulator
- `tb3_stage_explore`: coordination tools, launch interface, test environments

> 📄 Please refer to each package's README for specific build instructions and notes.

---

## ⚙️ Requirements

- ROS 2 Jazzy (or compatible)
- Python 3
- `rosdep` for resolving dependencies

Install system dependencies:
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
rosdep init
rosdep update
```

---

## 🧩 Clone the project

```bash
git clone --recurse-submodules https://github.com/GabrieleFirriolo/tb3_stage_explore_ws.git
cd tb3_stage_explore_ws
rosdep install --from-paths src --ignore-src -r -y
```

---

## 🛠️ Build the workspace

Some packages require staged builds. Refer to their READMEs for full details.

Two-step build process, required due to `gmapper` not being able to build without `openslam_gmapping` sourced:
```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install --packages-select openslam_gmapping
source install/setup.bash

colcon build --symlink-install
source install/setup.bash
```

---

## 🚀 Running the Simulation

```bash
cd src/tb3_stage_explore/python/
python3 singlerun.py ../worlds/E34-2.world
```
- The Stage interface will open.
- A new folder in `runs/outputs/<map>`  will be created to store run data.

You can visualize the simulation with RViz2 in another terminal:

```bash
rviz2 -d src/tb3_stage_explore/rviz/explore.rviz
```

---

## 🐳 Docker

This project supports containerized simulation using Docker.

### 🔧 Build Docker Image

First, build the Docker image using the provided `Dockerfile` (in the project root):

```bash
docker build -t ros2jazzy:explore .
```

### 🚀 Run Multiple Simulations in Parallel

Use the script `spawnContainers.py` to launch multiple isolated simulations using Docker containers.

Each container is launched with:

* a unique `ROS_DOMAIN_ID`
* a filtered `.world` map based on image size
* a mounted output folder for run results

```bash
python3 spawnContainers.py
```

You’ll be prompted to:

* select the subdirectory of the `worlds/` folder (or just `.` to use the default)
* set how many parallel containers (workers) to launch
* all selected `.world` files are can be filtered by their map image size.

> 🗂 Output logs and results will be saved under:

```
output/
```

---

## 📜 License

This project includes third-party packages. Please respect the original licenses.