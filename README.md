# 📦 Object Fetching via Waypoint Navigation (ROS2 + TurtleBot3)

## 📖 Overview

This project implements **autonomous object fetching** using a TurtleBot3 in a simulated Gazebo environment.

The robot:

* Navigates using **waypoints**
* Detects a **pickup marker**
* Confirms object identity
* Moves to a **drop-off location**

All navigation is fully autonomous using **ROS2 Navigation Stack (Nav2)**.

---

## 🚀 Features

* Autonomous navigation using **Nav2**
* Waypoint-based movement
* SLAM / localization using **AMCL**
* Simulation using **Gazebo**
* Visualization using **RViz**
* Modular design for:

  * Navigation
  * Perception
  * Task scheduling (bonus)

---

## 🛠️ Tech Stack

* ROS2 (Humble)
* Gazebo
* RViz2
* TurtleBot3
* Docker

---

## 📂 Project Structure

```bash
Object-Fetching-via-Waypoint-Navigation/
├── Dockerfile
├── README.md
├── src/
├── build/
├── install/
├── log/
```

---

## ⚙️ Setup Instructions

### 1️⃣ Clone the Repository

```bash
git clone <your-repo-url>
cd Object-Fetching-via-Waypoint-Navigation
```

---

### 2️⃣ Build Docker Image

```bash
docker build -t irrp-humble .
```

---

### 3️⃣ Enable GUI Access (Host)

```bash
xhost +local:docker
```

---

### 4️⃣ Run Docker Container

```bash
docker run -it --rm \
  --name irrp_ros \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/home/ros/ws \
  irrp-humble
```

---

## 🧠 Running the Project

### Terminal 1 — Launch Gazebo (Simulation)

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
```

---

### Terminal 2 — Launch Navigation Stack (Nav2)

```bash
docker exec -it irrp_ros bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
```

---

### Terminal 3 — Launch RViz

```bash
docker exec -it irrp_ros bash
rviz2
```

---

## 📍 RViz Configuration

Inside RViz:

1. Set:

   ```
   Fixed Frame → map
   ```

2. Add Displays:

   * RobotModel
   * TF
   * LaserScan
   * Map
   * Global Costmap
   * Local Costmap
   * Path

---

## 🧭 Localization (Important Step)

Before navigation, initialize robot pose:

1. Click **2D Pose Estimate**
2. Click on robot position
3. Drag to set orientation

---

## 🎯 Navigation

To move the robot:

1. Click **Nav2 Goal**
2. Select a target location

The robot will:

* Plan a path
* Avoid obstacles
* Move autonomously

---

## 🧪 Verification

Check active topics:

```bash
ros2 topic list
```

Expected topics:

* `/scan`
* `/odom`
* `/cmd_vel`
* `/tf`

---

## 🧩 Future Work

* Waypoint sequencing
* Marker detection for pickup
* Task scheduling (priority-based)
* Multi-object handling

---

## 👥 Team Roles

* **Navigation & Mapping Lead**

  * Nav2 setup
  * Localization
  * Path planning

* **Perception Lead**

  * Marker detection
  * Object identification

* **Task Manager**

  * Waypoint scheduling
  * Pickup/drop logic

---

## ❗ Common Issues

### Robot not visible in RViz

* Use **2D Pose Estimate**

### RViz GUI not opening

* Ensure:

  ```bash
  xhost +local:docker
  ```

### Multiple containers issue

* Always use **same container (`irrp_ros`)**

---

## 📌 Notes

* Use **one Docker container only**
* Do not run multiple containers for ROS nodes
* Always use `docker exec` for new terminals

---

## 📜 License

This project is for academic use.

---
* add diagrams
* or write the **waypoint navigation code section** 🔥
