📦 Object Fetching via Waypoint Navigation (ROS2 + TurtleBot3)
📖 Overview

This project implements autonomous object fetching using a TurtleBot3 robot in a simulated Gazebo environment.

The robot:

Navigates using predefined waypoints
Detects a pickup location (marker node)
Confirms object identity
Moves to a designated drop-off zone

All navigation is handled autonomously using the ROS2 Navigation Stack (Nav2).

🚀 Features
Autonomous navigation using Nav2
Waypoint-based movement
Localization using AMCL
Simulation in Gazebo (headless)
Visualization using RViz
Docker-based reproducible setup
🛠️ Tech Stack
ROS2 Humble
Gazebo
RViz2
TurtleBot3
Docker
📂 Project Structure
Object-Fetching-via-Waypoint-Navigation/
├── Dockerfile
├── README.md
├── src/
├── build/
├── install/
├── log/
⚙️ Setup Instructions
1. Clone the Repository
git clone <your-repo-url>
cd Object-Fetching-via-Waypoint-Navigation
2. Build Docker Image
docker build -t irrp-humble .
3. Enable GUI Access (Host)
xhost +local:docker
4. Run Docker Container
docker run -it --rm \
  --name irrp_ros \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/home/ros/ws \
  irrp-humble
🧠 Running the Project
Terminal 1 — Launch Simulation
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py gui:=false
Terminal 2 — Launch Navigation (Nav2)
docker exec -it irrp_ros bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True
Terminal 3 — Launch RViz
docker exec -it irrp_ros bash
rviz2
📍 RViz Configuration

Inside RViz:

Set Fixed Frame:

map
Add Displays:
RobotModel
TF
LaserScan (/scan)
Map
Global Costmap
Local Costmap
Path (/plan)
🧭 Localization (Important)

Before navigation, initialize the robot pose:

Click 2D Pose Estimate
Click near the robot position
Drag to set orientation
🎯 Navigation

To move the robot:

Click Nav2 Goal
Select a target point on the map

The robot will:

Plan a path
Avoid obstacles
Navigate autonomously
🧪 Verification

Check active ROS topics:

ros2 topic list

Expected topics include:

/scan
/odom
/cmd_vel
/tf
❗ Common Issues
Robot not visible in RViz
Use 2D Pose Estimate to initialize localization
RViz not opening

Run:

xhost +local:docker
Robot not moving
Ensure Nav2 is running

Check /cmd_vel:

ros2 topic echo /cmd_vel
Multiple containers issue
Always use a single container (irrp_ros)
Use docker exec for additional terminals
👥 Team Roles
Navigation & Mapping
Nav2 setup
Localization
Path planning
Perception
Marker detection
Object identification
Task Management
Waypoint scheduling
Pickup and drop logic
🔮 Future Work
Automated waypoint sequencing
Marker-based pickup logic
Priority-based task scheduling
Multi-object handling
📌 Notes
Always use a single Docker container
Edit code on host, run inside Docker
Use RViz for all navigation visualization
📜 License

This project is for academic use.
