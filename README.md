# autonomous_maze_solving_robot

This project demonstrates an autonomous TurtleBot3 robot navigating through a maze environment using SLAM (via Cartographer) and the Nav2 stack inside a Gazebo Classic simulation. The robot builds a map of the environment in real-time and uses global/local planning to reach target locations while avoiding obstacles.

---

## 📌 Features

- Spawn robot and maze environment using ROS 2 services
- Build 2D occupancy grid map using Cartographer SLAM
- Perform real-time localization and autonomous navigation using Nav2
- Visualize mapping, planning, and movement in RViz2

---

## 🛠️ System Requirements

- Ubuntu 20.04
- ROS 2 (Foxy or Humble)
- Gazebo Classic
- TurtleBot3 packages
- Cartographer ROS
- Nav2 stack
- Python 3

---

## 🧱 Folder Structure

bash
autonomous_tb3/
├── config/
│   └── turtlebot3_cartographer.lua
├── launch/
│   └── mapping_launch.py
├── worlds/
│   └── custom_maze_world.sdf
├── scripts/
│   └── spawn_entity.py
├── rviz/
│   └── mapping_config.rviz
├── package.xml
├── setup.py
└── README.md


---

## 🚀 Installation & Setup

1. Clone this repo into your ROS 2 workspace:

bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/autonomous_tb3.git
cd ..
colcon build --packages-select autonomous_tb3
source install/setup.bash


2. Set TurtleBot3 model (e.g., burger or waffle):

bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc


---

## 🔧 Launch Instructions

1. Start Gazebo with your maze world:

bash
gazebo --verbose ~/ros2_ws/src/autonomous_tb3/worlds/custom_maze_world.sdf


2. In a new terminal, spawn the TurtleBot3 inside the world:

bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_tb3 spawn_entity.py ~/ros2_ws/src/autonomous_tb3/models/turtlebot3_burger/model.sdf tb3 0.0 0.0


3. Launch SLAM mapping (Cartographer):

bash
ros2 launch autonomous_tb3 mapping_launch.py


4. Optionally, visualize in RViz2:

bash
rviz2 -d ~/ros2_ws/src/autonomous_tb3/rviz/mapping_config.rviz


5. After mapping is complete, save the map:

bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_maze_map


6. Launch Nav2 for autonomous navigation (once mapping is done):

bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/ros2_ws/maps/my_maze_map.yaml
