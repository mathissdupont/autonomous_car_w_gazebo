# Autonomous Vehicle Simulation

This repository contains the full stack implementation of an autonomous vehicle simulation platform developed using **ROS 2**, **Gazebo**, and custom AI-based decision modules. The system simulates a self-driving car navigating through structured roads using sensor data and real-time control logic.

---

## 🚗 Project Overview

This simulation system is designed for research and demonstration purposes. It emulates:

- Lane detection via onboard cameras
- Obstacle and curb detection using Lidar
- Localization with GPS & IMU fusion
- Decision-making with Python-based logic
- Vehicle control through differential drive

---

## 📦 Tech Stack

| Component           | Technology                         |
|---------------------|-------------------------------------|
| Simulation Engine   | Ignition Gazebo                     |
| Robot Middleware    | ROS 2 (Humble)                      |
| Language(s)         | Python, C++                         |
| Sensors             | Lidar, IMU, GPS, Camera             |
| Control Plugin      | DiffDrive Plugin (SDF/URDF)         |
| Visualization       | RViz, Gazebo GUI                    |

---

## 📁 Folder Structure

```
/autonomous_vehicle_sim
├── launch/                # ROS 2 launch files
├── models/                # SDF/URDF models with sensors
├── src/
│   ├── decision_maker/    # Python-based logic
│   ├── sensor_cleaner/    # Raw sensor preprocessing
│   └── localization/      # GPS/IMU fusion logic
├── config/                # Config files for params
├── maps/                  # Custom road environments
└── README.md              # This file
```

---

## 🧠 Core Functionalities

- ✅ Right Lane Tracking with yellow line detection
- ✅ Curb Avoidance using Lidar proximity
- ✅ T-Junction Handling with camera vision logic
- ✅ Stable Heading through IMU and GPS fusion
- ✅ ROS 2 TF Tree maintaining `odom → base_link` chain

---

## 🧪 How to Run

1. Clone the repo:
   ```bash
   git clone https://github.com/mathissdupont/autonomous_car_w_gazebo.git
   cd autonomous-vehicle-sim
   ```

2. Source your ROS 2 workspace:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. Build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ```

4. Launch the simulation:
   ```bash
   ros2 launch launch/simulation.launch.py
   ```

---

## 📄 License

MIT License © 2025

---

[![Watch on YouTube](https://img.youtube.com/vi/37UanjivitE?si=sMFmPIjP3bvUrCoU/0.jpg)](https://www.youtube.com/watch?v=37UanjivitE?si=sMFmPIjP3bvUrCoU)


## 🌐 Contact

For inquiries, reach out via:  
📧 
