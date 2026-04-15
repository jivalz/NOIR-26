# 🚀 Team Drishti – ROS 2 Workspace  

International Rover Challenge (IRC) 2026  

---

## 📌 Overview  
This repository contains the ROS 2 workspace used by Team Drishti for the International Rover Challenge 2026.  

It includes:  
- Cone detection using OpenVINO  
- ROS 2 communication (C++ and Python)  
- Launch system for running different rover tasks  

---

## 📁 Workspace Structure  

src/  
├── openvino_cone_detection/  
├── cpp_pubsub/  
├── py_pubsub/  
├── my_launch/  


---

## 🔹 Packages  

### openvino_cone_detection  
- Handles cone detection  
- Uses OpenVINO for faster inference  
- Publishes detection results  

### cpp_pubsub  
- C++ ROS 2 nodes  

### py_pubsub  
- Python ROS 2 nodes  

### my_launch  
- Contains launch files  
- Used to run multiple nodes together  

---

## ⚙️ Environment & Versions  
- ROS 2 Version: Humble  
- Operating System: Ubuntu 22.04  

---

## 📦 Dependencies  
This workspace depends on:  
- geographic_msgs  
- geodesy  

### ✅ Install All Dependencies  

cd ~/your_workspace  
rosdep install --from-paths src --ignore-src -r -y  


---

## 🔧 Sensors  

### LiDAR  
Driver: https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2  

### GPS  
Driver: [nmea_navsat_driver](https://github.com/ros-drivers/nmea_navsat_driver/tree/ros2)

---

## 🚀 Launch Instructions  

### 🔵 IDMO Task  

- ros2 launch my_launch joy2.launch.py
- ros2 run cpp_pubsub 2ps4
- ros2 launch my_launch idmo.launch.py


### 🟢 ABEX Task  

- ros2 launch my_launch joy2.launch.py
- ros2 run cpp_pubsub ps4_abex
- ros2 launch my_launch abex.launch.py


### 🔴 RADO Task  

#### Recon  

- ros2 launch my_launch joy2.launch.py
- ros2 run cpp_pubsub 2ps4
- ros2 launch my_launch rado_sensors.launch.py
- ros2 launch my_launch rado_recon.launch.py


#### Autonomous Delivery  

- ros2 launch my_launch rado_sensors.launch.py
- ros2 launch my_launch rado_auto.launch.py


---

## 🤖 Arduino (micro-ROS)  
- Microcontroller code for ESP32  
- Use micro_ros_arduino library built for ROS 2 Humble only  

Repository Link: [Ino Files](https://github.com/prakhar-29/Noir_Ino) 

---

## 🦾 Robotic Arm (URDF + MoveIt)

* Contains the robotic arm description (URDF) and MoveIt configuration
* Supports motion planning and execution using MoveIt

Repository Link: [Robotic Arm](https://github.com/prakhar-29/Robotic_arm_Noir)

## 👥 Contributors  
- [Jival Dhingra](https://github.com/jivalz)  
- [Prakhar Dwivedi](https://github.com/prakhar-29)  
- [Pranav Singh](https://github.com/Pranav-710)  

---

## 👥 Team Drishti  
Built for the International Rover Challenge (IRC) 2026   
