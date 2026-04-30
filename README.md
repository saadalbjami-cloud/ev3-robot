# 🤖 EV3 Autonomous Robot Navigation

A MicroPython-based autonomous navigation system built for the LEGO EV3 platform. The robot operates in unknown environments and makes real-time decisions for exploration, obstacle avoidance, and path optimization using a hybrid intelligent approach.

## 🧠 Project Overview
This project simulates autonomous robotic behavior in unknown or maze-like environments. It integrates multiple navigation strategies to balance exploration efficiency and optimal pathfinding, enabling robust decision-making in dynamic conditions.

## ⚙️ Features
- Fully autonomous navigation without human intervention  
- Real-time obstacle detection and avoidance  
- Hybrid navigation system (exploration + optimal pathfinding)  
- Return-to-start capability  
- Modular and structured MicroPython code  
- Sensor-driven decision making  

## 🧠 Algorithms Used
- A* Pathfinding Algorithm → optimal path computation with partial map knowledge  
- Random Mouse Algorithm → exploration of unknown areas  
- Right-Hand Rule → maze traversal fallback strategy  

These algorithms are combined into a hybrid system for improved robustness and adaptability.

## 🛠️ Hardware Platform
- LEGO Mindstorms EV3 Brick  
- Ultrasonic Sensor (distance measurement & obstacle detection)  
- Color Sensor (surface and line detection)  
- Gyro Sensor (orientation and angle control)  
- EV3 Motors (movement and steering)  

## 📁 Project Structure
- src/ → MicroPython source code for EV3 robot  
- docs/report.pdf → technical project report  
- docs/presentation.pptx → presentation slides  

## 📊 Results
- Successful autonomous navigation in unknown environments  
- Improved efficiency using hybrid algorithm switching  
- Reduced unnecessary movement compared to single-strategy methods  
- Stable real-world performance on EV3 hardware  

## 🎯 Objectives
- Implement real-world robotics algorithms in embedded systems  
- Develop autonomous decision-making using sensor inputs  
- Combine multiple navigation strategies into one system  
- Bridge theory with practical robotics implementation  

## 🚀 Technical Highlights
- Event-driven control system  
- Hybrid AI-inspired navigation logic  
- Sensor fusion for decision making  
- Modular and scalable architecture
