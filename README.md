🤖 EV3 Autonomous Robot Navigation

A MicroPython-based autonomous navigation system implemented on the LEGO EV3 platform.
The robot is designed to operate in unknown environments, making real-time decisions for exploration, obstacle avoidance, and path optimization using a hybrid intelligent strategy.

🧠 Project Summary

This project simulates real-world autonomous robotics behavior in partially unknown or maze-like environments.
It integrates multiple navigation strategies to balance exploration efficiency and optimal pathfinding, mimicking decision-making in mobile robotic systems.

⚙️ Key Features
Fully autonomous navigation without human intervention
Real-time obstacle detection and avoidance
Hybrid decision-making system (exploration + optimal path planning)
Ability to return to the starting position
Modular and structured MicroPython architecture for EV3
Sensor-driven adaptive behavior
🧠 Algorithms & Strategy

The system combines multiple algorithms to handle different navigation scenarios:

A* Pathfinding Algorithm → computes the optimal path when the map is partially known
Random Mouse Algorithm → enables exploration of unknown areas
Right-Hand Rule → ensures reliable maze traversal and fallback navigation

➡️ These algorithms are integrated into a hybrid control strategy to improve robustness in dynamic environments.

🛠️ Hardware Platform
LEGO Mindstorms EV3 Brick
Ultrasonic Sensor → distance measurement & obstacle detection
Color Sensor → surface detection and navigation cues
Gyro Sensor → orientation and angle control
EV3 Motors → movement and steering control
📁 Project Structure
src/ → core MicroPython implementation for EV3
docs/report.pdf → detailed technical report (design, algorithms, results)
docs/presentation.pptx → project presentation slides
📊 Results & Outcomes
Demonstrated successful autonomous navigation in unknown environments
Improved navigation reliability using hybrid algorithm switching
Reduced unnecessary movement compared to single-algorithm approaches
Verified stable performance using EV3 hardware testing
🎯 Project Objectives
Implement real-world robotics algorithms in embedded systems
Develop autonomous decision-making using sensor inputs
Combine multiple navigation strategies into a single intelligent system
Bridge theoretical algorithms with practical robotics implementation
🚀 Technical Highlights
Event-driven robotic control system
Hybrid AI-inspired navigation logic
Efficient sensor fusion for decision-making
Scalable and modular code structure for future improvements
