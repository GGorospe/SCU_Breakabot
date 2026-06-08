<p align="center">
  <img src="graphics/logo.png" alt="Break-a-Bot Logo" width="600"/>
</p>

<h3 align="center">Demonstrating Diagnostics for Mobile Robotics</h3>

<p align="center">
  <a href="https://www.scu.edu/engineering/labs--research/labs/robotic-systems-lab/">Santa Clara University · Robotic Systems Lab</a>
</p>

---

## Overview

**Break-a-Bot** is a fault-tolerant mobile robotics platform developed at Santa Clara University's Robotic Systems Lab under the direction of **Dr. Chris Kitts**. The platform is purpose-built to demonstrate and validate diagnostic techniques for wheeled mobile robots — intentionally introducing, detecting, and characterizing hardware faults in a controlled environment.

The robot features omnidirectional (holonomic) movement via omni-wheels, redundant subsystems, and experimental breakpoints that allow researchers to simulate real-world failure conditions and observe system response.

---

## Students:

**George Gorospe** masters students worked on the Break-a-bot from 2022 to 2026.

---

## Features

- **Holonomic drive** — Omnidirectional motion via omni-wheel configuration
-  **Real-time fault injection** — Hardware breakpoints for inducing controlled component failures
-  **Battery monitoring** — Continuous voltage telemetry and low-battery detection
-  **Current sensing** — Per-motor current monitoring for anomaly detection
-  ** RoboteQ motor controlers** — A dedicated motor controller for each of the three motors
-  **Relay switching** — Configurable relay board for component isolation and redundancy testing
-  **Automated telemetry** — Structured data collection for post-run analysis

---

## Repository Structure

The repository is structured as a ROS2 workspace and contains the following packages.

| Package | Role |
|---|---|
| `breakabot_hw` | Hardware interface nodes (sensors, actuators) |
| `breakabot_core` | Control, planning, and analysis nodes |
| `breakabot_bringup` | Launch files and configuration |

More information can be found in the docs folder which includes the following:

[Architecture](docs/HARDWARE.md)

[Hardware Description](docs/HARDWARE.md)

[Quickstart Guide](docs/quickstart.md)

---

## Hardware

| Component | Description |
|---|---|
| Drive System | 3-wheel omnidirectional platform |
| Motor Control | PWM shield |
| Power Monitoring | Analog battery monitor |
| Current Sensing | Per-channel current sensors via analog mux |
| Fault Switching | Relay board with configurable breakpoints |
| Compute | Raspberry Pi (or compatible SBC) |


---

## Research Background

Break-a-Bot is built on a long-running SCU research program exploring various diagnostic methods in the context of for anomaly detection in robotic systems. The break-a-bot is a purpose built testbed for developing and demonstraing diagnostic routines. 

This platform serves as a live testbed for:
- Fault detection and isolation algorithms
- Diagnostic model validation
- Redundancy and fault-tolerance strategy evaluation

> Kitts, C. "Managing space system anomalies using first principles reasoning." *IEEE Robotics & Automation Magazine*, 13.4 (2006): 39–50.

---

## Affiliation

**[Robotic Systems Laboratory](https://www.scu.edu/engineering/labs--research/labs/robotic-systems-lab/)**  
Department of Mechanical Engineering  
Santa Clara University  
Advisor: Dr. Chris Kitts

---

## License

This project is developed for academic research purposes at Santa Clara University. Please contact the lab for licensing and usage inquiries.
