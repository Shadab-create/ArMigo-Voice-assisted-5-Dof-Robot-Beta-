# ArMigo: Voice-Assisted 5-DoF Robot Arm (Beta)

ArMigo is a voice-assisted, 5 Degrees of Freedom (5-DoF) robotic arm development project. This repository houses the complete design, physics simulation environment, and hardware control software for the initial beta build.

---

## 🛠️ Project Structure

The project files are organized into the following main directories:

* **`ArMigo(stl)/`** — Original 3D CAD modeling assets (`.stl` files) for the structural links including the Base, Base Cover, Joint 1, Joint 2, Joint 3, and Joint 4.
* **`Armigo Pcb/`** — Complete hardware electronic designs built using KiCad (`.kicad_pcb`, `.kicad_sch`), including design rules and auto-backups.
* **`Inverse kinematics full along with manual control/`** — Core algorithm software.
  * `main.py` — Central script running the robot operation.
  * `inverse_kinematics.py` — Mathematical models mapping coordinate targets to joint angles.
  * `robot_ik_control.py` / `manual.py` — Dynamic control loops for automated IK and manual overriding.
  * `motor_control.py` / `esp32.c++` — Embedded firmware scripts to bridge software commands to physical actuators.
  * `camera_detection.py` — Computer vision functions and calibration tools using ArUco markers.
  * `my_robot/` — **PyBullet simulation workspace** containing the unified `.urdf` model definition, structural assets, and configurations.

---

## 🚀 Key Features

* **PyBullet Physics Simulation**: Features a virtual environment utilizing PyBullet to parse the robot's `.urdf` and validate motion algorithms safely before deploying to hardware.
* **5-DoF Robotic Kinematics**: Embedded geometric inverse kinematics models for precise spatial trajectory tracking inside the simulator.
* **Voice Assistance Integration**: Developed framework mapping voice inputs to high-level motion primitives (Beta phase functionality).
* **Integrated Hardware Design**: Custom control electronics designed directly via KiCad alongside simulation-ready URDF models.
* **Vision-Guided Calibration**: Support for camera-based coordinate space mapping utilizing ArUco fiducial tracking markers.

---

## ⚙️ Dependencies & Prerequisites

To run the PyBullet simulation environment and core Python controllers locally, install the required packages:

```bash
pip install pybullet numpy opencv-python opencv-contrib-python
```

---

## 📝 Development Status
This repository functions as an **active development playground**. Large simulation recording files (`.mp4`) are explicitly excluded from tracking via `.gitignore` configurations to adhere to hosting footprint optimization constraints.
