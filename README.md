# RobotraceSim 🏎️🤖

RobotraceSim is an interactive **line-follower robot simulator**, written in **Python + PySide6**.
It provides visual editors for **tracks** and **robots**, plus a **simulation engine** that runs external controllers in **Python** or **C**.

---

## ✨ Features

* **Track Editor**

  * Create race tracks with straight and arc segments.
  * Configure area size, tape width, and origin.
  * Add start/finish lines and curvature markers.
  * Import/export tracks in JSON format.

* **Robot Editor**

  * Define robot envelope (up to 250×250 mm, following official rules).
  * Place wheels and sensors with grid snapping support.
  * Origin adjustment relative to robot envelope or sensors.
  * Import/export robot models as JSON.

* **Simulator**

  * Differential drive kinematics with simplified first-order motor dynamics.
  * Sensor modeling with tape coverage estimation and marker detection.
  * Support for external controllers:

    * **Python** (`.py`)
    * **C** (`.so`, `.dll`, `.dylib`)
  * Logs every step: position, heading, velocities, PWM outputs, sensor values, events.
  * Detects track exit and finish line crossing.

---

## 📂 Project Structure

```
.
├── controller_example.py   # Example Python controller
├── robot_editor.py         # Robot editor (UI)
├── robot_model.py          # Robot data model
├── robot_geometry.py       # Robot geometry utilities
├── track_editor.py         # Track editor (UI)
├── track_model.py          # Track data model
├── track_geometry.py       # Track geometry utilities
├── simulation.py           # Simulation engine
```

---

## 🚀 Getting Started

### 1. Requirements

* Python **3.10+**
* Dependencies:

  ```bash
  pip install PySide6
  ```

### 2. Run the editors

* **Track Editor**

  ```bash
  python track_editor.py
  ```
* **Robot Editor**

  ```bash
  python robot_editor.py
  ```

### 3. Run the simulator

```bash
python simulation.py
```

---

## 🛠 Custom Controllers

You can implement your own control logic in **Python** or **C**.

### Python example

```python
def control_step(state):
    # state contains time, pose, velocities, wheelbase, and sensor data
    # return PWM signals for left and right motors
    return {"pwm_left": 2048, "pwm_right": 2048}
```

### C example (signature)

```c
void control_step(
    const double* sensors, int n,
    double x, double y, double heading_deg,
    double v, double w,
    int* pwm_left, int* pwm_right
);
```

See [`controller_example.py`](controller_example.py) for a reference.

---

## 🛠 Typical Workflow

1. Open the **Track Editor** and design a track.
2. Use the **Robot Editor** to configure robot size, wheels, and sensors.
3. Write a custom controller (`.py` or `.c`).
4. Run the **Simulator** and observe results.
5. Refine your robot design or controller and repeat.

---

## 📜 License

This project is distributed under the **MIT License**.
See the [LICENSE](LICENSE) file for details.
