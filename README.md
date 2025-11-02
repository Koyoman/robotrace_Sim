# 🏎️ RobotraceSim — Line-Follower Robot Simulator

RobotraceSim is an interactive **line-follower robot simulator**, developed in **Python + PySide6**, with support for external controllers written in **Python**.
It includes graphical tools for **track design** and **robot modeling**, plus a **simulation engine** with full logging(CSV/JSON).

---

## ✨ Features

### 🎯 Track Editor (`track_editor.py`)
- Visual creation of race tracks with **straight** and **arc** segments.
- Adjustable **area dimensions**, **tape width**, and **origin** position.
- Automatic **curvature-change markers** and **start/finish** line configuration.
- Import/export in **JSON format** (`Example/Track/track_1.json`).
- Intuitive interface with **zoom**, **drag**, and **keyboard shortcuts** (`Ctrl + O/S/F`).
- Start/Finish mode controls for **position**, **direction**, and **fine adjustment** using a slider.

### 🤖 Robot Editor (`robot_editor.py`)
- Define the **robot envelope** (up to 250×250 mm).
- Place **wheels** (22×15 mm) and **sensors** (5×5 mm) with **grid snapping**.
- Adjust the **origin (0, 0)** relative to the robot or selected sensor.
- Full **JSON import/export** support (`Example/Robot/robot-spec.json`).
- Interactive tools to rename, add, or remove sensors.
- Visualization with **zoom**, **pan**, and **reference axes**.

### 🧠 Simulation Engine (`simulation.py`)
- **Differential-drive kinematics** with **first-order motor dynamics** (`τ ≈ 0.1 s`).
- Sensor modeling using geometric intersection with the tape (`utills_c/linesim.dll`).
- External controller support:
  - **Python** (`.py`, e.g., `Example/Controller/PID_default.py`)
  - **C** (compiled `.dll`, `.so`, `.dylib` — via CTypes bindings)
- Automatic **logging** in `Logs/sim_log_YYYYMMDD_HHMMSS.csv/json`.
- Detects:
  - **Track exit**
  - **Finish line crossing**
  - **Line loss/recovery** via robust FSM logic
- Visual markers for **curvature changes**, **start**, and **finish** lines.

---

## 📂 Project Structure

```
RobotraceSim/
│
├── robot_editor.py
├── track_editor.py
├── simulation.py
│
├── Utils/
│   ├── robot_geometry.py
│   ├── robot_model.py
│   ├── track_geometry.py
│   └── track_model.py
│
├── utills_c/
│   ├── linesim.c/.dll/.lib
│   └── linesim.h
│
├── Example/
│   ├── Controller/
│   │   ├── P_basic.py
│   │   ├── PID_basic.py
│   │   ├── PID_basic_untuned.py
│   │   └── PID_default.py
│   │
│   ├── Robot/
│   │   └── robot-spec.json
│   │
│   └── Track/
│       └── track_1.json
│
└── README.md
```

---

## ⚙️ Requirements

- **Python 3.10+**
- **Dependencies:**
  ```bash
  pip install PySide6
  ```
- On **Windows**, the simulator uses the native DLL `utills_c/linesim.dll` (included).

---

## 🚀 Usage

### 1. Launch the editors
```bash
python track_editor.py   # Track Editor
python robot_editor.py   # Robot Editor
```

### 2. Run the simulator
```bash
python simulation.py
```

> The simulator provides an interactive interface to load:
> - Track (`Example/Track/track_1.json`)
> - Robot (`Example/Robot/robot-spec.json`)
> - Controller (`Example/Controller/PID_default.py`)

---

## 🧩 Custom Controllers

### 📘 Python Example
```python
def control_step(state):
    # state includes: time, position, velocity, heading, and sensors
    pwmL, pwmR = 2048, 2048
    return {"pwm_left": pwmL, "pwm_right": pwmR}
```

Full example: [`Example/Controller/PID_default.py`](Example/Controller/PID_default.py)
This controller implements a **classic PID** with:
- Sensor hysteresis filtering (`TH_LO/TH_HI`)
- Memory of the last detected side of the line (`last_side`)
- Automatic recovery behavior when all sensors lose the track

---

## 📈 Simulation Logs

Results are automatically saved under:
```
Logs/sim_log_YYYYMMDD_HHMMSS.csv
Logs/sim_log_YYYYMMDD_HHMMSS.json
```

Each step contains:
| Field | Description |
|-------|--------------|
| `t_ms` | Time (ms) |
| `x_mm`, `y_mm`, `heading_deg` | Position and heading |
| `v_mm_s`, `omega_rad_s` | Linear and angular velocities |
| `pwm_left`, `pwm_right` | PWM signals |
| `sensors[]` | Sensor readings |

---

## 🔄 Typical Workflow

1. ✏️ Design the track using **Track Editor**
2. 🧱 Configure the robot using **Robot Editor**
3. 🧠 Develop the controller (`.py`)
4. ▶️ Run the **Simulation Engine**
5. 📊 Analyze logs and fine-tune parameters

---

## 📜 License

Distributed under the **MIT License**.
See the [LICENSE](LICENSE) file for details.
