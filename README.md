# EMG and Elbow Angle Acquisition System

A real-time biomedical signal acquisition system that captures **EMG (Electromyography)** signals and **elbow joint angle** using dual MPU6050 IMU sensors on an Arduino Nano. The data is processed in **MATLAB/Simulink**, then streamed via **UDP** to a **Unity 3D** application for real-time arm motion visualization.

---

## 📋 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Project Structure](#project-structure)
- [Setup & Installation](#setup--installation)
  - [1. Arduino Firmware](#1-arduino-firmware)
  - [2. MATLAB / Simulink](#2-matlab--simulink)
  - [3. Unity 3D Visualization](#3-unity-3d-visualization)
- [Usage](#usage)
  - [Quick Start](#quick-start)
  - [Runtime Commands](#runtime-commands)
- [Signal Processing Pipeline](#signal-processing-pipeline)
  - [Elbow Angle (MPU6050)](#elbow-angle-mpu6050)
  - [EMG Signal](#emg-signal)
- [UDP Communication Protocol](#udp-communication-protocol)
- [Demo Video](#demo-video)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

This project was developed as part of a university capstone project (Đồ án). It demonstrates a complete end-to-end pipeline:

1. **Sensor Acquisition** — Arduino Nano reads EMG analog signal and dual MPU6050 quaternion data via I2C/DMP.
2. **Signal Processing** — MATLAB applies Butterworth low-pass filtering to EMG, computes elbow flexion angle from relative quaternion orientation.
3. **3D Visualization** — Unity receives processed data over UDP and animates a 3D human arm model in real time.

### Key Features

- **Dual MPU6050 DMP** — One on the upper arm (`0x68`), one on the forearm (`0x69`). Elbow angle is computed using the relative quaternion method (arm-axis vector angle), which is robust against wrist pronation/supination.
- **EMG Processing** — Real-time rectification, Butterworth low-pass envelope, baseline auto-calibration, and muscle activation detection.
- **Median + EMA Filter** — On-board angle filtering (window=5 median → exponential moving average, α=0.22) to remove sensor noise while preserving responsiveness.
- **Two MATLAB modes** — Standalone script (`run_emg_live_local.m`) or Simulink model (`emg_live_simulink.slx`) with custom System objects.
- **Zero-pose calibration** — Automatic at startup; re-calibrate anytime by sending `c` via Serial.

---

## System Architecture

```
┌─────────────┐    Serial     ┌───────────────────┐    UDP      ┌──────────────┐
│ Arduino Nano │──────────────▶│ MATLAB / Simulink │────────────▶│    Unity     │
│              │   115200 baud │                   │  127.0.0.1  │              │
│ • 2× MPU6050 │   CSV format  │ • Serial parse    │  port 55001 │ • 3D arm     │
│ • EMG sensor │               │ • EMG filter      │  30 Hz      │   model      │
│   (A0 pin)   │               │ • Butterworth LPF │             │ • Real-time  │
└─────────────┘               │ • UDP sender      │             │   animation  │
                               └───────────────────┘             └──────────────┘
```

---

## Hardware Requirements

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Microcontroller** | Arduino Nano (ATmega328P) | New bootloader variant |
| **IMU Sensor ×2** | MPU6050 (GY-521 breakout) | DMP6 firmware, I2C |
| **EMG Sensor** | EMG Analog Sensor (e.g., A10-09) | Analog output to pin A0 |
| **Wiring** | I2C bus (SDA=A4, SCL=A5) | Upper MPU: AD0→GND (`0x68`), Forearm MPU: AD0→VCC (`0x69`) |
| **Power** | USB from PC | Powers Arduino + sensors |

### Wiring Diagram

```
Arduino Nano
├── A0  ← EMG sensor OUT
├── A4 (SDA) ──┬── MPU6050 #1 (Upper arm, 0x68, AD0=GND)
├── A5 (SCL) ──┤
               └── MPU6050 #2 (Forearm, 0x69, AD0=VCC)
└── USB ── PC (COM9)
```

---

## Software Requirements

| Software | Version | Purpose |
|----------|---------|---------|
| **PlatformIO** (VS Code) | Latest | Arduino firmware build & upload |
| **MATLAB** | R2021a+ | Serial reading, EMG filtering, UDP sending |
| **Simulink** (optional) | R2021a+ | Alternative real-time pipeline with System objects |
| **Unity** | 2021.3+ | 3D visualization of arm motion |
| **Blender** (optional) | 3.x | 3D arm model editing |

### Arduino Libraries (auto-installed by PlatformIO)

- `jrowberg/I2Cdevlib-MPU6050@^1.0.0`
- `jrowberg/I2Cdevlib-Core@^1.0.0`

---

## Project Structure

```
EMG_and_Elbow_Angle_Acquisition_System/
│
├── EMG_MPU_VSCODE/                # Arduino firmware (PlatformIO project)
│   ├── src/
│   │   └── main.cpp               # Main firmware: dual MPU6050 + EMG → Serial CSV
│   ├── platformio.ini             # Build config: ATmega328P, 115200 baud, I2C libs
│   └── .vscode/                   # VS Code / PlatformIO IDE settings
│
├── EMG_MPU_MATLAB/                # MATLAB signal processing scripts
│   ├── run_emg_live_local.m       # Standalone live script (Serial → EMG filter → UDP)
│   ├── emg_live_simulink.slx      # Simulink model (alternative to script)
│   ├── SerialCsvReader.m          # Simulink System object: serial reader & CSV parser
│   ├── UdpUnitySender.m           # Simulink System object: UDP packet sender
│   ├── emg_init.m                 # Simulink model initialization parameters
│   ├── test_emg_mpu_to_unity_fake.m  # Test script: fake EMG+angle data → Unity
│   └── emg_mpu_signal_verification.slx  # Signal verification model
│
├── MPU_Testing_VSCODE/            # MPU6050 standalone testing & development
│   ├── MPU_success.cpp            # Full debug firmware: 14-column CSV with axis decomposition
│   └── MPU_matlab_unity success.m # MATLAB script for debug firmware → Unity
│
├── Unity System/                  # Unity 3D visualization project
│   └── Assets/
│       ├── Scripts/
│       │   └── LocalUdpElbowReceiver.cs  # UDP listener + smooth angle interpolation
│       ├── Models/                # 3D arm model assets
│       └── Scenes/                # Unity scene files
│
├── Model 3D/                      # Blender source files for the arm model
│   ├── Tao khoi1.blend            # Blender project file
│   └── Tao khoi_demo6.fbx         # Exported FBX for Unity import
│
├── Video EMG/                     # Demo video and Premiere project
│   └── Video simulation.mp4       # System demonstration video
│
└── README.md                      # This file
```

---

## Setup & Installation

### 1. Arduino Firmware

```bash
# Open the PlatformIO project in VS Code
cd EMG_MPU_VSCODE

# Edit platformio.ini if your Arduino is on a different COM port
# Default: COM9, 115200 baud

# Build and upload
pio run --target upload

# (Optional) Monitor serial output to verify CSV data
pio device monitor
```

> **Important:** Close the Serial Monitor before running MATLAB. Only one application can access the COM port at a time.

**Zero-Pose Calibration:**
- Keep your arm **straight and still** during the 3-second startup countdown.
- The current pose becomes **0°**.
- Send `c` or `C` via Serial to re-calibrate at any time.

### 2. MATLAB / Simulink

#### Option A: Standalone Script (recommended for first use)

```matlab
% 1. Open MATLAB
% 2. Navigate to EMG_MPU_MATLAB/
% 3. Edit COM port if needed (line 7):
%    PORT = "COM9";
% 4. Run:
run_emg_live_local
```

#### Option B: Simulink Model

```matlab
% 1. Run initialization script first:
emg_init

% 2. Open and run the Simulink model:
open_system('emg_live_simulink')
% Press the Run button in Simulink
```

#### Testing Without Hardware

```matlab
% Send fake EMG + angle data to Unity for testing:
test_emg_mpu_to_unity_fake
```

### 3. Unity 3D Visualization

1. Open `Unity System/` as a Unity project (Unity 2021.3+).
2. Open the main scene from `Assets/Scenes/`.
3. Verify the `LocalUdpElbowReceiver` component settings:
   - **Listen Port:** `55001` (must match MATLAB's `UNITY_PORT`)
   - **Input Min/Max Angle:** `0` / `140`
   - **Follow Speed:** `18` (smoothing factor)
4. Press **Play** in Unity Editor.
5. Then start MATLAB script — the 3D arm should move in real time.

---

## Usage

### Quick Start

1. **Power on** Arduino Nano with sensors connected.
2. **Upload firmware** (if not already done).
3. **Open Unity** project and press Play.
4. **Run MATLAB** script `run_emg_live_local.m`.
5. Keep arm straight for calibration (3 seconds).
6. Flex your elbow — the 3D arm mirrors your movement!

### Runtime Commands

| Action | How |
|--------|-----|
| **Re-calibrate zero pose** | Send `c` via Serial |
| **Stop MATLAB script** | Press `Ctrl+C` in Command Window |
| **Stop Simulink** | Press Stop in Simulink toolbar |
| **Stop Unity** | Press Stop in Unity Editor |

---

## Signal Processing Pipeline

### Elbow Angle (MPU6050)

1. **DMP Quaternion** — Each MPU6050 outputs a quaternion from its on-chip Digital Motion Processor.
2. **Sign Continuity** — Quaternion sign is kept consistent across frames to prevent 360° flips (`q` and `-q` represent the same rotation).
3. **Relative Quaternion** — `q_relative = conj(q_upper) × q_lower` gives forearm orientation in the upper arm's reference frame.
4. **Zero Compensation** — `q_delta = conj(q_relative_zero) × q_relative_now` removes the initial offset.
5. **Arm-Axis Vector Angle** — The local +X axis (longitudinal arm direction) is rotated by `q_delta`. The angle between the original and rotated +X axis is the elbow flexion angle. This method is **immune to wrist pronation/supination**.
6. **Median Filter** (window=5) — Removes impulse noise / outlier spikes.
7. **EMA Smoothing** (α=0.22) — `filtered += α × (median - filtered)` for smooth animation.

### EMG Signal

1. **Raw ADC** — `analogRead(A0)` → 10-bit (0–1023).
2. **Baseline Calibration** — Median of first 2 seconds of relaxed readings.
3. **Rectification** — `|raw - baseline|`
4. **Butterworth LPF** — 2nd-order, cutoff 5 Hz at fs=100 Hz → envelope extraction.
5. **Normalization** — `level = clamp(envelope / 512, 0, 1)`
6. **Activation Detection** — `active = (envelope > 35)`

---

## UDP Communication Protocol

**Direction:** MATLAB → Unity (localhost `127.0.0.1:55001`)

**Packet format (CSV string):**

```
time_ms,elbow_angle,emg_raw,emg_envelope,baseline,emg_envelope,active
```

| Field | Type | Range | Description |
|-------|------|-------|-------------|
| `time_ms` | int | 0+ | Arduino timestamp (ms) |
| `elbow_angle` | float | 0.0–140.0 | Filtered elbow flexion angle (°) |
| `emg_raw` | int | 0–1023 | Raw EMG ADC value |
| `emg_envelope` | float | 0+ | Filtered EMG envelope amplitude |
| `baseline` | float | 0+ | Calibrated EMG baseline |
| `active` | int | 0/1 | Muscle contraction detected |

**Rate:** 30 Hz (configurable via `UDP_RATE_HZ`)

---

## Demo Video

A demonstration video is available at [`Video EMG/Video simulation.mp4`](Video%20EMG/Video%20simulation.mp4).

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `Cannot open Serial COMx` | Close Arduino Serial Monitor / PlatformIO Monitor before running MATLAB |
| No data appearing in MATLAB | Check COM port number; verify Arduino is powered and sensors are wired |
| MPU6050 not found (`0x68` or `0x69`) | Check I2C wiring (SDA→A4, SCL→A5); verify AD0 pin level for each sensor |
| Angle jumps or drifts | Re-calibrate by sending `c`; keep arm still during calibration |
| Unity not receiving data | Ensure UDP port matches (`55001`); start Unity Play Mode before or after MATLAB |
| EMG baseline seems wrong | Keep muscle relaxed during the first 2 seconds after valid data appears |
| `[WARNING] No Serial bytes for >5s` | Check USB cable, Arduino power, or close other Serial Monitor instances |

---

## License

This project was developed for academic purposes as part of a university capstone project (Đồ án 1).

---

## Acknowledgments

- **I2Cdevlib** by Jeff Rowberg — MPU6050 DMP library
- **PlatformIO** — Embedded development platform
- **MathWorks** — MATLAB & Simulink
- **Unity Technologies** — 3D visualization engine
