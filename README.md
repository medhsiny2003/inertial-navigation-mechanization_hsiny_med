# INS_Mechanization — Inertial Navigation System Mechanization

**Author:** Mohammed Hsiny  
**Field:** Electrical Engineering & Industrial Control Student  
**Institution:** Faculty of Sciences and Techniques of Mohammedia  
**Year:** 2026  
**Repository:** inertial-navigation-mechanization_hsiny_med  

---

## Overview

This project implements a complete **Inertial Navigation System (INS) mechanization** in C++ for a strapdown IMU, with trajectory analysis and error visualization in GNU Octave. It includes a Python-based synthetic IMU data generator to simulate realistic flight trajectories and sensor imperfections (biases, noise).

The goal is to demonstrate, through simulation and quantitative analysis, **why pure inertial navigation cannot remain accurate over long durations**, and to establish the foundation for a sensor-fusion architecture (IMU + aiding sources) suitable for autonomous drone navigation in GPS-denied environments.

---

## Motivation

The starting point of this project came from observing the massive use of autonomous drones in recent conflicts — Ukraine, Russia, Iran — where electronic warfare and GPS jamming have made satellite-based navigation unreliable. The operational lesson is clear: **the future of autonomous aerial systems depends on inertial navigation and onboard sensor fusion**, not on external signals that can be denied.

This raised a question I wanted to answer rigorously:

> *Can a drone navigate autonomously using only an IMU, without any external aiding source?*

Historically, the first system to attempt this was the **V2 ballistic missile** (Germany, 1942–1945), the world's first operational ballistic missile. Its guidance system was purely inertial: a stabilized gyroscopic platform combined with a pendulous integrating gyroscopic accelerometer (PIGA) and an analog computer (the *Mischgerät*) that cut off the engine once the required velocity was reached. It was brilliant for its time — but **inaccurate by design**, with a CEP of several kilometers.

I wanted to understand *why* — mathematically, physically, and computationally.

---

## What This Project Does

### 1. IMU Mechanization (C++)

The core of the project is a full strapdown INS mechanization implemented in C++11, using the **Eigen3** library for linear algebra. It performs three fundamental updates at each time step:

- **Attitude update** — quaternion propagation from gyroscope measurements using equivalent rotation vector integration.
- **Velocity update** — integration of accelerometer data, corrected for Earth rotation (Coriolis effect $2\boldsymbol{\omega}_{ie}^n$), transport rate ($\boldsymbol{\omega}_{en}^n$), sculling motion, and normal gravity on the WGS84 ellipsoid.
- **Position update** — recursive integration of velocity over the WGS84 ellipsoid with meridian and prime vertical radii of curvature ($R_M, R_N$).

The mechanization also includes:
- **Static bias calibration** for both accelerometer and gyroscope.
- **Zero Velocity Update (ZVU)** correction.
- Configurable processing modes (example benchmark dataset vs. self-generated synthetic flight data).

### 2. Synthetic IMU Data Generator (Python)

A Python script (`simulation/generate_imu_data.py`) generates realistic synthetic IMU data for a 90-second drone flight across 5 flight phases:
1. Vertical takeoff ($0 - 10\,\text{s}$)
2. Level flight North ($10 - 40\,\text{s}$)
3. Coordinated right turn ($40 - 50\,\text{s}$)
4. Level flight East ($50 - 80\,\text{s}$)
5. Controlled descent ($80 - 90\,\text{s}$)

Realistic sensor biases and Gaussian noise are added to the accelerometer and gyroscope outputs, and a corresponding **ground-truth trajectory** is produced for comparison.

### 3. Error Analysis (GNU Octave / MATLAB)

A suite of analysis scripts computes and visualizes:
- 3D trajectory comparison (estimated vs. ground truth)
- Position error over time (horizontal and vertical ENU drift)
- Velocity and attitude error curves
- Moving-window **RMSE** evolution over time

The expected result — and the central lesson of this project — is that the position error **grows quadratically with time** under pure inertial navigation, even with well-calibrated sensors.

---

## Mathematical Foundation

The mechanization is based on the standard strapdown INS equations, as described in classical references (Titterton & Weston, *Strapdown Inertial Navigation Technology*; Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*; Farrell, *Aided Navigation*).

Key relations implemented:

- **Quaternion kinematics**:
  $$\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes \boldsymbol{\omega}_{nb}^b$$
- **Velocity dynamics in the navigation frame**:
  $$\dot{\mathbf{v}}^n = \mathbf{C}_b^n \mathbf{f}^b - (2\boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n) \times \mathbf{v}^n + \mathbf{g}^n(\phi, h)$$
- **Geodetic position rates over the WGS84 ellipsoid**:
  $$\dot{\phi} = \frac{v_N}{R_M + h}, \quad \dot{\lambda} = \frac{v_E}{(R_N + h)\cos\phi}, \quad \dot{h} = -v_D$$

---

## Project Structure

```text
.
├── code/
│   ├── main.cpp                    # Mechanization runner (Mode 0: Benchmark, Mode 1: Simulated)
│   ├── IMU_Structs.h               # Data structures, geodetic constants, math interfaces
│   ├── NavigationUpdate.cpp        # Attitude, Velocity, and Position strapdown update routines
│   ├── Calculations.cpp            # Quaternions, normal gravity, extrapolation algorithms
│   ├── CoordinateChange.cpp        # Geodetic (BLH) <-> ECEF (XYZ) <-> Local ENU transforms
│   ├── Calibration.cpp             # Static bias estimation and sensor compensation
│   ├── ReadFile.cpp                # Universal ASCII / Binary telemetry file parser
│   ├── SaveFile.cpp                # Result, error, and trajectory exporters
│   ├── MatrixPrint.cpp             # Debug utilities for Eigen vectors and matrices
│   ├── Data/                       # Working data directory (GroupOne.ASC, TruthOne.nav)
│   └── Result/                     # Output solution files and Octave/MATLAB scripts
│       ├── main.m                  # Interactive visualizer menu
│       ├── DataFig.m               # State comparison plots (9 subplots)
│       ├── DiffFig.m               # Residuals and error curves
│       ├── dEnuFig.m               # 2D/3D ENU trajectory visualization
│       ├── RmseWithTime.m          # Moving-window RMSE & drift rate computation
│       ├── ReadFile.m              # Universal data parser for Octave & MATLAB
│       ├── rmse.m                  # Root Mean Square Error utility
│       ├── table2array.m           # Octave matrix compatibility helper
│       └── Figures/                # Benchmarks and reference plots
├── report/
│   ├── lab_report.pdf              # Comprehensive technical report
│   └── diagrams.pptx               # System architecture and workflow diagrams
├── simulation/
│   ├── generate_imu_data.py        # 90s drone flight IMU synthetic data generator
│   ├── requirements.txt            # Python dependencies (numpy, matplotlib, scipy, pandas)
│   └── README.md                   # Simulation user guide
└── README.md
```

---

## Compilation & Execution

### Prerequisites
- C++11 compliant compiler (`g++` via MSYS2 / MinGW-w64 on Windows, or GCC on Linux)
- Eigen3 linear algebra library
- Python 3.8+ with NumPy
- GNU Octave (or MATLAB) for visualization

### 1. Generating Simulation Data
```bash
cd simulation
pip install -r requirements.txt
python generate_imu_data.py
```
Copy the generated files to the `code/Data/` directory:
```bash
cp Data/GroupOne.ASC ../code/Data/
cp Data/TruthOne.nav ../code/Data/
```

### 2. Compiling the C++ Mechanization
```bash
cd ../code
g++ -std=c++11 main.cpp Calculations.cpp Calibration.cpp CoordinateChange.cpp \
    NavigationUpdate.cpp ReadFile.cpp SaveFile.cpp MatrixPrint.cpp \
    -I /mingw64/include/eigen3 -o ins_mechanization.exe
```

### 3. Running the Solver
```bash
./ins_mechanization.exe 1
```
*(Use `1` for self-generated flight data, or `0` for benchmark binary logs).*

### 4. Analyzing Results with GNU Octave
```matlab
cd code/Result
main
```
Select from the interactive menu:
- `1` : Plot difference between estimated results and ground truth
- `2` : Plot comparison between estimated states and ground truth
- `3` : Plot 2D/3D trajectory comparison in ENU frame
- `4` : Plot moving-window RMSE evolution over time
