# INS_Mechanization

**Author:** Mohammed Hsiny  
**Title:** Élève ingénieur en génie électrique et contrôle industriel  
**Institution:** Faculté des Sciences et Techniques de Mohammedia  
**Based on the original project by:** GYH-WHU  
**Year:** 2025  
**Repository:** inertial-navigation-mechanization_hsiny_med  

> This project is a modified, translated, and personalized version of the original GYH-WHU/INS_Mechanization repository, adapted for educational purposes.

---

## Overview

This repository contains an implementation of pure Inertial Navigation System (INS) mechanization algorithms. Built in C++ using the Eigen linear algebra library, it processes raw IMU measurements (accelerometers and gyroscopes) to recursively calculate attitude, velocity, and position in the local navigation frame (East-North-Up / North-East-Down on the WGS-84 reference ellipsoid).

The project includes:
- Pure INS mechanization algorithms in C++.
- Attitude update via equivalent rotation vector and quaternions.
- Velocity update with sculling, gravity, and Coriolis compensations.
- Position update on the WGS-84 ellipsoid.
- Synthetic drone trajectory and IMU measurement simulation in Python.
- Analysis and visualization scripts for GNU Octave and MATLAB.
- Comprehensive technical report and diagrams in the `report/` folder.

---

## Key Features

1. **Attitude Update**: High-precision quaternion integration and equivalent rotation vector algorithm.
2. **Velocity Update**: Sculling compensation, normal gravity calculation ($g(\phi, h)$), and Coriolis force compensation ($2\boldsymbol{\omega}_{ie}^n + \boldsymbol{\omega}_{en}^n$).
3. **Position Update**: Geodetic coordinates ($\phi, \lambda, h$) integration with curvature radii ($R_M, R_N$).
4. **Synthetic Data Generator**: Python script simulating a 90-second 5-phase drone flight with realistic MEMS IMU sensor noise and biases.
5. **Octave/MATLAB Analysis**: Trajectory visualization, difference analysis against ground truth, and RMSE calculation over time.

---

## Project Structure

```text
.
├── code/
│   ├── main.cpp                    # Main interactive entry point (0: ExampleData, 1: SelfData)
│   ├── IMU_Structs.h               # Data structures, constants, and function declarations
│   ├── NavigationUpdate.cpp        # Attitude, Velocity, and Position mechanization updates
│   ├── Calculations.cpp            # Quaternions, normal gravity, extrapolation routines
│   ├── CoordinateChange.cpp        # Geodetic (BLH), ECEF (XYZ), and ENU transformations
│   ├── Calibration.cpp             # Accelerometer and gyroscope bias estimation
│   ├── ReadFile.cpp                # Binary/ASCII IMU and ground truth file parsers
│   ├── SaveFile.cpp                # Result, ground truth, error, and trajectory exporters
│   ├── MatrixPrint.cpp             # Debug printing utilities for Eigen matrices
│   ├── Data/                       # Input data directory (GroupOne.ASC, TruthOne.nav)
│   └── Result/                     # Output results and Octave/MATLAB plotting scripts
│       ├── main.m                  # Interactive visualization menu
│       ├── DataFig.m               # State comparison plots
│       ├── DiffFig.m               # Error curve plots
│       ├── dEnuFig.m               # 2D/3D ENU trajectory plots
│       ├── RmseWithTime.m          # Moving-window RMSE evolution
│       ├── ReadFile.m              # Result parser helper
│       ├── rmse.m                  # Root Mean Square Error utility
│       └── Figures/                # Pre-rendered benchmark figures
├── report/
│   ├── lab_report.pdf              # Comprehensive technical report
│   └── diagrams.pptx               # System architecture and workflow diagrams
├── simulation/
│   ├── generate_imu_data.py        # 90s drone flight IMU synthetic data generator
│   ├── requirements.txt            # Python dependencies (numpy)
│   └── README.md                   # Simulation documentation
└── README.md
```

---

## Simulation Data Generation

If you don't have access to hardware IMU logs, you can generate synthetic IMU data using the Python script in `simulation/`.

### Requirements
```bash
pip install -r simulation/requirements.txt
```

### Generating Data
```bash
cd simulation
python generate_imu_data.py
```

This creates:
- `simulation/Data/GroupOne.ASC`: Synthetic IMU gyro & accelerometer readings.
- `simulation/Data/TruthOne.nav`: Ground truth drone trajectory.

Copy the generated files to the `code/Data/` folder:
```bash
cp Data/GroupOne.ASC ../code/Data/
cp Data/TruthOne.nav ../code/Data/
```

---

## Compilation

### Prerequisites
- C++11 compiler (`g++` via MSYS2 / MinGW-w64)
- Eigen3 linear algebra library

### Install Eigen3 (MSYS2)
```bash
pacman -S mingw-w64-x86_64-eigen3
```

### Compile
```bash
cd code
g++ -std=c++11 main.cpp Calculations.cpp Calibration.cpp CoordinateChange.cpp \
    NavigationUpdate.cpp ReadFile.cpp SaveFile.cpp MatrixPrint.cpp \
    -I /mingw64/include/eigen3 -o ins_mechanization.exe
```

### Run
```bash
./ins_mechanization.exe
```
Choose Mode 1 (Self-test / Simulated data).

---

## Analysis with GNU Octave

Launch GNU Octave and execute:
```matlab
cd code/Result
main
```

Available analysis options in the menu:
- `0`: Compute initial average velocity, position, and attitude.
- `1`: Plot difference between estimated results and ground truth.
- `2`: Plot comparison between estimated results and ground truth.
- `3`: Plot 2D/3D trajectory comparison.
- `4`: Plot RMSE evolution over time.
- `5`: Plot example reference data difference and comparison.

---

## Authors & Attribution

- **Original Author:** GYH-WHU
- **Modified, Translated, and Enhanced by:** Mohammed Hsiny
- **Title:** Élève ingénieur en génie électrique et contrôle industriel (Faculté des Sciences et Techniques de Mohammedia)
- **Year:** 2025 - 2026
