# Simulation Data Generator

Author: Mohammed Hsiny

This folder contains a Python script that generates synthetic IMU data
for testing the INS_Mechanization project.

## Installation
```bash
pip install -r requirements.txt
```

## Usage
```bash
python generate_imu_data.py
```

This creates a Data/ folder containing:
- GroupOne.ASC : Raw IMU measurements (gyro + accelerometer)
- TruthOne.nav : Ground truth trajectory

## Integration

Copy the files to code/Data/:
```bash
cp Data/GroupOne.ASC ../code/Data/
cp Data/TruthOne.nav ../code/Data/
```

Then run the C++ program in Mode 1 (Self-test).

## Expected Results

The position error should grow quadratically with time, demonstrating
the inherent drift of pure inertial navigation.
