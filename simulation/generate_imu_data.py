"""
Synthetic IMU Data Generator for INS_Mechanization
Author: Mohammed Hsiny
Year: 2026
"""

import numpy as np
import os

FS = 100.0
DT = 1.0 / FS
T_TOTAL = 90.0
N = int(T_TOTAL * FS)

R_EARTH = 6378137.0
GRAVITY = 9.80665
LAT0 = np.deg2rad(31.63)
LON0 = np.deg2rad(-7.98)
H0 = 100.0

ACC_BIAS = np.array([0.02, -0.015, 0.01])
GYRO_BIAS = np.array([0.001, 0.0005, -0.0008])
ACC_NOISE_STD = 0.01
GYRO_NOISE_STD = 0.001


def generate_trajectory():
    t = np.arange(N) * DT
    lat = np.zeros(N); lon = np.zeros(N); h = np.zeros(N)
    vn = np.zeros(N); ve = np.zeros(N); vd = np.zeros(N)
    roll = np.zeros(N); pitch = np.zeros(N); yaw = np.zeros(N)

    lat[0] = LAT0; lon[0] = LON0; h[0] = H0

    for i in range(1, N):
        if t[i] <= 10.0:
            vd[i] = -5.0 * (t[i] / 10.0)
        elif t[i] <= 40.0:
            vd[i] = -5.0; vn[i] = 15.0
        elif t[i] <= 50.0:
            vd[i] = -5.0
            progress = (t[i] - 40.0) / 10.0
            yaw[i] = progress * np.deg2rad(90.0)
            vn[i] = 15.0 * np.cos(yaw[i])
            ve[i] = 15.0 * np.sin(yaw[i])
            roll[i] = -np.deg2rad(15.0) * np.sin(progress * np.pi)
        elif t[i] <= 80.0:
            vd[i] = -5.0; ve[i] = 15.0; yaw[i] = np.deg2rad(90.0)
        else:
            vd[i] = -5.0 + 5.0 * ((t[i] - 80.0) / 10.0)
            yaw[i] = np.deg2rad(90.0)

        dlat = vn[i] * DT / R_EARTH
        dlon = ve[i] * DT / (R_EARTH * np.cos(lat[i-1]))
        dh = -vd[i] * DT
        lat[i] = lat[i-1] + dlat
        lon[i] = lon[i-1] + dlon
        h[i] = h[i-1] + dh

    return t, lat, lon, h, vn, ve, vd, roll, pitch, yaw


def compute_imu_measurements(t, lat, lon, h, vn, ve, vd, roll, pitch, yaw):
    roll_rate = np.gradient(roll, DT)
    pitch_rate = np.gradient(pitch, DT)
    yaw_rate = np.gradient(yaw, DT)

    wx = roll_rate - yaw_rate * np.sin(pitch)
    wy = pitch_rate * np.cos(roll) + yaw_rate * np.cos(pitch) * np.sin(roll)
    wz = -pitch_rate * np.sin(roll) + yaw_rate * np.cos(pitch) * np.cos(roll)

    ax_n = np.gradient(vn, DT)
    ay_n = np.gradient(ve, DT)
    az_n = np.gradient(vd, DT) - GRAVITY

    def rotation_matrix(r, p, y):
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        return np.array([
            [cy*cp, sy*cp, -sp],
            [cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr],
            [cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr]
        ])

    acc_body = np.zeros((N, 3))
    for i in range(N):
        R = rotation_matrix(roll[i], pitch[i], yaw[i])
        acc_nav = np.array([ax_n[i], ay_n[i], az_n[i]])
        acc_body[i] = R @ acc_nav

    np.random.seed(42)
    gyro = np.column_stack([wx, wy, wz]) + GYRO_BIAS + \
           np.random.randn(N, 3) * GYRO_NOISE_STD
    accel = acc_body + ACC_BIAS + \
            np.random.randn(N, 3) * ACC_NOISE_STD
    accel[:, 2] *= -1  # Flip Z to match NED convention used by the C++ code
    return gyro, accel


def write_imu_file(filename, t, gyro, accel):
    with open(filename, 'w') as f:
        f.write("# Time(s) Gyro_X Gyro_Y Gyro_Z Acc_X Acc_Y Acc_Z\n")
        for i in range(N):
            f.write(f"{t[i]:.6f} "
                    f"{gyro[i,0]:.8f} {gyro[i,1]:.8f} {gyro[i,2]:.8f} "
                    f"{accel[i,0]:.6f} {accel[i,1]:.6f} {accel[i,2]:.6f}\n")
    print(f"[OK] IMU file written: {filename}")


def write_nav_file(filename, t, lat, lon, h, vn, ve, vd, roll, pitch, yaw):
    with open(filename, 'w') as f:
        f.write("# Time(s) Lat(rad) Lon(rad) H(m) VN VE VD Roll Pitch Yaw\n")
        for i in range(N):
            f.write(f"{t[i]:.6f} "
                    f"{lat[i]:.10f} {lon[i]:.10f} {h[i]:.4f} "
                    f"{vn[i]:.4f} {ve[i]:.4f} {vd[i]:.4f} "
                    f"{roll[i]:.8f} {pitch[i]:.8f} {yaw[i]:.8f}\n")
    print(f"[OK] NAV file written: {filename}")


if __name__ == "__main__":
    print("=" * 60)
    print("Synthetic IMU Data Generator - Mohammed Hsiny")
    print("=" * 60)

    output_dir = "Data"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print("\n[1/4] Generating reference trajectory...")
    t, lat, lon, h, vn, ve, vd, roll, pitch, yaw = generate_trajectory()

    print("[2/4] Computing IMU measurements...")
    gyro, accel = compute_imu_measurements(t, lat, lon, h, vn, ve, vd,
                                           roll, pitch, yaw)

    print("[3/4] Writing files...")
    write_imu_file(os.path.join(output_dir, "GroupOne.ASC"), t, gyro, accel)
    write_nav_file(os.path.join(output_dir, "TruthOne.nav"), t, lat, lon, h,
                   vn, ve, vd, roll, pitch, yaw)

    print("\n[4/4] Summary:")
    print(f"  - Duration: {T_TOTAL} s")
    print(f"  - Samples: {N}")
    print(f"  - North distance: {np.sum(vn)*DT:.1f} m")
    print(f"  - East distance: {np.sum(ve)*DT:.1f} m")
    print("\n[OK] Data ready.")
    print("=" * 60)
