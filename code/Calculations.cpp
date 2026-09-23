// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Field: Electrical Engineering & Industrial Control Student
// Institution: Faculty of Sciences and Techniques of Mohammedia
// Year: 2026
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

/* Quaternion Multiplication */
// Output: result = p (x) q
void QuaternionMultiply(Quater p, Quater q, Quater* result)
{
    result->q[0] = p.q[0] * q.q[0] - p.q[1] * q.q[1] - p.q[2] * q.q[2] - p.q[3] * q.q[3];
    result->q[1] = p.q[0] * q.q[1] + p.q[1] * q.q[0] + p.q[2] * q.q[3] - p.q[3] * q.q[2];
    result->q[2] = p.q[0] * q.q[2] - p.q[1] * q.q[3] + p.q[2] * q.q[0] + p.q[3] * q.q[1];
    result->q[3] = p.q[0] * q.q[3] + p.q[1] * q.q[2] - p.q[2] * q.q[1] + p.q[3] * q.q[0];
}

/* Compute Attitude Angles from Quaternion */
// Input: Quaternion Q
// Output: Attitude angles in result->pos (rad)
void CalPostureWithQuaternion(Quater Q, INSDataEpoch* result)
{
    double C32 = 2.0 * (Q.q[2] * Q.q[3] + Q.q[0] * Q.q[1]);
    double C33 = pow(Q.q[0], 2) - pow(Q.q[1], 2) - pow(Q.q[2], 2) + pow(Q.q[3], 2);
    result->pos.roll = atan2(C32, C33);    // Roll angle (rad)

    double C31 = 2.0 * (Q.q[1] * Q.q[3] - Q.q[0] * Q.q[2]);
    result->pos.pitch = atan(-C31 / sqrt(C32 * C32 + C33 * C33));  // Pitch angle (rad)

    double C11 = pow(Q.q[0], 2) + pow(Q.q[1], 2) - pow(Q.q[2], 2) - pow(Q.q[3], 2);
    double C21 = 2.0 * (Q.q[1] * Q.q[2] + Q.q[0] * Q.q[3]);
    result->pos.yaw = atan2(C21, C11); // Heading / Yaw angle (rad)
}

/* Skew-Symmetric Matrix of a 3D Vector */
// Output: result matrix
void SkewSymmetricMatrix(Vector3d vec, Matrix3d* result)
{
    (*result) << 0, -vec[2], vec[1],
                 vec[2], 0, -vec[0],
                -vec[1], vec[0], 0;
}

/* Compute Normal Gravity at a Given Position (Latitude & Height) */
// Output: gpn vector
void Calgpn(POSITION blh, Vector3d* gpn)
{
    double g0;
    g0 = 9.7803267715 * (1 + 0.0052790414 * pow(sin(blh.latitude), 2) + 0.0000232718 * pow(sin(blh.latitude), 4));

    double g_faih;
    g_faih = g0 - (3.087691089e-6 - 4.397731e-9 * pow(sin(blh.latitude), 2)) * blh.H + 0.721e-12 * pow(blh.H, 2);

    (*gpn) << 0, 0, g_faih;
}

/* Extrapolate Angular Velocity and Velocity */
// Input: tk, tk_1, tk_2, previous velocities and positions
// Output: w_ie_n, w_en_n, and v_k_12 at midpoint k-1/2
void Extrapolation(double tk, double tk_1, double tk_2, VELOCITY vel_prv, VELOCITY vel_pprv, POSITION blh_prv, POSITION blh_pprv, Vector3d* w_ie_n, Vector3d* w_en_n, Vector3d* v_k_12)
{
    // 1. Angular rate interpolation
    double R_M_k_1 = a * (1 - pow(e, 2)) / sqrt(pow(1 - pow(e * sin(blh_prv.latitude), 2), 3));
    double R_M_k_2 = a * (1 - pow(e, 2)) / sqrt(pow(1 - pow(e * sin(blh_pprv.latitude), 2), 3));
    double R_N_k_1 = a / sqrt(1 - pow(e * sin(blh_prv.latitude), 2));
    double R_N_k_2 = a / sqrt(1 - pow(e * sin(blh_pprv.latitude), 2));

    double wie_1[3] = { we * cos(blh_prv.latitude), 0, -we * sin(blh_prv.latitude) };
    double wie_2[3] = { we * cos(blh_pprv.latitude), 0, -we * sin(blh_pprv.latitude) };

    double wen_1[3] = { vel_prv.Ve / (R_N_k_1 + blh_prv.H), -vel_prv.Vn / (R_M_k_1 + blh_prv.H), -vel_prv.Ve * tan(blh_prv.latitude) / (R_N_k_1 + blh_prv.H) };
    double wen_2[3] = { vel_pprv.Ve / (R_N_k_2 + blh_pprv.H), -vel_pprv.Vn / (R_M_k_2 + blh_pprv.H), -vel_pprv.Ve * tan(blh_pprv.latitude) / (R_N_k_2 + blh_pprv.H) };

    double dt2 = tk_1 - tk_2;
    double dt1 = (tk - tk_1) / 2.0;

    double wie_k[3] = { 0.0, 0.0, 0.0 };
    double wen_k[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < 3; i++)
    {
        wie_k[i] = (dt2 != 0) ? (wie_1[i] + dt1 * (wie_1[i] - wie_2[i]) / dt2) : wie_1[i];
        wen_k[i] = (dt2 != 0) ? (wen_1[i] + dt1 * (wen_1[i] - wen_2[i]) / dt2) : wen_1[i];
    }
    (*w_ie_n) << wie_k[0], wie_k[1], wie_k[2];
    (*w_en_n) << wen_k[0], wen_k[1], wen_k[2];

    // 2. Velocity interpolation
    double v_1[3] = { vel_prv.Vn, vel_prv.Ve, vel_prv.Vd };
    double v_2[3] = { vel_pprv.Vn, vel_pprv.Ve, vel_pprv.Vd };

    double v_k[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < 3; i++)
    {
        v_k[i] = (dt2 != 0) ? (v_1[i] + dt1 * (v_1[i] - v_2[i]) / dt2) : v_1[i];
    }
    (*v_k_12) << v_k[0], v_k[1], v_k[2];
}

/* Calculate Increments */
void CalculateIncrement(IMUDataEpoch* data_prv, IMUDataEpoch* data_cur)
{
    // Gyroscope increments
    data_cur->Gyr.X -= data_prv->Gyr.X;
    data_cur->Gyr.Y -= data_prv->Gyr.Y;
    data_cur->Gyr.Z -= data_prv->Gyr.Z;

    // Accelerometer increments
    data_cur->Acc.X -= data_prv->Acc.X;
    data_cur->Acc.Y -= data_prv->Acc.Y;
    data_cur->Acc.Z -= data_prv->Acc.Z;
}