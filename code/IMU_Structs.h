// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Title: Élève ingénieur en génie électrique et contrôle industriel
// Based on the original project by GYH-WHU
// Year: 2025
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#pragma once
#include <cmath>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdio>

using namespace std;
using namespace Eigen;

/*-----------------------------------------------
    Constant Definitions
------------------------------------------------*/
/* Mathematical Constants */
#define PAI 3.141592653589      // Pi
#define Deg (180.0 / PAI)       // Radians to degrees conversion factor
#define Rad (PAI / 180.0)       // Degrees to radians conversion factor

/* WGS84 Ellipsoid Parameters */
#define a 6378137.0             // Semi-major axis (m)
#define b 6356752.3142          // Semi-minor axis (m)
#define e (sqrt(a * a - b * b) / a)       // First eccentricity
#define we 7.292115e-5          // Earth rotation rate (rad/s)
#define gravity 9.7936174       // Standard gravity acceleration (m/s^2)
#define R_WGS84 6378137.0       // Earth semi-major axis (m)
#define F_WGS84 (1.0 / 298.257223563)   // Earth flattening

// Initial Constant Definitions
/* Reference Data */
const double initial_time = 91620.0;                    // Initial time (s)
const double initial_latitude = 23.1373950708 * Rad;    // Latitude (rad)
const double initial_longitude = 113.3713651222 * Rad;  // Longitude (rad)
const double initial_elevation = 2.175;                 // Altitude / Height (m)
const double initial_velocity_north = 0.0;              // North velocity (m/s)
const double initial_velocity_east = 0.0;               // East velocity (m/s)
const double initial_velocity_upward = 0.0;             // Upward velocity (m/s)
const double initial_roll = 0.0107951084511778 * Rad;   // Roll angle (rad)
const double initial_pitch = -2.14251290749072 * Rad;   // Pitch angle (rad)
const double initial_heading = -75.7498049314083 * Rad; // Heading angle (rad)

/* Self-test Data */
const double starttime = 440532.000;    // Reference start time
const double endtime = 442609.990;      // Reference end time

const double ours_initial_week = 2390;          // Initial GPS week
const double ours_initial_second = 440190.020;  // Initial second of week
const double ours_initial_latitude = 30.5279685193 * Rad;     // Initial latitude
const double ours_initial_longitude = 114.3555367313 * Rad;   // Initial longitude
const double ours_initial_height = 23.3659979239;             // Initial height
const double ours_initial_Vn = 0.0;             // Initial North velocity
const double ours_initial_Ve = 0.0;             // Initial East velocity
const double ours_initial_Vd = 0.0;             // Initial Down velocity
const double ours_initial_roll = -0.2236406320 * Rad;           // Initial roll
const double ours_initial_pitch = -0.0932009081 * Rad;          // Initial pitch
const double ours_initial_yaw = (189.9907547479 - 360.0) * Rad; // Initial yaw / heading

const int zero_time_intervals_num = 14;         // Number of zero-velocity intervals

/* Device Types */
enum DeviceType { XWGI, NSC, CGI };     // XWGI: calibration, NSC: alignment, CGI: pure INS
#define rate_CGI 100                    // CGI sampling frequency (Hz)
#define acc_CGI (1.0 / 655360.0)        // CGI accelerometer scale factor
#define gyr_CGI (1.0 / 160849.543863)   // CGI gyroscope scale factor

#define rate_XWGI 100                   // XWGI sampling rate (Hz)
#define acc_XWGI 1.5258789063e-6        // XWGI accelerometer scale factor
#define gyr_XWGI 1.0850694444e-7        // XWGI gyroscope scale factor

/*-----------------------------------------------
    Structure Definitions
------------------------------------------------*/
/* GPS Time */
struct GPSTIME
{
    int Week;           // GPS Week
    double Second;      // Second of Week
    GPSTIME() { Week = 0; Second = 0; }
};

/* Accelerometer Data */
struct ACCDAT
{
    double X, Y, Z;
    ACCDAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/* Gyroscope Data */
struct GYRODAT
{
    double X, Y, Z;
    GYRODAT() { X = 0.0; Y = 0.0; Z = 0.0; }
};

/* Position Result */
struct POSITION
{
    double latitude;    // Latitude (rad or deg depending on context)
    double longitude;   // Longitude (rad or deg)
    double H;           // Height (m)
    POSITION() { latitude = longitude = H = 0.0; }
};

/* Trajectory Offset (dENU) */
struct dENU
{
    double dE;
    double dN;
    double dU;
    dENU() { dE = dN = dU = 0.0; }
};

/* Velocity Result */
struct VELOCITY
{
    double Vn, Ve, Vd;  // North, East, Down velocity (m/s)
    VELOCITY() { Vn = Ve = Vd = 0.0; }
};

/* Attitude / Posture Result */
struct POSTURE
{
    double roll, pitch, yaw;    // Roll, Pitch, Yaw (rad)
    POSTURE() { roll = pitch = yaw = 0.0; }
};

/* Raw Data (Reference Data) */
struct RAWDAT
{
    GPSTIME time;       // Week, Second
    ACCDAT acc;         // Accelerometer measurement
    GYRODAT gyr;        // Gyroscope measurement (rad)
    POSITION blh;       // Reference position (Lat, Lon, H)
    VELOCITY vel;       // Reference velocity
    POSTURE pos;        // Reference attitude
};

/* Quaternion Structure */
struct Quater
{
    double q[4];        // q0, q1, q2, q3
    Quater() { for (int i = 0; i < 4; i++) { q[i] = 0.0; } }

    // Set quaternion for body frame update (input: angular increment vector in radians)
    void Setkk_1(POSTURE pos)
    {
        double mol_pos12 = sqrt(pow(pos.roll / 2.0, 2) + pow(pos.pitch / 2.0, 2) + pow(pos.yaw / 2.0, 2));
        double coef = 0.0;
        if (mol_pos12 != 0) coef = sin(mol_pos12) / mol_pos12;
        q[0] = cos(mol_pos12);
        q[1] = 0.5 * pos.roll * coef;
        q[2] = 0.5 * pos.pitch * coef;
        q[3] = 0.5 * pos.yaw * coef;        
    }

    // Set quaternion for navigation frame update (input: angular increment vector in radians)
    void Setk_1k(POSTURE pos)
    {
        double mol_pos12 = sqrt(pow(pos.roll / 2.0, 2) + pow(pos.pitch / 2.0, 2) + pow(pos.yaw / 2.0, 2));
        double coef = (mol_pos12 != 0) ? (sin(mol_pos12) / mol_pos12) : 1.0;
        q[0] = cos(mol_pos12);
        q[1] = -0.5 * pos.roll * coef;
        q[2] = -0.5 * pos.pitch * coef;
        q[3] = -0.5 * pos.yaw * coef;        
    }

    // Set initial attitude quaternion
    void SetQbn(POSTURE pos)
    {
        double fai = pos.roll / 2.0, theta = pos.pitch / 2.0, pusai = pos.yaw / 2.0;
        q[0] = cos(fai) * cos(theta) * cos(pusai) + sin(fai) * sin(theta) * sin(pusai);
        q[1] = sin(fai) * cos(theta) * cos(pusai) - cos(fai) * sin(theta) * sin(pusai);
        q[2] = cos(fai) * sin(theta) * cos(pusai) + sin(fai) * cos(theta) * sin(pusai);
        q[3] = cos(fai) * cos(theta) * sin(pusai) - sin(fai) * sin(theta) * cos(pusai);
    }
};

/* Direction Cosine Matrix */
struct CosineMatrix
{
    double row1[3];
    double row2[3];
    double row3[3];
    Matrix3d Cbn = Matrix3d::Zero(3, 3);

    CosineMatrix() { for (int i = 0; i < 3; i++) { row1[i] = row2[i] = row3[i] = 0.0; } }

    // Set Cbn direction cosine matrix from Euler angles
    void SetCbn(POSTURE pos)
    {
        double theta = pos.pitch, pusai = pos.yaw, fai = pos.roll;  // Pitch, Yaw, Roll (rad)
        row1[0] = cos(theta) * cos(pusai);
        row1[1] = -cos(fai) * sin(pusai) + sin(fai) * sin(theta) * cos(pusai);
        row1[2] = sin(fai) * sin(pusai) + cos(fai) * sin(theta) * cos(pusai);
        row2[0] = cos(theta) * sin(pusai);
        row2[1] = cos(fai) * cos(pusai) + sin(fai) * sin(theta) * sin(pusai);
        row2[2] = -sin(fai) * cos(pusai) + cos(fai) * sin(theta) * sin(pusai);
        row3[0] = -sin(theta);
        row3[1] = sin(fai) * cos(theta);
        row3[2] = cos(fai) * cos(theta);

        for (int i = 0; i < 3; i++)
        {
            Cbn(0, i) = row1[i];
            Cbn(1, i) = row2[i];
            Cbn(2, i) = row3[i];
        }
    }
};

/* IMU Data Epoch Structure */
struct IMUDataEpoch 
{
    double TimeStamp;   // Timestamp (s)
    GYRODAT Gyr;        // Angular increment (rad)
    ACCDAT Acc;         // Velocity increment (m/s)
    IMUDataEpoch() { TimeStamp = 0.0; }
};

/* INS Reference Result Epoch Structure */
struct INSDataEpoch 
{
    double TimeStamp;   // Timestamp (s)
    POSITION blh;       // Position (deg / m)
    VELOCITY vel;       // Velocity (m/s)
    POSTURE pos;        // Attitude (deg / rad)
    INSDataEpoch() { TimeStamp = 0.0; }
};

/* Zero-Velocity Update Time Interval */
struct TimeInterval
{
    double start;       // Stationary start time
    double end;         // Motion start time
    bool used;          // Has this interval been processed

    TimeInterval(double starttime = 0.0, double endtime = 0.0, bool isUsed = false)
        : start(starttime), end(endtime), used(isUsed) { }
};

/* Zero-Velocity Time Intervals Array */
struct TimeIntervalsArray
{
    TimeInterval interval[zero_time_intervals_num];

    TimeIntervalsArray()
    {
        interval[0] = TimeInterval(440664, 440708, false);
        interval[1] = TimeInterval(440762, 440838, false);
        interval[2] = TimeInterval(440883, 440915, false);
        interval[3] = TimeInterval(440974, 441044, false);
        interval[4] = TimeInterval(441116, 441171, false);
        interval[5] = TimeInterval(441225, 441271, false);
        interval[6] = TimeInterval(441361, 441428, false);
        interval[7] = TimeInterval(441538, 441597, false);
        interval[8] = TimeInterval(441656, 441680, false);
        interval[9] = TimeInterval(441758, 441796, false);
        interval[10] = TimeInterval(441894, 441926, false);
        interval[11] = TimeInterval(442020, 442063, false);
        interval[12] = TimeInterval(442144, 442233, false);
        interval[13] = TimeInterval(442319, 442609.990, false);
    }

    TimeInterval& getInterval(int index)
    {
        if (index >= 0 && index < zero_time_intervals_num) return interval[index];
        else {
            cout << "Index out of time interval bounds!\n";
            return interval[0];
        }
    }
};

/*-----------------------------------------------
    File Operation Functions
------------------------------------------------*/
bool ReadExamplePureIMUData(FILE* file, IMUDataEpoch* mImuDataEpoch);       // Read binary example IMU raw data
bool ReadExamplePureINSData(FILE* file, INSDataEpoch* mInsResultEpoch);     // Read binary example INS reference data
void SaveExampleData(FILE* OutputFile, IMUDataEpoch mImu, INSDataEpoch mIns, int mode);         // Save example data

bool ReadIMURawData_CGI(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device);  // Read CGI raw data
bool ReadIMURawData_Else(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device); // Read other IMU raw data
bool ReadTruthData(ifstream& file, INSDataEpoch* Rawdata);                          // Read reference truth data
void SaveOurResult(FILE* OutputFile, IMUDataEpoch mImu, VELOCITY vel, POSITION blh, POSTURE pos);   // Save computation result
void SaveTrueResult(FILE* OutputFile, INSDataEpoch mIns);                          // Save reference truth result
void SaveDiffResult(FILE* OutputFile, IMUDataEpoch mImu, double dvel[3], double dblh[3], double dpos[3]); // Save difference result
void SavedENUResult(FILE* OutputFile, IMUDataEpoch mImu, dENU truedata, dENU ourdata);             // Save dENU result

/*-----------------------------------------------
    Matrix Printing Functions
------------------------------------------------*/
void Matrixprint(const MatrixXd Mat, const string name);
void Vectorprint(const VectorXd Mat, const string name);

/*-----------------------------------------------
    Calibration Functions (Accelerometer & Gyroscope Bias)
------------------------------------------------*/
void CalAvgAcc_Gyr(IMUDataEpoch rawdata, double* epochnum, double* accmean, double* gyrmean);
void AccCalibration(double accmean[3], IMUDataEpoch* rawdata);
void GyrCalibration(double gyrmean[3], IMUDataEpoch* rawdata);

/*-----------------------------------------------
    Quaternion, Matrix, and Auxiliary Math Functions
------------------------------------------------*/
void QuaternionMultiply(Quater p, Quater q, Quater* result);
void CalPostureWithQuaternion(Quater Q, INSDataEpoch* result);
void SkewSymmetricMatrix(Vector3d vec, Matrix3d* result);
void Calgpn(POSITION blh, Vector3d* gpn);
void Extrapolation(double tk, double tk_1, double tk_2, 
    VELOCITY vel_prv, VELOCITY vel_pprv, 
    POSITION blh_prv, POSITION blh_pprv, 
    Vector3d* w_ie_n, Vector3d* w_en_n, Vector3d* v_k_12);

/*-----------------------------------------------
    INS Mechanization Algorithm Functions
------------------------------------------------*/
bool PostureUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, INSDataEpoch result_prv, Quater result_k_1, Quater* result_k, INSDataEpoch* result_cur);
bool VelocityUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur);
bool PositionUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur);

/*-----------------------------------------------
    Coordinate Transformation Functions
------------------------------------------------*/
void BLHToXYZ(POSITION BLH, double XYZ[3], const double R, const double F);
void XYZToBLH(double XYZ[3], POSITION* BLH, const double R, const double F);
void BlhToNeuMat(POSITION* Blh, Matrix3d& Mat);
void CompEnudPos(const double Xs[], const double Xr[], POSITION* Blh, dENU* dENU);