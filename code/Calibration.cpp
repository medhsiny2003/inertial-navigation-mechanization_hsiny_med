// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Field: Electrical Engineering & Industrial Control Student
// Institution: Faculty of Sciences and Techniques of Mohammedia
// Year: 2026
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

/* Compute Average Accelerometer and Gyroscope Outputs */
// Input: raw data rawdata
// Input: epoch counter epochnum (initially 0)
// Output: accmean (average specific force)
// Output: gyrmean (average angular velocity)
void CalAvgAcc_Gyr(IMUDataEpoch rawdata, double* epochnum, double* accmean, double* gyrmean)
{
	// Increment epoch counter
	(*epochnum)++;

	// Accumulate specific force
	accmean[0] = accmean[0] * (*epochnum - 1) + rawdata.Acc.X;
	accmean[1] = accmean[1] * (*epochnum - 1) + rawdata.Acc.Y;
	accmean[2] = accmean[2] * (*epochnum - 1) + rawdata.Acc.Z;

	// Accumulate angular velocity
	gyrmean[0] = gyrmean[0] * (*epochnum - 1) + rawdata.Gyr.X;
	gyrmean[1] = gyrmean[1] * (*epochnum - 1) + rawdata.Gyr.Y;
	gyrmean[2] = gyrmean[2] * (*epochnum - 1) + rawdata.Gyr.Z;

	// Compute running averages
	for (int i = 0; i < 3; i++)
	{
		accmean[i] /= (*epochnum);
		gyrmean[i] /= (*epochnum);
	}
}

/* Accelerometer Zero Bias Calibration */
// Input: average specific force accmean
// Input/Output: raw data rawdata (bias compensated)
void AccCalibration(double accmean[3], IMUDataEpoch* rawdata)
{
	double biasAcc[3] = { 0.0, 0.0, 0.0 };
	biasAcc[0] = accmean[0];
	biasAcc[1] = accmean[1];
	// With SI units, accmean[2] ≈ -gravity when stationary.
	// The bias is therefore accmean[2] + gravity ≈ 0.
	biasAcc[2] = accmean[2] + gravity;

	rawdata->Acc.X -= biasAcc[0];
	rawdata->Acc.Y -= biasAcc[1];
	rawdata->Acc.Z -= biasAcc[2];
}

/* Gyroscope Zero Bias Calibration */
// Input: average angular velocity gyrmean
// Input/Output: raw data rawdata (bias compensated)
void GyrCalibration(double gyrmean[3], IMUDataEpoch* rawdata)
{
	double i1 = we * cos(ours_initial_latitude) * cos(ours_initial_yaw) / 100.0;
	double i2 = we * cos(ours_initial_latitude) * sin(ours_initial_yaw) / 100.0;
	double i3 = we * sin(ours_initial_latitude) / 100.0;
	double biasGyr[3] = { 0.0, 0.0, 0.0 };
	biasGyr[0] = gyrmean[0];
	biasGyr[1] = gyrmean[1];
	biasGyr[2] = gyrmean[2];

	biasGyr[0] -= i1;
	biasGyr[1] -= i2;
	biasGyr[2] -= i3;
	rawdata->Gyr.X -= biasGyr[0];
	rawdata->Gyr.Y -= biasGyr[1];
	rawdata->Gyr.Z -= biasGyr[2];
}