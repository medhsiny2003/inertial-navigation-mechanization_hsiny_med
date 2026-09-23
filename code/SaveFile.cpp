// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Field: Electrical Engineering & Industrial Control Student
// Institution: Faculty of Sciences and Techniques of Mohammedia
// Year: 2026
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

/* Save example data in text format */
// Input: mode 0 for raw IMU data, mode 1 for reference INS data
void SaveExampleData(FILE* OutputFile, IMUDataEpoch mImu, INSDataEpoch mIns, int mode)
{
	if (mode == 0) fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		mImu.Acc.X, mImu.Acc.Y, mImu.Acc.Z,
		mImu.Gyr.X, mImu.Gyr.Y, mImu.Gyr.Z);
	if (mode == 1) fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mIns.TimeStamp,
		mIns.blh.latitude, mIns.blh.longitude, mIns.blh.H,
		mIns.vel.Vn, mIns.vel.Ve, mIns.vel.Vd,
		mIns.pos.roll, mIns.pos.pitch, mIns.pos.yaw);
}

/* Save our mechanization results */
void SaveOurResult(FILE* OutputFile, IMUDataEpoch mImu, VELOCITY vel, POSITION blh, POSTURE pos)
{
	double Yaw = pos.yaw * Deg;
	if (Yaw < 0.0) Yaw += 360.0;

	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		blh.latitude * Deg, blh.longitude * Deg, blh.H,
		vel.Vn, vel.Ve, vel.Vd,
		pos.roll * Deg, pos.pitch * Deg, Yaw);
}

/* Save reference ground truth results */
void SaveTrueResult(FILE* OutputFile, INSDataEpoch mIns)
{
	double lat = (mIns.blh.latitude > 2 * PAI || mIns.blh.latitude < -2 * PAI) ? mIns.blh.latitude : (mIns.blh.latitude * Deg);
	double lon = (mIns.blh.longitude > 2 * PAI || mIns.blh.longitude < -2 * PAI) ? mIns.blh.longitude : (mIns.blh.longitude * Deg);
	double roll = (mIns.pos.roll > 2 * PAI || mIns.pos.roll < -2 * PAI) ? mIns.pos.roll : (mIns.pos.roll * Deg);
	double pitch = (mIns.pos.pitch > 2 * PAI || mIns.pos.pitch < -2 * PAI) ? mIns.pos.pitch : (mIns.pos.pitch * Deg);
	double yaw = (mIns.pos.yaw > 2 * PAI || mIns.pos.yaw < -2 * PAI) ? mIns.pos.yaw : (mIns.pos.yaw * Deg);
	if (yaw < 0.0) yaw += 360.0;

	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mIns.TimeStamp,
		lat, lon, mIns.blh.H,
		mIns.vel.Vn, mIns.vel.Ve, mIns.vel.Vd,
		roll, pitch, yaw);
}

/* Save difference / error results */
void SaveDiffResult(FILE* OutputFile, IMUDataEpoch mImu, double dvel[3], double dblh[3], double dpos[3])
{
	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		dvel[0], dvel[1], dvel[2],
		dblh[0], dblh[1], dblh[2],
		dpos[0], dpos[1], dpos[2]);
}

/* Save dENU trajectory results */
void SavedENUResult(FILE* OutputFile, IMUDataEpoch mImu, dENU truedata, dENU ourdata)
{
	fprintf(OutputFile, "%0.15f %0.15f %0.15f %0.15f %0.15f %0.15f %0.15f\n", mImu.TimeStamp,
		truedata.dN, truedata.dE, truedata.dU,
		ourdata.dN, ourdata.dE, ourdata.dU);
}