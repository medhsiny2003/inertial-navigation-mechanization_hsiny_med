// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Field: Electrical Engineering & Industrial Control Student
// Institution: Faculty of Sciences and Techniques of Mohammedia
// Year: 2026
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

/* Attitude Update */
// Input: Current raw data pos_cur, previous raw data pos_prv
// Input: Previous INS state result_prv, previous quaternion result_k_1
// Output: Updated quaternion result_k, updated INS state result_cur
bool PostureUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, INSDataEpoch result_prv, Quater result_k_1, Quater* result_k, INSDataEpoch* result_cur)
{
	// 1. Equivalent rotation vector update in body frame (b-frame)
	Vector3d q_cur(pos_cur.Gyr.X, pos_cur.Gyr.Y, pos_cur.Gyr.Z);	// Angular increment (rad)
	Vector3d q_prv(pos_prv.Gyr.X, pos_prv.Gyr.Y, pos_prv.Gyr.Z);

	Vector3d ERV_b = q_cur + q_prv.cross(q_cur) / 12.0;	// b-frame equivalent rotation vector (rad)
	POSTURE EqualRotaionVec_b;
	EqualRotaionVec_b.roll = ERV_b[0];
	EqualRotaionVec_b.pitch = ERV_b[1]; 
	EqualRotaionVec_b.yaw = ERV_b[2];

	Quater qbkk_1;	// Body frame quaternion from k-1 to k
	qbkk_1.Setkk_1(EqualRotaionVec_b);

	// 2. Equivalent rotation vector update in navigation frame (n-frame)
	double deltaT = pos_cur.TimeStamp - pos_prv.TimeStamp;	// Time step
	Vector3d w_ie_n(we * cos(result_prv.blh.latitude), 0, -we * sin(result_prv.blh.latitude));	// Earth rate in n-frame
	
	double R_M = a * (1 - pow(e, 2)) / sqrt(pow(1 - pow(e * sin(result_prv.blh.latitude), 2), 3));
	double R_N = a / sqrt(1 - pow(e * sin(result_prv.blh.latitude), 2));
	Vector3d w_en_n(result_prv.vel.Ve / (R_N + result_prv.blh.H), -result_prv.vel.Vn / (R_M + result_prv.blh.H), -result_prv.vel.Ve * tan(result_prv.blh.latitude) / (R_N + result_prv.blh.H));

	Vector3d ERV_n = (w_ie_n + w_en_n) * deltaT;
	POSTURE EqualRotationVec_n;	// n-frame equivalent rotation vector
	EqualRotationVec_n.roll = ERV_n[0];
	EqualRotationVec_n.pitch = ERV_n[1];
	EqualRotationVec_n.yaw = ERV_n[2];

	Quater qnk_1k;	// Navigation frame quaternion
	qnk_1k.Setk_1k(EqualRotationVec_n);

	// 3. Compute current attitude quaternion
	Quater middle;
	QuaternionMultiply(qnk_1k, result_k_1, &middle);
	QuaternionMultiply(middle, qbkk_1, result_k);

	// 4. Normalize attitude quaternion
	double Rou = sqrt(pow(result_k->q[0], 2) + pow(result_k->q[1], 2) + pow(result_k->q[2], 2) + pow(result_k->q[3], 2));
	for (int i = 0; i < 4; i++) { result_k->q[i] = result_k->q[i] / Rou; }

	// 5. Extract Euler angles from quaternion
	CalPostureWithQuaternion(*result_k, result_cur);

	return true;
}

/* Velocity Update */
// Input: Current, previous, and second-previous raw IMU epochs
// Input: Previous and second-previous INS states
// Output: Updated velocity in result_cur
bool VelocityUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur)
{
	// 1. Calculate velocity increment vfk_bk_1 in body frame (sculling correction)
	Vector3d vk(pos_cur.Acc.X, pos_cur.Acc.Y, pos_cur.Acc.Z);	// Current velocity increment (m/s)
	Vector3d vk_1(pos_prv.Acc.X, pos_prv.Acc.Y, pos_prv.Acc.Z);	// Previous velocity increment (m/s)

	Vector3d thetak(pos_cur.Gyr.X, pos_cur.Gyr.Y, pos_cur.Gyr.Z);	// Current angle increment (rad)
	Vector3d thetak_1(pos_prv.Gyr.X, pos_prv.Gyr.Y, pos_prv.Gyr.Z);	// Previous angle increment (rad)

	Vector3d vfk_bk_1 = vk + 0.5 * thetak.cross(vk) + (thetak_1.cross(vk) + vk_1.cross(thetak)) / 12.0;

	// 2. Calculate zetak_1k rotation vector in navigation frame (extrapolated)
	double deltaT = pos_cur.TimeStamp - pos_prv.TimeStamp;
	Vector3d w_ie_n = Vector3d::Zero(), w_en_n = Vector3d::Zero(), v_k_12 = Vector3d::Zero();

	Extrapolation(pos_cur.TimeStamp, pos_prv.TimeStamp, pos_pprv.TimeStamp, result_prv.vel, result_pprv.vel, result_prv.blh, result_pprv.blh, &w_ie_n, &w_en_n, &v_k_12);
	Vector3d zetak_1k = (w_ie_n + w_en_n) * deltaT;

	// 3. Calculate specific force integral term vfkn
	CosineMatrix Cbn_k_1;
	Cbn_k_1.SetCbn(result_prv.pos);

	Matrix3d Skew_zetak_1k = Matrix3d::Zero(3, 3);
	SkewSymmetricMatrix(zetak_1k, &Skew_zetak_1k);

	Matrix3d I = Matrix3d::Identity();
	Vector3d vfkn = (I - 0.5 * Skew_zetak_1k) * Cbn_k_1.Cbn * vfk_bk_1;

	// 4. Calculate gravity and Coriolis integral terms vgcor_kn
	Vector3d gpn_k_1 = Vector3d::Zero(), gpn_k_2 = Vector3d::Zero(), gpn_k_12 = Vector3d::Zero();
	Calgpn(result_prv.blh, &gpn_k_1);
	Calgpn(result_pprv.blh, &gpn_k_2);
	double dt_prev = pos_prv.TimeStamp - pos_pprv.TimeStamp;
	if (dt_prev != 0.0)
	{
		gpn_k_12 = gpn_k_1 + deltaT / 2.0 * (gpn_k_1 - gpn_k_2) / dt_prev;
	}
	else
	{
		gpn_k_12 = gpn_k_1;
	}

	Vector3d vgcor_kn = (gpn_k_12 - (2 * w_ie_n + w_en_n).cross(v_k_12)) * deltaT;

	// 5. Update velocity vkn
	Vector3d vkn_1(result_prv.vel.Vn, result_prv.vel.Ve, result_prv.vel.Vd);
	Vector3d vkn = vkn_1 + vfkn + vgcor_kn;

	result_cur->vel.Vn = vkn[0];
	result_cur->vel.Ve = vkn[1];
	result_cur->vel.Vd = vkn[2];

	return true;
}

/* Position Update */
// Input: Current, previous, and second-previous raw IMU epochs
// Input: Current and previous velocities
// Input: Previous and second-previous positions
// Output: Updated position in result_cur
bool PositionUpdate(IMUDataEpoch pos_cur, IMUDataEpoch pos_prv, IMUDataEpoch pos_pprv, INSDataEpoch result_prv, INSDataEpoch result_pprv, INSDataEpoch* result_cur)
{
	// 1. Height update
	double deltaT = pos_cur.TimeStamp - pos_prv.TimeStamp;
	result_cur->blh.H = result_prv.blh.H - (result_cur->vel.Vd + result_prv.vel.Vd) / 2.0 * deltaT;

	// 2. Latitude update
	double R_M_k_1 = a * (1 - pow(e, 2)) / sqrt(pow(1 - pow(e * sin(result_prv.blh.latitude), 2), 3));
	double h_ba = (result_cur->blh.H + result_prv.blh.H) / 2.0;

	result_cur->blh.latitude = result_prv.blh.latitude + (result_cur->vel.Vn + result_prv.vel.Vn) / (2.0 * (R_M_k_1 + h_ba)) * deltaT;

	// 3. Longitude update
	double R_N_k_1 = a / sqrt(1 - pow(e * sin(result_prv.blh.latitude), 2));
	double R_N_k_2 = a / sqrt(1 - pow(e * sin(result_pprv.blh.latitude), 2));
	double dt_prev = pos_prv.TimeStamp - pos_pprv.TimeStamp;
	double R_N12 = R_N_k_1 + ((dt_prev != 0) ? (deltaT / 2.0 * (R_N_k_1 - R_N_k_2) / dt_prev) : 0.0);
	double fai_ba = (result_cur->blh.latitude + result_prv.blh.latitude) / 2.0;

	result_cur->blh.longitude = result_prv.blh.longitude + (result_cur->vel.Ve + result_prv.vel.Ve) / (2.0 * (R_N12 + h_ba) * cos(fai_ba)) * deltaT;

	return true;
}
