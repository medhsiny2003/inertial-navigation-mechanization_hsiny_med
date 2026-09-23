// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Title: Élève ingénieur en génie électrique et contrôle industriel
// Based on the original project by GYH-WHU
// Year: 2025
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

void Matrixprint(const MatrixXd Mat, const string name)
{
	cout << name << " (" << Mat.rows() << "x" << Mat.cols() << "):" << endl;
	IOFormat fmt(6, 0, " ", "\n", "", "", "", "");
	cout << Mat.format(fmt) << "\n" << "\n";
}

void Vectorprint(const VectorXd Mat, const string name)
{
	cout << name << " (" << Mat.rows() << "x" << Mat.cols() << "):" << endl;
	IOFormat fmt(6, 0, " ", "\n", "", "", "", "");
	cout << Mat.format(fmt) << "\n" << "\n";
}