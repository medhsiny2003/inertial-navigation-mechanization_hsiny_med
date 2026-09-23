// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Field: Electrical Engineering & Industrial Control Student
// Institution: Faculty of Sciences and Techniques of Mohammedia
// Year: 2026
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