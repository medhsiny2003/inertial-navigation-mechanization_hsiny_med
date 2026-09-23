// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Field: Electrical Engineering & Industrial Control Student
// Institution: Faculty of Sciences and Techniques of Mohammedia
// Year: 2026
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

/*-----------------------------------------------
    Geodetic to Cartesian Coordinates (BLH -> XYZ)
------------------------------------------------*/
void BLHToXYZ(POSITION BLH, double XYZ[3], const double R, const double F)
{
    double a_cor = R;                                               // Semi-major axis
    double f = F;                                                   // Flattening
    double b_cor = a_cor * (1.0 - f);                               // Semi-minor axis
    double e2 = (a_cor * a_cor - b_cor * b_cor) / (a_cor * a_cor);  // First eccentricity squared

    double L = BLH.longitude;   // Longitude (rad)
    double B = BLH.latitude;    // Latitude (rad)
    double H = BLH.H;           // Height (m)

    double sinB = sin(B);
    double cosB = cos(B);
    double sinL = sin(L);
    double cosL = cos(L);

    double N = a_cor / sqrt(1.0 - e2 * sinB * sinB); // Prime vertical radius of curvature

    XYZ[0] = (N + H) * cosB * cosL;
    XYZ[1] = (N + H) * cosB * sinL;
    XYZ[2] = (N * (1.0 - e2) + H) * sinB;
}

/*-----------------------------------------------
    Cartesian to Geodetic Coordinates (XYZ -> BLH)
------------------------------------------------*/
void XYZToBLH(double XYZ[3], POSITION* BLH, const double R, const double F)
{
    double a_cor = R;
    double b_cor = a_cor * (1.0 - F);
    double e2 = (a_cor * a_cor - b_cor * b_cor) / (a_cor * a_cor);

    double x = XYZ[0];
    double y = XYZ[1];
    double z = XYZ[2];
    double p = sqrt(x * x + y * y);

    double B = atan2(z, p * (1.0 - e2)); // Initial latitude (rad)
    double H = 0.0;

    int MaxCount = 1000;
    double Threshold = 1e-12;

    int count = 0;
    double delta = 0.0;

    do {
        double sinB = sin(B);
        double N = a_cor / sqrt(1.0 - e2 * sinB * sinB);
        H = p / cos(B) - N;

        double B_new = atan2(z + e2 * (a_cor / sqrt(1.0 - e2 * sinB * sinB)) * sinB, p);
        delta = B - B_new;
        B = B_new;
        count++;
    } while (count < MaxCount && std::fabs(delta) > Threshold);

    if (count >= MaxCount) {
        std::cerr << "Warning: Did not converge within " << MaxCount
            << " iterations (error: " << std::fabs(delta)
            << " rad)" << std::endl;
    }

    BLH->latitude = B * Deg;           // Latitude (deg)
    BLH->longitude = atan2(y, x) * Deg; // Longitude (deg)
    BLH->H = H;                         // Height (m)
}

/*-----------------------------------------------
    Generate BLH -> NEU Rotation Matrix
------------------------------------------------*/
void BlhToNeuMat(POSITION* Blh, Matrix3d& Mat)
{
    double B = Blh->latitude;
    double L = Blh->longitude;

    double sinB = sin(B), cosB = cos(B);
    double sinL = sin(L), cosL = cos(L);

    Mat << -sinL, cosL, 0.0,
        -sinB * cosL, -sinB * sinL, cosB,
        cosB * cosL, cosB * sinL, sinB;
}

/*-----------------------------------------------
    Compute ENU Position Error
------------------------------------------------*/
void CompEnudPos(const double Xs[], const double Xr[], POSITION* Blh, dENU* denu)
{
    Vector3d Delta;
    Delta << Xs[0] - Xr[0], Xs[1] - Xr[1], Xs[2] - Xr[2];

    Matrix3d R;
    BlhToNeuMat(Blh, R);  // Rotation matrix to E/N/U

    Vector3d d_neu = R * Delta;

    denu->dE = d_neu(0); // East error
    denu->dN = d_neu(1); // North error
    denu->dU = d_neu(2); // Up error
}