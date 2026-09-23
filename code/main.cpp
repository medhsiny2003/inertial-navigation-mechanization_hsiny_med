// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Title: Élève ingénieur en génie électrique et contrôle industriel
// Based on the original project by GYH-WHU
// Year: 2025
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

int main(int argc, char* argv[])
{
    int mode = 1;   // Solution mode: 0 = ExampleData, 1 = SelfData
    cout << "============================================================\n";
    cout << "INS Mechanization - Inertial Navigation System\n";
    cout << "Author: Mohammed Hsiny (2025)\n";
    cout << "Repository: inertial-navigation-mechanization_hsiny_med\n";
    cout << "============================================================\n";

    if (argc > 1) {
        mode = atoi(argv[1]);
        cout << "[INFO] Mode selected from argument: " << mode << "\n";
    } else {
        cout << "Please choose the mode of solution:\n  0 ExampleData\n  1 SelfData\n";
        cout << "Enter mode (0 or 1): ";
        string mode_input;
        if (cin >> mode_input) {
            mode = atoi(mode_input.c_str());
        } else {
            mode = 1; // Default to 1 (SelfData / Simulation)
            cout << "\n[INFO] Defaulting to mode 1 (SelfData)\n";
        }
    }
    if (!(mode == 0 || mode == 1)) {
        cout << "Error: Invalid solution mode selected!\n";
        return 1;
    }

    switch (mode)
    {
    case 0:
    {
        cout << "\n[INFO] Starting solution for Example Data. Processing...\n";

        // Open input and output files
        FILE* m_imu = fopen("Data/Example_Data/IMU.bin", "rb");
        FILE* m_ins = fopen("Data/Example_Data/PureIns.bin", "rb");

        FILE* result_example = fopen("Result/ExampleResult/INS_Result.txt", "wb");
        FILE* true_example = fopen("Result/ExampleResult/INS_True.txt", "wb");
        FILE* diff_result_example = fopen("Result/ExampleResult/INS_Diff_Result.txt", "wb");
        FILE* denu_result = fopen("Result/ExampleResult/denu_Result.txt", "wb");

        if (!m_imu) { cerr << "Error: Cannot open Data/Example_Data/IMU.bin!\n"; return -1; }
        if (!m_ins) { cerr << "Error: Cannot open Data/Example_Data/PureIns.bin!\n"; return -1; }
        if (!result_example) { cerr << "Error: Cannot create Result/ExampleResult/INS_Result.txt!\n"; return -1; }
        if (!true_example) { cerr << "Error: Cannot create Result/ExampleResult/INS_True.txt!\n"; return -1; }
        if (!diff_result_example) { cerr << "Error: Cannot create Result/ExampleResult/INS_Diff_Result.txt!\n"; return -1; }
        if (!denu_result) { cerr << "Error: Cannot create Result/ExampleResult/denu_Result.txt!\n"; return -1; }

        fprintf(result_example, "# TimeStamp,BLH(deg),Vel_xyz(m/s),Pos_roll_pitch_yaw(deg)\n");
        fprintf(true_example, "# TimeStamp,BLH(deg),Vel_xyz(m/s),Pos_roll_pitch_yaw(deg)\n");
        fprintf(diff_result_example, "# TimeStamp,dVel(m/s),dBlh(deg),dPos_roll_pitch_yaw(deg)\n");
        fprintf(denu_result, "# TimeStamp,truedata dneu(m),ourdata dneu(m)\n");

        IMUDataEpoch mImuDataEpoch_prv, mImuDataEpoch_pprv;
        INSDataEpoch result_prv;
        INSDataEpoch result_pprv;
        Quater Q_prv;

        POSITION ZeroPoint;
        double basexyz[3] = { 0.0, 0.0, 0.0 };
        ZeroPoint.latitude = initial_latitude;
        ZeroPoint.longitude = initial_longitude;
        ZeroPoint.H = initial_elevation;
        BLHToXYZ(ZeroPoint, basexyz, R_WGS84, F_WGS84);

        bool flag_imu = true;
        bool Is_prv = false;
        bool Is_pprv = false;

        double caltime = 0.0;
        int calnum = 0;

        while (flag_imu)
        {
            IMUDataEpoch mImuDataEpoch_cur;
            INSDataEpoch mInsDataEpoch_cur;
            INSDataEpoch result_cur;
            Quater Q_cur;

            flag_imu = ReadExamplePureIMUData(m_imu, &mImuDataEpoch_cur);
            if (!flag_imu) break;
            if (mImuDataEpoch_cur.TimeStamp < 91619.995) continue;

            if (!Is_prv)
            {
                mImuDataEpoch_prv = mImuDataEpoch_cur;
                Is_prv = true;
                continue;
            }
            else if (!Is_pprv)
            {
                mImuDataEpoch_pprv = mImuDataEpoch_prv;
                mImuDataEpoch_prv = mImuDataEpoch_cur;
                Is_pprv = true;

                result_prv.blh.latitude = initial_latitude;
                result_prv.blh.longitude = initial_longitude;
                result_prv.blh.H = initial_elevation;

                result_pprv.blh.latitude = initial_latitude;
                result_pprv.blh.longitude = initial_longitude;
                result_pprv.blh.H = initial_elevation;

                result_prv.pos.roll = initial_roll;
                result_prv.pos.pitch = initial_pitch;
                result_prv.pos.yaw = initial_heading;
                Q_prv.SetQbn(result_prv.pos);
                continue;
            }

            caltime += (mImuDataEpoch_cur.TimeStamp - mImuDataEpoch_prv.TimeStamp);
            if (caltime > 600.0)
            {
                caltime = 0.0;
                calnum += 1;
                printf("[INFO] Processed %d0 minutes of example data.\n", calnum);
            }

            ReadExamplePureINSData(m_ins, &mInsDataEpoch_cur);

            // 1. Velocity Update
            VelocityUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv, mImuDataEpoch_pprv,
                result_prv, result_pprv, &result_cur);

            // 2. Position Update
            PositionUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv, mImuDataEpoch_pprv,
                result_prv, result_pprv, &result_cur);

            double blh[3] = { 0.0, 0.0, 0.0 };
            blh[0] = result_cur.blh.latitude * Deg;
            blh[1] = result_cur.blh.longitude * Deg;
            blh[2] = result_cur.blh.H;

            // 3. Attitude Update
            PostureUpdate(mImuDataEpoch_cur, mImuDataEpoch_prv,
                result_prv, Q_prv, &Q_cur, &result_cur);

            double POS[3] = { 0.0, 0.0, 0.0 };
            POS[0] = result_cur.pos.roll * Deg;
            POS[1] = result_cur.pos.pitch * Deg;
            POS[2] = result_cur.pos.yaw * Deg;

            SaveOurResult(result_example, mImuDataEpoch_cur, result_cur.vel, result_cur.blh, result_cur.pos);

            if (mInsDataEpoch_cur.pos.yaw < 0) mInsDataEpoch_cur.pos.yaw += 360;
            SaveTrueResult(true_example, mInsDataEpoch_cur);

            // Difference / Error
            double dvel[3] = { result_cur.vel.Vn - mInsDataEpoch_cur.vel.Vn, result_cur.vel.Ve - mInsDataEpoch_cur.vel.Ve, result_cur.vel.Vd - mInsDataEpoch_cur.vel.Vd };
            double dblh[3] = { blh[0] - mInsDataEpoch_cur.blh.latitude, blh[1] - mInsDataEpoch_cur.blh.longitude, blh[2] - mInsDataEpoch_cur.blh.H };
            if (POS[2] < 0) POS[2] += 360;
            double dpos[3] = { POS[0] - mInsDataEpoch_cur.pos.roll, POS[1] - mInsDataEpoch_cur.pos.pitch, POS[2] - mInsDataEpoch_cur.pos.yaw };
            if (fabs(mInsDataEpoch_cur.TimeStamp - mInsDataEpoch_cur.TimeStamp) < 1e-3)
                SaveDiffResult(diff_result_example, mImuDataEpoch_cur, dvel, dblh, dpos);

            // Trajectory in ENU
            double truexyz[3] = { 0.0, 0.0, 0.0 };
            double ourxyz[3] = { 0.0, 0.0, 0.0 };
            dENU truedenu, ourdenu;
            mInsDataEpoch_cur.blh.latitude *= Rad;
            mInsDataEpoch_cur.blh.longitude *= Rad;

            BLHToXYZ(mInsDataEpoch_cur.blh, truexyz, R_WGS84, F_WGS84);
            BLHToXYZ(result_cur.blh, ourxyz, R_WGS84, F_WGS84);
            CompEnudPos(basexyz, truexyz, &ZeroPoint, &truedenu);
            CompEnudPos(basexyz, ourxyz, &ZeroPoint, &ourdenu);

            SavedENUResult(denu_result, mImuDataEpoch_cur, truedenu, ourdenu);

            mImuDataEpoch_pprv = mImuDataEpoch_prv;
            mImuDataEpoch_prv = mImuDataEpoch_cur;

            result_pprv = result_prv;
            result_prv = result_cur;
            Q_prv = Q_cur;
        }

        cout << "[OK] Example data solution complete. Results saved in Result/ExampleResult/\n";
        fclose(m_imu);
        fclose(m_ins);
        fclose(result_example);
        fclose(true_example);
        fclose(diff_result_example);
        fclose(denu_result);
        break;
    }
    case 1:
    {
        cout << "\n[INFO] Starting solution for SelfData / Simulated Data. Processing...\n";

        string ourdata_file_path = "Data/GroupOne.ASC";
        string truth_file_path = "Data/TruthOne.nav";

        ifstream ourdata_file(ourdata_file_path);
        ifstream truth_file(truth_file_path);
        if (!ourdata_file) {
            ourdata_file_path = "Data\\GroupOne.ASC";
            truth_file_path = "Data\\TruthOne.nav";
            ourdata_file.open(ourdata_file_path);
            truth_file.open(truth_file_path);
        }
        if (!ourdata_file) { cerr << "Error: Cannot open " << ourdata_file_path << "!\n"; return -1; }
        if (!truth_file) { cerr << "Error: Cannot open " << truth_file_path << "!\n"; return -1; }

        FILE* result_file = fopen("Result/result.txt", "wb");
        if (!result_file) { cerr << "Error: Cannot create Result/result.txt!\n"; return -1; }
        fprintf(result_file, "# TimeStamp,BLH(deg),Vel(m/s),Pos(deg)\n");
        
        FILE* true_file = fopen("Result/true.txt", "wb");
        if (!true_file) { cerr << "Error: Cannot create Result/true.txt!\n"; return -1; }
        fprintf(true_file, "# TimeStamp,BLH(deg),Vel(m/s),Pos(deg)\n");

        FILE* diff_result_file = fopen("Result/diff_result.txt", "wb");
        if (!diff_result_file) { cerr << "Error: Cannot create Result/diff_result.txt!\n"; return -1; }
        fprintf(diff_result_file, "# TimeStamp,dVel(m/s),dBLH(deg),dPos(deg)\n");

        FILE* denu_result_file = fopen("Result/denu_result.txt", "wb");
        if (!denu_result_file) { cerr << "Error: Cannot create Result/denu_result.txt!\n"; return -1; }
        fprintf(denu_result_file, "# TimeStamp,truedata dneu(m),ourdata dneu(m)\n");

        IMUDataEpoch ourdata_prv;
        IMUDataEpoch ourdata_pprv;
        INSDataEpoch result_prv;
        INSDataEpoch result_pprv;
        Quater Q_prv;

        INSDataEpoch truedata;
        TimeIntervalsArray ZeroSpeed;

        POSITION ZeroPoint;
        double basexyz[3] = { 0.0, 0.0, 0.0 };

        bool Is_prv = false;
        bool Is_pprv = false;
        bool flag_imu = true;
        bool Is_Cali = false;
        bool Is_Zero = false;

        double epochnum = 0.0;
        double accmean[3] = { 0.0, 0.0, 0.0 };
        double gyrmean[3] = { 0.0, 0.0, 0.0 };

        double caltime = 0.0;
        int calnum = 0;

        // Peak initial reference truth to establish starting coordinate reference
        bool first_truth_read = ReadTruthData(truth_file, &truedata);
        double actual_start_time = truedata.TimeStamp;
        double sim_initial_lat = truedata.blh.latitude;
        double sim_initial_lon = truedata.blh.longitude;
        double sim_initial_h = truedata.blh.H;
        double sim_initial_roll = truedata.pos.roll;
        double sim_initial_pitch = truedata.pos.pitch;
        double sim_initial_yaw = truedata.pos.yaw;

        // If coordinates are in degrees (e.g. > 2*pi), convert to rad for internal computation
        if (fabs(sim_initial_lat) > 2 * PAI) sim_initial_lat *= Rad;
        if (fabs(sim_initial_lon) > 2 * PAI) sim_initial_lon *= Rad;
        if (fabs(sim_initial_roll) > 2 * PAI) sim_initial_roll *= Rad;
        if (fabs(sim_initial_pitch) > 2 * PAI) sim_initial_pitch *= Rad;
        if (fabs(sim_initial_yaw) > 2 * PAI) sim_initial_yaw *= Rad;

        ZeroPoint.latitude = sim_initial_lat;
        ZeroPoint.longitude = sim_initial_lon;
        ZeroPoint.H = sim_initial_h;
        BLHToXYZ(ZeroPoint, basexyz, R_WGS84, F_WGS84);

        double solve_start_time = (actual_start_time > 1000.0) ? starttime : actual_start_time;
        double solve_end_time = (actual_start_time > 1000.0) ? endtime : 1e9;

        while (flag_imu)
        {
            IMUDataEpoch ourdata_cur;
            INSDataEpoch result_cur;
            Quater Q_cur;

            DeviceType PureIMU_One = CGI;
            flag_imu = ReadIMURawData_CGI(ourdata_file, &ourdata_cur, PureIMU_One);
            if (!flag_imu) break;
            if (ourdata_cur.TimeStamp > solve_end_time) break;

            if (ourdata_cur.TimeStamp < solve_start_time - 0.02)
            {
                CalAvgAcc_Gyr(ourdata_cur, &epochnum, accmean, gyrmean);
                continue;
            }

            if (Is_Cali)
            {
                AccCalibration(accmean, &ourdata_cur);
            }

            if (!Is_prv)
            {
                ourdata_prv = ourdata_cur;
                Is_prv = true;
                continue;
            }
            else if (!Is_pprv)
            {
                ourdata_pprv = ourdata_prv;
                ourdata_prv = ourdata_cur;
                Is_pprv = true;

                result_prv.blh.latitude = sim_initial_lat;
                result_prv.blh.longitude = sim_initial_lon;
                result_prv.blh.H = sim_initial_h;

                result_pprv.blh.latitude = sim_initial_lat;
                result_pprv.blh.longitude = sim_initial_lon;
                result_pprv.blh.H = sim_initial_h;

                result_prv.vel.Vn = truedata.vel.Vn;
                result_prv.vel.Ve = truedata.vel.Ve;
                result_prv.vel.Vd = truedata.vel.Vd;

                result_prv.pos.roll = sim_initial_roll;
                result_prv.pos.pitch = sim_initial_pitch;
                result_prv.pos.yaw = sim_initial_yaw;

                Q_prv.SetQbn(result_prv.pos);
                continue;
            }

            caltime += (ourdata_cur.TimeStamp - ourdata_prv.TimeStamp);
            if (caltime >= 600.0)
            {
                caltime = 0.0;
                calnum += 1;
                printf("[INFO] Processed %d0 minutes of self data.\n", calnum);
            }

            // Sync truth data to current timestamp
            while (truedata.TimeStamp < ourdata_cur.TimeStamp - 0.005)
            {
                if (!ReadTruthData(truth_file, &truedata)) break;
            }

            result_cur.TimeStamp = ourdata_cur.TimeStamp;

            // 1. Update Velocity
            VelocityUpdate(ourdata_cur, ourdata_prv, ourdata_pprv,
                result_prv, result_pprv, &result_cur);

            if (Is_Zero)
            {
                for (int i = 0; i < zero_time_intervals_num; i++)
                {
                    if (ourdata_cur.TimeStamp > ZeroSpeed.getInterval(i).end)
                    {
                        ZeroSpeed.getInterval(i).used = true;
                        continue;
                    }

                    if (i < zero_time_intervals_num - 1 &&
                        ourdata_cur.TimeStamp > ZeroSpeed.getInterval(i).end &&
                        ourdata_cur.TimeStamp < ZeroSpeed.getInterval(i + 1).start)
                        break;

                    if (ourdata_cur.TimeStamp >= ZeroSpeed.getInterval(i).start &&
                        ourdata_cur.TimeStamp <= ZeroSpeed.getInterval(i).end)
                    {
                        result_cur.vel.Vn = 0.0;
                        result_cur.vel.Ve = 0.0;
                        result_cur.vel.Vd = 0.0;
                        break;
                    }
                }
            }

            // 2. Update Position
            PositionUpdate(ourdata_cur, ourdata_prv, ourdata_pprv,
                result_prv, result_pprv, &result_cur);

            double blh[3] = { 0.0, 0.0, 0.0 };
            blh[0] = result_cur.blh.latitude * Deg;
            blh[1] = result_cur.blh.longitude * Deg;
            blh[2] = result_cur.blh.H;

            // 3. Update Attitude
            PostureUpdate(ourdata_cur, ourdata_prv,
                result_prv, Q_prv, &Q_cur, &result_cur);

            double POS[3] = { 0.0, 0.0, 0.0 };
            POS[0] = result_cur.pos.roll * Deg;
            POS[1] = result_cur.pos.pitch * Deg;
            POS[2] = result_cur.pos.yaw * Deg;
            if (POS[2] < 0) POS[2] += 360.0;

            // Save results
            if (fabs(ourdata_cur.TimeStamp - truedata.TimeStamp) < 1e-2)
            {
                SaveOurResult(result_file, ourdata_cur, result_cur.vel, result_cur.blh, result_cur.pos);
                SaveTrueResult(true_file, truedata);

                double true_lat = (truedata.blh.latitude > 2 * PAI || truedata.blh.latitude < -2 * PAI) ? truedata.blh.latitude : (truedata.blh.latitude * Deg);
                double true_lon = (truedata.blh.longitude > 2 * PAI || truedata.blh.longitude < -2 * PAI) ? truedata.blh.longitude : (truedata.blh.longitude * Deg);
                double true_roll = (truedata.pos.roll > 2 * PAI || truedata.pos.roll < -2 * PAI) ? truedata.pos.roll : (truedata.pos.roll * Deg);
                double true_pitch = (truedata.pos.pitch > 2 * PAI || truedata.pos.pitch < -2 * PAI) ? truedata.pos.pitch : (truedata.pos.pitch * Deg);
                double true_yaw = (truedata.pos.yaw > 2 * PAI || truedata.pos.yaw < -2 * PAI) ? truedata.pos.yaw : (truedata.pos.yaw * Deg);
                if (true_yaw < 0.0) true_yaw += 360.0;

                double dvel[3] = { result_cur.vel.Vn - truedata.vel.Vn, result_cur.vel.Ve - truedata.vel.Ve, result_cur.vel.Vd - truedata.vel.Vd };
                double dblh[3] = { blh[0] - true_lat, blh[1] - true_lon, blh[2] - truedata.blh.H };
                double dpos[3] = { POS[0] - true_roll, POS[1] - true_pitch, POS[2] - true_yaw };

                if (dpos[2] > 180.0) dpos[2] -= 360.0;
                if (dpos[2] < -180.0) dpos[2] += 360.0;
                SaveDiffResult(diff_result_file, ourdata_cur, dvel, dblh, dpos);
            }

            // Save trajectory in ENU coordinates
            double truexyz[3] = { 0.0, 0.0, 0.0 };
            double ourxyz[3] = { 0.0, 0.0, 0.0 };
            dENU truedenu, ourdenu;

            POSITION trueloc = truedata.blh;
            if (fabs(trueloc.latitude) > 2 * PAI) trueloc.latitude *= Rad;
            if (fabs(trueloc.longitude) > 2 * PAI) trueloc.longitude *= Rad;

            BLHToXYZ(trueloc, truexyz, R_WGS84, F_WGS84);
            BLHToXYZ(result_cur.blh, ourxyz, R_WGS84, F_WGS84);
            CompEnudPos(basexyz, truexyz, &ZeroPoint, &truedenu);
            CompEnudPos(basexyz, ourxyz, &ZeroPoint, &ourdenu);

            SavedENUResult(denu_result_file, ourdata_cur, truedenu, ourdenu);

            ourdata_pprv = ourdata_prv;
            ourdata_prv = ourdata_cur;

            result_pprv = result_prv;
            result_prv = result_cur;
            Q_prv = Q_cur;
        }

        cout << "[OK] Self data solution complete. Results saved in Result/\n";
        ourdata_file.close();
        truth_file.close();
        fclose(result_file);
        fclose(true_file);
        fclose(diff_result_file);
        fclose(denu_result_file);
        break;
    }
    }

    return 0;
}