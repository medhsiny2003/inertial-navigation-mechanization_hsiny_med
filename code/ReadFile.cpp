// ============================================================
// INS_Mechanization - Inertial Navigation System Mechanization
// Author: Mohammed Hsiny
// Field: Electrical Engineering & Industrial Control Student
// Institution: Faculty of Sciences and Techniques of Mohammedia
// Year: 2026
// Repository: inertial-navigation-mechanization_hsiny_med
// ============================================================

#include "IMU_Structs.h"

/**************************************/
/*          Example Data              */
/**************************************/
/* Read one epoch of pure INS example IMU data (binary) */
bool ReadExamplePureIMUData(FILE* file, IMUDataEpoch* mImuDataEpoch)
{
	if (!file) { cout << "Cannot open the Example Data file!" << endl; return false; }

	size_t readSize = fread(mImuDataEpoch, 8, 7, file);  // 7 double fields (8 bytes each)
	if (readSize != 7) { return false; }
	return true;
}

/* Read one epoch of pure INS example reference data (binary) */
bool ReadExamplePureINSData(FILE* file, INSDataEpoch* mInsResultEpoch)
{
	if (!file) { cout << "Cannot open the Example Data file!" << endl; return false; }

	size_t readSize = fread(mInsResultEpoch, 8, 10, file);  // 10 double fields (8 bytes each)
	if (readSize != 10) { return false; }
	return true;
}

/* Read IMU raw data (supports NovAtel ASCII logs and generic space-separated formats) */
bool ReadIMURawData_CGI(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device)
{
	double rate = 1.0, convert_a = 1.0, convert_g = 1.0;
	if (device == CGI) { convert_a = acc_CGI; convert_g = gyr_CGI; }
	else if (device == XWGI) { convert_a = acc_XWGI; convert_g = gyr_XWGI; }

	string line;
	while (getline(file, line))
	{
		if (line.empty()) continue;
		if (line[0] == '#' && line.rfind("#RAWIMUA", 0) != 0) continue; // Skip generic comment lines

		// Check for NovAtel #RAWIMUA format
		if (line.rfind("#RAWIMUA", 0) == 0)
		{
			stringstream ss(line);
			string part;
			getline(ss, part, ','); // #RAWIMUA
			getline(ss, part, ','); // COM1
			getline(ss, part, ','); // 0
			getline(ss, part, ','); // 113.0
			getline(ss, part, ','); // FINE
			getline(ss, part, ','); // Week
			getline(ss, part, ','); // Second of week
			Rawdata->TimeStamp = stod(part);

			getline(ss, part, ';'); // Skip rest of header
			getline(ss, part, ','); // Second week
			getline(ss, part, ','); // Second sec
			getline(ss, part, ','); // Status

			// Accelerometers
			getline(ss, part, ','); Rawdata->Acc.Z = -stod(part) * convert_a * rate;
			getline(ss, part, ','); Rawdata->Acc.X = -stod(part) * convert_a * rate;
			getline(ss, part, ','); Rawdata->Acc.Y = stod(part) * convert_a * rate;

			// Gyroscopes
			getline(ss, part, ','); Rawdata->Gyr.Z = -stod(part) * convert_g * rate * Rad;
			getline(ss, part, ','); Rawdata->Gyr.X = -stod(part) * convert_g * rate * Rad;
			getline(ss, part, ','); Rawdata->Gyr.Y = stod(part) * convert_g * rate * Rad;

			return true;
		}

		// Generic space-separated format: Time GyroX GyroY GyroZ AccX AccY AccZ
		stringstream ss(line);
		vector<string> parts;
		string part;
		while (ss >> part)
		{
			parts.push_back(part);
		}

		if (parts.size() >= 7)
		{
			Rawdata->TimeStamp = stod(parts[0]);
			Rawdata->Gyr.X = stod(parts[1]) * (1.0 / rate_CGI); // Angular increment
			Rawdata->Gyr.Y = stod(parts[2]) * (1.0 / rate_CGI);
			Rawdata->Gyr.Z = stod(parts[3]) * (1.0 / rate_CGI);
			Rawdata->Acc.X = stod(parts[4]) * (1.0 / rate_CGI); // Velocity increment
			Rawdata->Acc.Y = stod(parts[5]) * (1.0 / rate_CGI);
			Rawdata->Acc.Z = stod(parts[6]) * (1.0 / rate_CGI);
			return true;
		}
	}
	return false;
}

/* Read IMU raw data (alternative format) */
bool ReadIMURawData_Else(ifstream& file, IMUDataEpoch* Rawdata, DeviceType device)
{
	double rate = 1.0, convert_a = acc_XWGI, convert_g = gyr_XWGI;
	if (device != XWGI) return false;

	string line;
	if (getline(file, line))
	{
		if (line.empty()) return false;
		if (line[0] == '#') return false;

		stringstream ss(line);
		string part;

		getline(ss, part, ';');
		getline(ss, part, ','); // Week
		getline(ss, part, ','); // Second of week
		Rawdata->TimeStamp = stod(part);

		getline(ss, part, ','); // Status

		getline(ss, part, ','); Rawdata->Acc.Z = -stod(part) * convert_a * rate;
		getline(ss, part, ','); Rawdata->Acc.X = stod(part) * convert_a * rate;
		getline(ss, part, ','); Rawdata->Acc.Y = stod(part) * convert_a * rate;

		getline(ss, part, ','); Rawdata->Gyr.Z = -stod(part) * convert_g * rate;
		getline(ss, part, ','); Rawdata->Gyr.X = stod(part) * convert_g * rate;
		getline(ss, part, '*'); Rawdata->Gyr.Y = stod(part) * convert_g * rate;

		return true;
	}
	return false;
}

/* Read reference ground truth trajectory data */
bool ReadTruthData(ifstream& file, INSDataEpoch* Rawdata)
{
	string line;
	while (getline(file, line))
	{
		if (line.empty() || line[0] == '#') continue;

		stringstream ss(line);
		string part;
		vector<string> parts;
		while (ss >> part)
		{
			parts.push_back(part);
		}

		if (parts.size() == 10)
		{
			// Format: Time Lat Lon H Vn Ve Vd Roll Pitch Yaw
			Rawdata->TimeStamp = stod(parts[0]);
			Rawdata->blh.latitude = stod(parts[1]);
			Rawdata->blh.longitude = stod(parts[2]);
			Rawdata->blh.H = stod(parts[3]);
			Rawdata->vel.Vn = stod(parts[4]);
			Rawdata->vel.Ve = stod(parts[5]);
			Rawdata->vel.Vd = stod(parts[6]);
			Rawdata->pos.roll = stod(parts[7]);
			Rawdata->pos.pitch = stod(parts[8]);
			Rawdata->pos.yaw = stod(parts[9]);
			return true;
		}
		else if (parts.size() >= 11)
		{
			// Format with GPS Week: Week Time Lat Lon H Vn Ve Vd Roll Pitch Yaw
			Rawdata->TimeStamp = stod(parts[1]);
			Rawdata->blh.latitude = stod(parts[2]);
			Rawdata->blh.longitude = stod(parts[3]);
			Rawdata->blh.H = stod(parts[4]);
			Rawdata->vel.Vn = stod(parts[5]);
			Rawdata->vel.Ve = stod(parts[6]);
			Rawdata->vel.Vd = stod(parts[7]);
			Rawdata->pos.roll = stod(parts[8]);
			Rawdata->pos.pitch = stod(parts[9]);
			Rawdata->pos.yaw = stod(parts[10]);
			return true;
		}
	}
	return false;
}