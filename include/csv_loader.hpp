#pragma once
#include "sensors.hpp"
#include <vector>

std::vector<IMUData> loadIMUCSV(const std::string& filename);
std::vector<GPSData> loadGPSCSV(const std::string& filename);
