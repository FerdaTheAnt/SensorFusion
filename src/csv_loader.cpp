#include "csv_loader.hpp"
#include "sensors.hpp"

#include <vector>
#include <fstream>
#include <sstream>

std::vector<IMUData> loadIMUCSV(const std::string& filename) {
    std::vector<IMUData> data;
    std::ifstream file(filename);
    std::string line;

    std::getline(file, line); //csv header

    while (std::getline(file, line)){
        std::stringstream ss(line);
        std::string token;
        IMUData entry;

        std::getline(ss, token, ',');
        entry.timestamp = std::stod(token);
        for(int i = 0; i<3; ++i){
            std::getline(ss, token, ',');
            entry.accel[i] = std::stod(token);
        }
        for(int i = 0; i<3; ++i){
            std::getline(ss, token, ',');
            entry.gyro[i] = std::stod(token);
        }

        data.push_back(entry);
    }
    return data;
}

std::vector<GPSData> loadGPSCSV(const std::string& filename) {
    std::vector<GPSData> data;
    std::ifstream file(filename);
    std::string line;

    std::getline(file, line);

    while(std::getline(file, line)){
        std::stringstream ss(line);
        std::string token;
        GPSData entry;

        std::getline(ss, token, ',');
        entry.timestamp = std::stod(token);
        for(int i = 0; i<3; ++i){
            std::getline(ss, token, ',');
            entry.position[i] = std::stod(token);
        }
        for(int i = 0; i<3; ++i){
            std::getline(ss, token, ',');
            entry.velocity[i] = std::stod(token);
        }

        data.push_back(entry);
    }
    return data;
}
