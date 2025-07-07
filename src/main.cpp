#include "csv_loader.hpp"
#include "SensorFusionEngine.hpp"
#include <iostream>

int main() {
    auto imu_data = loadIMUCSV("data/imu.csv");
    if(imu_data.empty()) {
        std::cerr << "No data loaded.\n";
        return 1;
    }
    auto gps_data = loadGPSCSV("data/gps.csv");
    if(gps_data.empty()) {
        std::cerr << "No data loaded.\n";
        return 1;
    }

    std::vector<FusedState> results;
    SensorFusionEngine fusionEngine;
    fusionEngine.handleGPS(gps_data[0]);
    fusionEngine.handleIMU(imu_data[0]);
    results.push_back(fusionEngine.getCurrentState());

    for(size_t i = 1; i < imu_data.size(); ++i) {
        fusionEngine.handleGPS(gps_data[i]);
        fusionEngine.handleIMU(imu_data[i]);
        results.push_back(fusionEngine.getCurrentState());
    }

    for (const auto& state : results) {
        std::cout
            << state.timestamp << ", "
            << state.orientation.w() << " "
            << state.orientation.x() << " "
            << state.orientation.y() << " "
            << state.orientation.z() << "\n";
    }

    return 0;
}
